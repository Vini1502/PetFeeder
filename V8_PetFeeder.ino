// =========================================================================
// BIBLIOTECAS DO SISTEMA, REDE E PERIFÉRICOS
// =========================================================================
#include <WiFi.h>        // Gerencia a conexão Wi-Fi (modo Station) do ESP32
#include <WebServer.h>   // Cria o servidor HTTP local na porta 80 para a interface web
#include <Stepper.h>     // Controla o acionamento das bobinas do motor de passo
#include <Preferences.h> // Grava e recupera dados na memória não volátil (Flash/NVS)
#include <ESPmDNS.h>     // Permite acessar o dispositivo via http://petfeeder.local
#include "time.h"        // Manipulação de estruturas de data e hora (struct tm)
#include <sys/time.h>    // Permite ajustar o relógio interno do sistema (settimeofday)

// Bibliotecas do Bluetooth Low Energy (BLE) para comunicação UART sem fio
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>     // Descritor necessário para habilitar notificações (Notify) no BLE

Preferences preferences; // Objeto para persistir credenciais Wi-Fi após reinicialização

// Credenciais padrão de Wi-Fi (caso não haja rede salva na memória)
String ssid = "Galaxy S20";
String password = "kyojin12";

// Configuração do servidor NTP para sincronização automática de horário via internet
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 3600; // Fuso horário de Brasília (UTC-3)
const int daylightOffset_sec = 0;

// =========================================================================
// MAPEAMENTO DE PINOS E CONFIGURAÇÃO DE HARDWARE
// =========================================================================
#define TRIG_PIN 33         // Pino de disparo (Trigger) do sensor ultrassônico HC-SR04
#define ECHO_PIN 32         // Pino de leitura do retorno (Echo) do sensor ultrassônico
#define ALTO_FALANTE_PIN 21 // Pino PWM ligado à base do transistor 2N2222A (via resistor 1k)

// Pinos digitais conectados às entradas IN1..IN4 do driver ULN2003
#define IN1 19
#define IN2 18
#define IN3 5
#define IN4 16

const int PASSOS_POR_VOLTA = 2048; // Resolução do motor 28BYJ-48 com caixa de redução
// Sequência IN1, IN3, IN2, IN4 é necessária para a ordem correta de passo nesse driver
Stepper motorPasso(PASSOS_POR_VOLTA, IN1, IN3, IN2, IN4);

// Limites físicos de calibração do reservatório (em centímetros)
const float DIST_CHEIO = 4.0;  // Distância sensor-ração quando o pote está 100% cheio
const float DIST_VAZIO = 14.0; // Distância sensor-ração quando o pote está 0% (vazio)

// Variáveis globais de cache para evitar leituras bloqueantes a cada requisição HTTP
float ultimaDistancia = -1.0;
int ultimaPorcentagem = -1;

// Variáveis de controle do agendamento automático
int horaAgendada = 8;
int minutoAgendado = 0;
bool agendamentoAtivo = true;
bool jaAlimentouNoMinuto = false; // Trava para impedir múltiplos disparos no mesmo minuto

// Ponteiros e flags de estado da comunicação Bluetooth BLE
BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL; // Canal de envio (ESP32 -> Celular)
BLECharacteristic* pRxCharacteristic = NULL; // Canal de recepção (Celular -> ESP32)
bool deviceConnected = false;
bool oldDeviceConnected = false;
volatile bool triggerAlimentarBLE = false; // Flag volatile usada entre callback BLE e loop()

// Flags para troca assíncrona e segura de rede Wi-Fi via comandos BLE
volatile bool triggerConectarWiFi = false;
String pendenteSSID = "";
String pendenteSenha = "";

// UUIDs padrão do serviço Nordic UART (emula porta serial sobre BLE)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

WebServer server(80); // Instancia o servidor web na porta HTTP padrão (80)
unsigned long ultimaAtualizacaoSensor = 0; // Temporizador não bloqueante (millis)

// =========================================================================
// FUNÇÕES DE CONECTIVIDADE (BLE E WI-FI)
// =========================================================================

// Envia strings de status ou resposta para o aplicativo conectado via Bluetooth
void enviarMensagemBLE(String msg) {
  if (deviceConnected && pTxCharacteristic) {
    pTxCharacteristic->setValue(msg.c_str());
    pTxCharacteristic->notify(); // Dispara notificação push para o cliente BLE
  }
}

// Conecta o ESP32 à rede Wi-Fi e inicializa NTP e mDNS em caso de sucesso
bool conectarWiFi(String novoSSID, String novaSenha) {
  WiFi.disconnect();
  delay(500);

  WiFi.mode(WIFI_STA); // Configura o ESP32 como cliente Wi-Fi (Station)
  WiFi.begin(novoSSID.c_str(), novaSenha.c_str());

  Serial.print("A ligar Wi-Fi a ");
  Serial.println(novoSSID);

  // Aguarda até 10 segundos (20 tentativas x 500ms) pela conexão
  int tentativas = 0;
  while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
    delay(500);
    Serial.print(".");
    tentativas++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    // Sincroniza o relógio interno com o servidor NTP da internet
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // Inicia o resolvedor mDNS para acesso amigável na rede local
    if (MDNS.begin("petfeeder")) {
      Serial.println("mDNS ativo: http://petfeeder.local");
    }

    Serial.print("WiFi Conectado! IP: ");
    Serial.println(WiFi.localIP());

    String resp = "\n[Wi-Fi] Conectado!\n[IP Atual] " + WiFi.localIP().toString() + "\n";
    enviarMensagemBLE(resp);
    return true;
  } else {
    Serial.println("Falha no Wi-Fi!");
    String resp = "\n[Wi-Fi] Falha ao ligar a: " + novoSSID + "\n";
    enviarMensagemBLE(resp);
    return false;
  }
}

// =========================================================================
// FUNÇÕES DO SENSOR ULTRASSÔNICO E CÁLCULO DE NÍVEL
// =========================================================================

// Realiza uma medição pontual de distância usando pulso ultrassônico
float lerDistanciaUnica() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH); // Envia pulso sonoro de 10 microssegundos
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Mede o tempo de retorno do eco (timeout de 30ms limita alcance a ~5 metros)
  unsigned long duracao = pulseIn(ECHO_PIN, HIGH, 30000);

  // Descarta leituras nulas (timeout) ou menores que ~2 cm (zona cega do sensor)
  if (duracao == 0 || duracao < 115) {
    return -1.0;
  }
  // Converte tempo em distância: (tempo * velocidade do som 0.0343 cm/us) / 2 (ida e volta)
  return (duracao * 0.0343) / 2.0;
}

// Coleta 5 amostras e aplica filtro de mediana para eliminar ruídos de leitura
float lerDistancia() {
  const int NUM_AMOSTRAS = 5;
  float leituras[NUM_AMOSTRAS];
  int validas = 0;

  for (int i = 0; i < NUM_AMOSTRAS; i++) {
    float d = lerDistanciaUnica();
    if (d > 0) {
      leituras[validas++] = d;
    }
    if (i < NUM_AMOSTRAS - 1) {
      delay(35); // Pequeno intervalo entre pulsos para evitar ecos residuais
    }
  }

  if (validas == 0) {
    return -1.0; // Retorna erro caso todas as 5 amostras falhem
  }

  // Ordenação crescente (Selection Sort) para extrair o valor central (mediana)
  for (int i = 0; i < validas - 1; i++) {
    for (int j = i + 1; j < validas; j++) {
      if (leituras[j] < leituras[i]) {
        float temp = leituras[i];
        leituras[i] = leituras[j];
        leituras[j] = temp;
      }
    }
  }

  return leituras[validas / 2]; // Retorna a mediana das leituras válidas
}

// Converte a distância medida (cm) em porcentagem linear de preenchimento (0% a 100%)
int calcularPorcentagem(float dist) {
  if (dist < 0) return -1;
  if (dist <= DIST_CHEIO) return 100;
  if (dist >= DIST_VAZIO) return 0;

  float pct = ((DIST_VAZIO - dist) / (DIST_VAZIO - DIST_CHEIO)) * 100.0;
  return constrain((int)round(pct), 0, 100); // Garante que o resultado fique entre 0 e 100
}

// Atualiza as variáveis globais de distância e porcentagem
void atualizarLeituraSensor() {
  ultimaDistancia = lerDistancia();
  ultimaPorcentagem = calcularPorcentagem(ultimaDistancia);

  Serial.print("Distancia: ");
  Serial.print(ultimaDistancia);
  Serial.print(" cm | Nivel: ");
  Serial.print(ultimaPorcentagem);
  Serial.println("%");
}

// =========================================================================
// FUNÇÕES DE RELÓGIO, ÁUDIO, MOTOR E AGENDAMENTO
// =========================================================================

// Formata a hora atual do relógio interno do ESP32 no padrão "HH:MM:SS"
String obterHora() {
  struct tm timeinfo;
  // Verifica se a hora é válida (ano > 2020)
  if (!getLocalTime(&timeinfo) || timeinfo.tm_year < (2020 - 1900)) {
    return "--:--:--";
  }
  char horaBuff[10];
  strftime(horaBuff, sizeof(horaBuff), "%H:%M:%S", &timeinfo);
  return String(horaBuff);
}

// Toca a melodia "Victory Fanfare" (Final Fantasy) via PWM para chamar o pet
void emitirAlertaAlimentacao() {
  const int melodia[]  = { 2093, 2093, 2093, 2093, 1661, 1865, 2093, 1865, 2093 };
  const int duracoes[] = { 110,  110,  110,  330,  330,  330,  130,  110,  600 };
  const int pausas[]   = {  30,   30,   30,   30,   30,   30,   90,   20,   50 };

  const int totalNotas = sizeof(melodia) / sizeof(melodia[0]);

  for (int i = 0; i < totalNotas; i++) {
    tone(ALTO_FALANTE_PIN, melodia[i]); // Gera onda quadrada na frequência da nota
    delay(duracoes[i]);
    noTone(ALTO_FALANTE_PIN);           // Interrompe o som entre notas
    delay(pausas[i]);
  }
}

// Desenergiza todas as bobinas do motor para evitar aquecimento e consumo em repouso
void desligarMotorPasso() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Ciclo completo de alimentação: emite som, gira a rosca por 60s e atualiza o nível
void alimentar() {
  Serial.println("ALIMENTANDO!");
  emitirAlertaAlimentacao();

  motorPasso.setSpeed(14); // 14 RPM: velocidade máxima mantendo torque estável no 28BYJ-48
  unsigned long tempoInicio = millis();

  // Gira continuamente por 60 segundos (60000 ms)
  while (millis() - tempoInicio < 60000) {
    motorPasso.step(-32); // Passo negativo define o sentido de rotação do mecanismo
    yield();              // Libera tempo para tarefas internas do ESP32 (evita reset por Watchdog)
  }

  desligarMotorPasso();
  Serial.println("Alimentacao concluida.");
  atualizarLeituraSensor(); // Mede quanto de ração restou após dispensar
}

// Compara o horário atual com o horário programado e dispara a alimentação uma vez
void verificarAgendamento() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo) && timeinfo.tm_year > (2020 - 1900)) {
    if (agendamentoAtivo) {
      if (timeinfo.tm_hour == horaAgendada && timeinfo.tm_min == minutoAgendado) {
        if (!jaAlimentouNoMinuto) {
          alimentar();
          jaAlimentouNoMinuto = true; // Bloqueia novas execuções até o minuto mudar
        }
      } else {
        jaAlimentouNoMinuto = false; // Rearma a trava assim que o minuto avança
      }
    }
  }
}

// =========================================================================
// CLASSES E CALLBACKS DO BLUETOOTH BLE
// =========================================================================

// Gerencia eventos de conexão e desconexão de clientes Bluetooth
class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Dispositivo Bluetooth Conectado!");
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Dispositivo Bluetooth Desconectado.");
    }
};

// Interpreta comandos de texto enviados pelo celular na característica RX
class CharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = String(pCharacteristic->getValue().c_str());
      rxValue.trim();

      if (rxValue.length() > 0) {
        Serial.print("Comando Bluetooth Recebido: ");
        Serial.println(rxValue);

        String upperValue = rxValue;
        upperValue.toUpperCase();

        // Comando para dispensar ração (sinaliza flag para execução segura no loop)
        if (upperValue == "FEED" || upperValue == "ALIMENTAR" || upperValue == "1") {
          triggerAlimentarBLE = true;
          enviarMensagemBLE("A alimentar pet...\n");
        }
        // Comando de consulta do endereço IP atual
        else if (upperValue == "IP" || upperValue == "STATUS") {
          if (WiFi.status() == WL_CONNECTED) {
            enviarMensagemBLE("\n[IP Atual] " + WiFi.localIP().toString() + "\n");
          } else {
            enviarMensagemBLE("\n[IP] Wi-Fi Desconectado!\n");
          }
        }
        // Comando para configurar nova rede Wi-Fi no formato "WIFI:SSID,SENHA"
        else if (upperValue.startsWith("WIFI:")) {
          int commaIndex = rxValue.indexOf(',');
          if (commaIndex > 5) {
            pendenteSSID = rxValue.substring(5, commaIndex);
            pendenteSenha = rxValue.substring(commaIndex + 1);
            pendenteSSID.trim();
            pendenteSenha.trim();

            enviarMensagemBLE("\nA tentar ligar a rede: " + pendenteSSID + "...\n");
            triggerConectarWiFi = true; // Delega a reconexão para o loop principal
          } else {
            enviarMensagemBLE("\nErro! Formato correto: WIFI:NomeDaRede,SuaSenha\n");
          }
        }
        // Comandos para ajuste manual de relógio offline (TIME:AAAA,MM,DD,H,M,S ou EPOCH:...)
        else if (upperValue.startsWith("TIME:") || upperValue.startsWith("EPOCH:")) {
          int ano = 0, mes = 0, dia = 0, h = 0, m = 0, s = 0;

          if (sscanf(rxValue.c_str(), "TIME:%d,%d,%d,%d,%d,%d", &ano, &mes, &dia, &h, &m, &s) == 6) {
            struct tm timeinfo = {0};
            timeinfo.tm_year = ano - 1900;
            timeinfo.tm_mon  = mes - 1;
            timeinfo.tm_mday = dia;
            timeinfo.tm_hour = h;
            timeinfo.tm_min  = m;
            timeinfo.tm_sec  = s;

            time_t t = mktime(&timeinfo);
            struct timeval tv = { t, 0 };
            settimeofday(&tv, NULL); // Atualiza o RTC interno do ESP32
            enviarMensagemBLE("Data e Hora atualizadas via BLE!\n");
          }
          else if (sscanf(rxValue.c_str(), "TIME:%d:%d:%d", &h, &m, &s) >= 2) {
            struct tm timeinfo;
            if (!getLocalTime(&timeinfo)) {
              timeinfo.tm_year = 2026 - 1900;
              timeinfo.tm_mon  = 9;
              timeinfo.tm_mday = 20;
            }
            timeinfo.tm_hour = h;
            timeinfo.tm_min  = m;
            timeinfo.tm_sec  = s;

            time_t t = mktime(&timeinfo);
            struct timeval tv = { t, 0 };
            settimeofday(&tv, NULL);
            enviarMensagemBLE("Hora atualizada via BLE!\n");
          }
          else {
            int colonIndex = rxValue.indexOf(':');
            String epochStr = rxValue.substring(colonIndex + 1);
            epochStr.trim();
            long epoch = epochStr.toInt();

            if (epoch > 0) {
              struct timeval tv = { epoch, 0 };
              settimeofday(&tv, NULL);
              enviarMensagemBLE("Hora atualizada via BLE!\n");
            }
          }
        }
      }
    }
};

// =========================================================================
// INTERFACE WEB EMBUTIDA NA MEMÓRIA FLASH (PROGMEM)
// =========================================================================
// Armazena o código HTML/CSS/JS na Flash para economizar memória RAM do ESP32
const char INDEX_HTML[] PROGMEM =
"<!DOCTYPE html>"
"<html lang='pt'>"
"<head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0, user-scalable=no'>"
"<title>Pet Feeder</title>"
"<style>"
"* { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; }"
"body { background-color: #f7f1eb; color: #2d3748; padding: 16px; display: flex; justify-content: center; }"
".app-container { width: 100%; max-width: 400px; display: flex; flex-direction: column; gap: 14px; }"
".header { display: flex; align-items: center; justify-content: space-between; padding: 4px 0; }"
".header-info { display: flex; align-items: center; gap: 12px; }"
".logo-icon { width: 44px; height: 44px; background: #ff6b3b; border-radius: 50%; display: flex; align-items: center; justify-content: center; color: white; box-shadow: 0 4px 10px rgba(255,107,59,0.3); }"
".header-text h1 { font-size: 18px; font-weight: 700; }"
".header-text p { font-size: 12px; color: #a0aec0; }"
".gear-btn { width: 38px; height: 38px; border-radius: 50%; background: white; border: none; display: flex; align-items: center; justify-content: center; color: #4a5568; box-shadow: 0 2px 6px rgba(0,0,0,0.05); }"
".alert-banner { background: #fff5f5; color: #e53e3e; padding: 10px 14px; border-radius: 12px; font-size: 13px; font-weight: 500; display: flex; align-items: center; gap: 8px; border: 1px solid #fed7d7; transition: all 0.3s; }"
".alert-banner.hidden { display: none; }"
".card { background: white; border-radius: 22px; padding: 18px; box-shadow: 0 4px 15px rgba(0,0,0,0.03); }"
".card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 12px; }"
".card-title { font-size: 14px; font-weight: 600; color: #4a5568; }"
".ruler-text { font-size: 12px; color: #a0aec0; display: flex; align-items: center; gap: 4px; }"
".gauge-wrapper { display: flex; justify-content: center; align-items: center; padding: 10px 0; }"
".gauge-circle { width: 140px; height: 140px; border-radius: 50%; background: conic-gradient(#e53e3e 0deg, #edf2f7 0deg); display: flex; align-items: center; justify-content: center; position: relative; transition: background 0.5s ease; }"
".gauge-inner { width: 114px; height: 114px; background: white; border-radius: 50%; display: flex; flex-direction: column; align-items: center; justify-content: center; }"
".gauge-value { font-size: 28px; font-weight: 800; color: #e53e3e; line-height: 1; }"
".gauge-label { font-size: 11px; color: #718096; margin-top: 4px; font-weight: 500; }"
".grid-2 { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }"
".mini-card { background: white; border-radius: 18px; padding: 14px; box-shadow: 0 4px 15px rgba(0,0,0,0.03); }"
".mini-header { display: flex; align-items: center; gap: 6px; font-size: 10px; font-weight: 700; color: #a0aec0; text-transform: uppercase; margin-bottom: 8px; }"
".mini-value { font-size: 18px; font-weight: 800; color: #2d3748; }"
".dispense-card { text-align: center; display: flex; flex-direction: column; align-items: center; padding: 24px 18px; }"
".big-paw-btn { width: 100px; height: 100px; border-radius: 50%; background: linear-gradient(135deg, #ff6b3b, #ff8c5a); border: none; display: flex; align-items: center; justify-content: center; color: white; cursor: pointer; box-shadow: 0 8px 20px rgba(255,107,59,0.4); transition: transform 0.1s; margin-bottom: 12px; }"
".big-paw-btn:active { transform: scale(0.95); }"
".btn-label { font-size: 14px; font-weight: 700; color: #2d3748; }"
".schedule-header { display: flex; align-items: center; gap: 8px; font-size: 14px; font-weight: 700; color: #2d3748; margin-bottom: 16px; }"
".time-inputs { display: flex; align-items: center; justify-content: center; gap: 8px; margin-bottom: 16px; }"
".time-input { width: 68px; height: 50px; background: #f8fafc; border: 1px solid #e2e8f0; border-radius: 12px; font-size: 22px; font-weight: 700; text-align: center; color: #2d3748; outline: none; }"
".save-btn { width: 100%; height: 46px; background: #6db33f; border: none; border-radius: 12px; color: white; font-size: 14px; font-weight: 700; display: flex; align-items: center; justify-content: center; gap: 6px; cursor: pointer; box-shadow: 0 4px 12px rgba(109,179,63,0.3); }"
".footer { text-align: center; font-size: 11px; color: #cbd5e0; margin-top: 6px; }"
"</style>"
"</head>"
"<body>"
"<div class='app-container'>"
"<div class='header'>"
"<div class='header-info'>"
"<div class='logo-icon'>"
"<svg width='22' height='22' viewBox='0 0 24 24' fill='currentColor'>"
"<path d='M12,11.5A2.5,2.5 0 0,1 9.5,9A2.5,2.5 0 0,1 12,6.5A2.5,2.5 0 0,1 14.5,9A2.5,2.5 0 0,1 12,11.5"
"M12,2A10,10 0 0,0 2,12A10,10 0 0,0 12,22A10,10 0 0,0 22,12A10,10 0 0,0 12,2"
"M5,10A2,2 0 0,1 7,12A2,2 0 0,1 5,14A2,2 0 0,1 3,12A2,2 0 0,1 5,10"
"M19,10A2,2 0 0,1 21,12A2,2 0 0,1 19,14A2,2 0 0,1 17,12A2,2 0 0,1 19,10"
"M8.5,15.5C8.5,14.67 10.07,14 12,14C13.93,14 15.5,14.67 15.5,15.5C15.5,18.5 12,19.5 12,19.5C12,19.5 8.5,18.5 8.5,15.5Z'/>"
"</svg>"
"</div>"
"<div class='header-text'><h1>Pet Feeder</h1><p id='ipText'>Localizando...</p></div>"
"</div>"
"<button class='gear-btn'>"
"<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'>"
"<circle cx='12' cy='12' r='3'></circle>"
"<path d='M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z'></path>"
"</svg>"
"</button>"
"</div>"
"<div id='connAlert' class='alert-banner'>"
"<svg width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'>"
"<circle cx='12' cy='12' r='10'></circle><line x1='12' y1='8' x2='12' y2='12'></line><line x1='12' y1='16' x2='12.01' y2='16'></line>"
"</svg>"
"Sem ligação ao alimentador"
"</div>"
"<div class='card'>"
"<div class='card-header'>"
"<span class='card-title'>Nível de Ração</span>"
"<span class='ruler-text'>"
"<svg width='14' height='14' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2'>"
"<path d='M21.3 15.3a2.4 2.4 0 0 1 0 3.4l-2.6 2.6a2.4 2.4 0 0 1-3.4 0L2.7 8.7a2.4 2.4 0 0 1 0-3.4l2.6-2.6a2.4 2.4 0 0 1 3.4 0l12.6 12.6z'></path>"
"<path d='m14.5 12.5 2-2'></path><path d='m11.5 9.5 2-2'></path><path d='m8.5 6.5 2-2'></path><path d='m17.5 15.5 2-2'></path>"
"</svg>"
"<span id='distVal'>-- cm</span>"
"</span>"
"</div>"
"<div class='gauge-wrapper'>"
"<div id='gaugeCircle' class='gauge-circle'>"
"<div class='gauge-inner'>"
"<span id='pctVal' class='gauge-value'>0%</span>"
"<span id='pctLabel' class='gauge-label'>Ração baixa</span>"
"</div>"
"</div>"
"</div>"
"</div>"
"<div class='grid-2'>"
"<div class='mini-card'>"
"<div class='mini-header'>"
"<svg width='14' height='14' viewBox='0 0 24 24' fill='none' stroke='#3182ce' stroke-width='2'>"
"<circle cx='12' cy='12' r='10'></circle><polyline points='12 6 12 12 16 14'></polyline>"
"</svg>"
"Relógio"
"</div>"
"<div id='horaVal' class='mini-value'>--:--:--</div>"
"</div>"
"<div class='mini-card'>"
"<div class='mini-header'>"
"<svg width='14' height='14' viewBox='0 0 24 24' fill='none' stroke='#dd6b20' stroke-width='2'>"
"<circle cx='12' cy='12' r='10'></circle><polyline points='12 6 12 12 16 10'></polyline>"
"</svg>"
"Próxima Refeição"
"</div>"
"<div id='proxVal' class='mini-value'>—</div>"
"</div>"
"</div>"
"<div class='card dispense-card'>"
"<button class='big-paw-btn' onclick='alimentarAgora()'>"
"<svg width='42' height='42' viewBox='0 0 24 24' fill='currentColor'>"
"<path d='M12,11.5A2.5,2.5 0 0,1 9.5,9A2.5,2.5 0 0,1 12,6.5A2.5,2.5 0 0,1 14.5,9A2.5,2.5 0 0,1 12,11.5"
"M12,2A10,10 0 0,0 2,12A10,10 0 0,0 12,22A10,10 0 0,0 22,12A10,10 0 0,0 12,2"
"M5,10A2,2 0 0,1 7,12A2,2 0 0,1 5,14A2,2 0 0,1 3,12A2,2 0 0,1 5,10"
"M19,10A2,2 0 0,1 21,12A2,2 0 0,1 19,14A2,2 0 0,1 17,12A2,2 0 0,1 19,10"
"M8.5,15.5C8.5,14.67 10.07,14 12,14C13.93,14 15.5,14.67 15.5,15.5C15.5,18.5 12,19.5 12,19.5C12,19.5 8.5,18.5 8.5,15.5Z'/>"
"</svg>"
"</button>"
"<span class='btn-label'>Dispensar Ração Agora</span>"
"</div>"
"<div class='card'>"
"<div class='schedule-header'>"
"<svg width='18' height='18' viewBox='0 0 24 24' fill='none' stroke='#6db33f' stroke-width='2'>"
"<circle cx='12' cy='12' r='10'></circle><polyline points='12 6 12 12 16 14'></polyline>"
"</svg>"
"Agendar Refeição"
"</div>"
"<div class='time-inputs'>"
"<input type='number' id='inHora' class='time-input' min='0' max='23' value='08'>"
"<span style='font-size:22px; font-weight:700; color:#a0aec0;'>:</span>"
"<input type='number' id='inMin' class='time-input' min='0' max='59' value='00'>"
"</div>"
"<button class='save-btn' onclick='salvarAgendamento()'>"
"<svg width='16' height='16' viewBox='0 0 24 24' fill='none' stroke='currentColor' stroke-width='2.5'>"
"<line x1='12' y1='5' x2='12' y2='19'></line><line x1='5' y1='12' x2='19' y2='12'></line>"
"</svg>"
"Guardar Agendamento"
"</button>"
"</div>"
"<div class='footer'>Alimentador Inteligente • ESP32</div>"
"</div>"
"<script>"
"document.getElementById('ipText').innerText = window.location.hostname;"
// Sincroniza automaticamente o relógio do ESP32 usando o horário do navegador ao abrir a página
"fetch('/settime?epoch=' + Math.floor(Date.now() / 1000)).catch(()=>{});"
// Consulta a API /status a cada 2s e atualiza o medidor circular (conic-gradient) e os textos
"function atualizarStatus() {"
"  fetch('/status')"
"    .then(res => { if (!res.ok) throw new Error(); return res.json(); })"
"    .then(data => {"
"      document.getElementById('connAlert').classList.add('hidden');"
"      const pct = data.porcentagem;"
"      document.getElementById('pctVal').innerText = pct < 0 ? '--%' : pct + '%';"
"      document.getElementById('distVal').innerText = data.distancia < 0 ? 'Erro' : data.distancia.toFixed(1) + ' cm';"
"      document.getElementById('horaVal').innerText = data.hora;"
"      const h = String(data.horaAgendada).padStart(2, '0');"
"      const m = String(data.minutoAgendado).padStart(2, '0');"
"      document.getElementById('proxVal').innerText = `${h}:${m}`;"
"      let cor = '#e53e3e'; let txt = 'Ração baixa';"
"      if (pct > 50) { cor = '#6db33f'; txt = 'Ração suficiente'; }"
"      else if (pct > 20) { cor = '#dd6b20'; txt = 'Ração média'; }"
"      const deg = pct < 0 ? 0 : (pct / 100) * 360;"
"      document.getElementById('gaugeCircle').style.background = `conic-gradient(${cor} ${deg}deg, #edf2f7 ${deg}deg)`;"
"      document.getElementById('pctVal').style.color = cor;"
"      document.getElementById('pctLabel').innerText = pct < 0 ? 'Erro no Sensor' : txt;"
"    })"
"    .catch(() => { document.getElementById('connAlert').classList.remove('hidden'); });"
"}"
"function alimentarAgora() { fetch('/alimentar').then(() => atualizarStatus()); }"
"function salvarAgendamento() {"
"  const h = document.getElementById('inHora').value;"
"  const m = document.getElementById('inMin').value;"
"  fetch(`/agendar?hora=${h}&minuto=${m}`).then(() => atualizarStatus());"
"}"
"setInterval(atualizarStatus, 2000);" // Polling automático a cada 2000 ms
"atualizarStatus();"
"</script>"
"</body>"
"</html>";

// =========================================================================
// HANDLERS DAS ROTAS HTTP (API REST DO SERVIDOR WEB)
// =========================================================================

// Rota principal ("/"): entrega a página HTML completa ao navegador
void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

// Rota "/status": monta e envia o JSON com telemetria atualizada para o frontend
void handleStatusJSON() {
  String hora = obterHora();

  String json = "{";
  json += "\"hora\":\"" + hora + "\",";
  json += "\"porcentagem\":" + String(ultimaPorcentagem) + ",";
  json += "\"distancia\":" + String(ultimaDistancia, 1) + ",";
  json += "\"horaAgendada\":" + String(horaAgendada) + ",";
  json += "\"minutoAgendado\":" + String(minutoAgendado);
  json += "}";

  server.send(200, "application/json", json);
}

// Rota "/alimentar": aciona a dispensação imediata via botão web
void handleAlimentar() {
  alimentar();
  server.send(200, "text/plain", "OK");
}

// Rota "/agendar": recebe ?hora=X&minuto=Y e atualiza o horário programado
void handleAgendar() {
  if (server.hasArg("hora") && server.hasArg("minuto")) {
    horaAgendada = server.arg("hora").toInt();
    minutoAgendado = server.arg("minuto").toInt();
    agendamentoAtivo = true;
    jaAlimentouNoMinuto = false;
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// Rota "/settime": recebe o timestamp Unix (?epoch=...) do navegador e acerta o relógio
void handleSetTime() {
  if (server.hasArg("epoch")) {
    long epoch = server.arg("epoch").toInt();
    struct timeval tv = { epoch, 0 };
    settimeofday(&tv, NULL);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// =========================================================================
// INICIALIZAÇÃO (SETUP) E CICLO PRINCIPAL (LOOP)
// =========================================================================

void setup() {
  Serial.begin(115200);

  // Configura os pinos físicos de entrada e saída
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(ALTO_FALANTE_PIN, OUTPUT);
  noTone(ALTO_FALANTE_PIN);

  // Recupera SSID e senha salvos na memória flash (usa valores padrão se vazio)
  preferences.begin("wifi_config", false);
  ssid = preferences.getString("ssid", "Galaxy S20");
  password = preferences.getString("password", "kyojin12");

  conectarWiFi(ssid, password);

  // Inicializa a pilha Bluetooth BLE e cria o servidor UART
  BLEDevice::init("PetFeeder-ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Configura característica RX (escrita pelo celular) e TX (notificação para o celular)
  pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  // Inicia o anúncio (Advertising) para que o ESP32 fique visível no scan Bluetooth
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Bluetooth BLE Ativo: PetFeeder-ESP32");

  // Registra as rotas HTTP e inicia o servidor web
  server.on("/", handleRoot);
  server.on("/status", handleStatusJSON);
  server.on("/alimentar", handleAlimentar);
  server.on("/agendar", handleAgendar);
  server.on("/settime", handleSetTime);

  server.begin();
  atualizarLeituraSensor(); // Faz a primeira leitura do nível de ração ao ligar
}

void loop() {
  server.handleClient();   // Atende requisições HTTP pendentes dos navegadores
  verificarAgendamento();  // Verifica se chegou a hora programada para alimentar

  // Gerencia a transição de nova conexão BLE e envia o IP atual ao cliente
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = true;
    delay(500);
    if (WiFi.status() == WL_CONNECTED) {
      enviarMensagemBLE("\n[PetFeeder Ligado]\n[IP Atual] " + WiFi.localIP().toString() + "\n");
    } else {
      enviarMensagemBLE("\n[PetFeeder Ligado]\n[Wi-Fi] Nao conectado\n");
    }
  }

  // Reinicia o anúncio BLE após desconexão para permitir futuros pareamentos
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = false;
  }

  // Executa a dispensação solicitada via BLE fora da interrupção de escrita
  if (triggerAlimentarBLE) {
    triggerAlimentarBLE = false;
    alimentar();
  }

  // Executa a troca de rede Wi-Fi solicitada via BLE e salva na Flash se conectar
  if (triggerConectarWiFi) {
    triggerConectarWiFi = false;
    if (conectarWiFi(pendenteSSID, pendenteSenha)) {
      preferences.putString("ssid", pendenteSSID);
      preferences.putString("password", pendenteSenha);
      ssid = pendenteSSID;
      password = pendenteSenha;
    }
  }

  // Atualiza a leitura do sensor a cada 1 segundo (1000 ms) sem bloquear o loop
  if (millis() - ultimaAtualizacaoSensor >= 1000) {
    ultimaAtualizacaoSensor = millis();
    atualizarLeituraSensor();

    // Envia telemetria periódica da porcentagem de ração via Bluetooth se conectado
    if (deviceConnected && pTxCharacteristic) {
      String bleData = "RACAO:" + String(ultimaPorcentagem) + "%\n";
      pTxCharacteristic->setValue(bleData.c_str());
      pTxCharacteristic->notify();
    }
  }
}