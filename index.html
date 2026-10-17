// =====================================================================
//  CONTROLE DE MOTOR - L298N + LCD 16x2 + painel web  (firmware v3.0)
//
//  Funciona com as ligações atuais (nenhum fio novo é necessário).
//  Use junto com o arquivo painel_motor_v3.html
//
//  Opções (mude para 1 / 0 se precisar):
//    USAR_WATCHDOG  1 = reinicia o Arduino se o programa travar
//    USAR_DS18B20   1 = usa sonda DS18B20 em vez do TMP36
//                       (precisa das bibliotecas OneWire e DallasTemperature,
//                        sonda no pino A2 com resistor de 4,7 kΩ ao 5V)
// =====================================================================

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <avr/wdt.h>

#define VERSAO_FW "3.0"
#define USAR_WATCHDOG 1
#define USAR_DS18B20 0

#if USAR_DS18B20
  #include <OneWire.h>
  #include <DallasTemperature.h>
#endif

// =============================
// MOTOR - L298N (ENA com jumper colocado)
// IN1 e IN2 estão em pinos PWM (9 e 10): permitem velocidade e rampa
// =============================
const int IN1 = 9;
const int IN2 = 10;

// =============================
// BOTÕES (S1 = DIREITA | S2 = ESQUERDA | S3 = STOP)
// =============================
const int BOTAO_DIREITA = 8;
const int BOTAO_ESQUERDA = 6;
const int BOTAO_STOP = 7;

// =============================
// ENTRADAS ANALÓGICAS / SENSORES
// =============================
const int POT_TEMPO = A0;
const int SENSOR_TEMP = A1;     // TMP36
const int PINO_DS18B20 = A2;    // só se USAR_DS18B20 = 1

// =============================
// LCD: RS, E, DB4, DB5, DB6, DB7
// =============================
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

#if USAR_DS18B20
OneWire oneWire(PINO_DS18B20);
DallasTemperature sensorDS(&oneWire);
unsigned long ultimaLeituraDS = 0;
#endif

// =============================
// CONFIGURAÇÃO SALVA NA EEPROM
// =============================
struct Config {
  uint16_t magic;
  uint8_t  velocidade;          // 10..100 %
  uint8_t  rampaDecimos;        // 0..50  (0,0 a 5,0 s)
  uint16_t tempoMaxS;           // 10..3600 s
  bool     alarmeAtivo;
  bool     alarmeParar;         // true = bloqueia/interrompe o motor em alarme
  int16_t  tMinX10;             // faixa de temperatura (°C x 10)
  int16_t  tMaxX10;
  int16_t  offsetX100;          // calibração da temperatura (°C x 100)
  bool     tempoRemoto;         // true = tempo definido pelo painel
  uint16_t tempoRemotoDecimos;
};

const uint16_t MAGIC = 0xF3A5;
Config cfg;

void configPadrao() {
  cfg.magic = MAGIC;
  cfg.velocidade = 100;
  cfg.rampaDecimos = 0;
  cfg.tempoMaxS = 60;
  cfg.alarmeAtivo = false;
  cfg.alarmeParar = false;
  cfg.tMinX10 = 300;
  cfg.tMaxX10 = 350;
  cfg.offsetX100 = 0;
  cfg.tempoRemoto = false;
  cfg.tempoRemotoDecimos = 20;
}

void salvarConfig() {
  EEPROM.put(0, cfg);
}

void carregarConfig() {
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC) {
    configPadrao();
    salvarConfig();
    return;
  }
  // Garante valores válidos
  cfg.velocidade = constrain(cfg.velocidade, 10, 100);
  cfg.rampaDecimos = constrain(cfg.rampaDecimos, 0, 50);
  cfg.tempoMaxS = constrain(cfg.tempoMaxS, 10, 3600);
  cfg.offsetX100 = constrain(cfg.offsetX100, -1000, 1000);
  if (cfg.tempoRemotoDecimos < 1) cfg.tempoRemotoDecimos = 1;
  if (cfg.tempoRemotoDecimos > cfg.tempoMaxS * 10UL) cfg.tempoRemotoDecimos = cfg.tempoMaxS * 10UL;
}

// =============================
// ESTADO DO MOTOR
// =============================
bool motorLigado = false;
int direcao = 0;                       // 1 = direita, 2 = esquerda
unsigned long inicioMotor = 0;
unsigned long duracaoCicloMs = 0;      // fixa no início do ciclo
unsigned long decimosCiclo = 0;

unsigned long ciclos = 0;

int estadoAnteriorDireita = HIGH;
int estadoAnteriorEsquerda = HIGH;

// =============================
// EMERGÊNCIA / ALARME
// =============================
bool emergenciaWeb = false;
bool emergenciaAtiva = false;
bool alarmeTemp = false;
bool alarmeAnterior = false;
bool bloqueioAlarme = false;

// =============================
// SEQUÊNCIA DE CICLOS
// =============================
bool seqAtiva = false;
int seqModo = 1;                       // 1 = direita, 2 = esquerda, 3 = alternar
int seqProximaDir = 1;
unsigned int seqRestantes = 0;
unsigned long seqIntervaloMs = 0;
unsigned long seqProximoMs = 0;

// =============================
// COMUNICAÇÃO COM O PAINEL
// =============================
int comandoWeb = 0;                    // 1 = direita, 2 = esquerda
String bufferSerial = "";
unsigned long ultimoEnvio = 0;
float temperaturaBruta = NAN;


// =============================
// FUNÇÕES DO MOTOR
// =============================
void pararMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

// duty: 0 a 255 (PWM no pino de entrada ativo; o outro fica em LOW)
void aplicarMotor(int dir, int duty) {
  if (dir == 0 || duty <= 0) {
    pararMotor();
    return;
  }
  if (dir == 1) {
    digitalWrite(IN2, LOW);
    analogWrite(IN1, duty);
  } else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, duty);
  }
}

// Rampa de partida e de parada dentro do tempo programado
void atualizarMotor(unsigned long decorrido) {
  unsigned long restante = duracaoCicloMs - decorrido;
  unsigned long rampaMs = cfg.rampaDecimos * 100UL;
  unsigned long maxRampa = duracaoCicloMs / 2;
  if (rampaMs > maxRampa) rampaMs = maxRampa;

  float fator = 1.0;
  if (rampaMs > 0) {
    unsigned long m = (decorrido < restante) ? decorrido : restante;
    if (m < rampaMs) fator = (float)m / (float)rampaMs;
  }

  int duty = (int)(255.0 * cfg.velocidade / 100.0 * fator);
  aplicarMotor(direcao, duty);
}

void imprimirTempo(unsigned long decimos) {
  Serial.print(decimos / 10);
  Serial.print(".");
  Serial.print(decimos % 10);
}

void iniciarCiclo(int dir, unsigned long decimos, float temperatura) {
  motorLigado = true;
  direcao = dir;
  inicioMotor = millis();
  decimosCiclo = decimos;
  duracaoCicloMs = decimos * 100UL;

  atualizarMotor(0);

  Serial.println();
  Serial.println("=== NOVO CICLO ===");

  Serial.print("Ciclo: ");
  Serial.println(ciclos + 1);

  Serial.print("Direcao: ");
  Serial.println(dir == 1 ? "DIREITA" : "ESQUERDA");

  Serial.print("Tempo programado: ");
  imprimirTempo(decimos);
  Serial.println(" s");

  Serial.print("Velocidade: ");
  Serial.print(cfg.velocidade);
  Serial.println(" %");

  Serial.print("Temperatura inicial: ");
  Serial.print(temperatura, 1);
  Serial.println(" C");

  Serial.println("==================");
}

void tentarIniciar(int dir, unsigned long decimos, float temperatura) {
  if (motorLigado) return;
  if (seqAtiva) {
    Serial.println("RECUSADO;sequencia em andamento");
    return;
  }
  if (bloqueioAlarme) {
    Serial.println("RECUSADO;alarme de temperatura");
    return;
  }
  iniciarCiclo(dir, decimos, temperatura);
}

void cancelarSequencia() {
  seqAtiva = false;
  seqRestantes = 0;
}


// =============================
// TEMPERATURA
// =============================
float lerTemperatura() {
#if USAR_DS18B20
  if (millis() - ultimaLeituraDS >= 1000UL) {
    float t = sensorDS.getTempCByIndex(0);
    sensorDS.requestTemperatures();
    ultimaLeituraDS = millis();
    if (t > -100.0) temperaturaBruta = t;
    else temperaturaBruta = NAN;           // sonda desconectada
  }
#else
  analogRead(SENSOR_TEMP);                 // descarta a primeira leitura
  long soma = 0;
  for (int i = 0; i < 8; i++) {
    soma += analogRead(SENSOR_TEMP);
  }
  float tensao = (soma / 8.0) * (5.0 / 1023.0);
  temperaturaBruta = (tensao - 0.5) * 100.0;
#endif

  if (isnan(temperaturaBruta)) return NAN;
  return temperaturaBruta + cfg.offsetX100 / 100.0;
}


// =============================
// COMUNICAÇÃO: ENVIO
// =============================
void enviarVersao() {
  Serial.print("VERSAO;");
  Serial.println(VERSAO_FW);
}

// CONFIG;velocidade;rampa_s;tempoMax_s;alarmeAtivo;tMin;tMax;parar;offset;sensor
void enviarConfig() {
  Serial.print("CONFIG;");
  Serial.print(cfg.velocidade);
  Serial.print(";");
  Serial.print(cfg.rampaDecimos / 10.0, 1);
  Serial.print(";");
  Serial.print(cfg.tempoMaxS);
  Serial.print(";");
  Serial.print(cfg.alarmeAtivo ? 1 : 0);
  Serial.print(";");
  Serial.print(cfg.tMinX10 / 10.0, 1);
  Serial.print(";");
  Serial.print(cfg.tMaxX10 / 10.0, 1);
  Serial.print(";");
  Serial.print(cfg.alarmeParar ? 1 : 0);
  Serial.print(";");
  Serial.print(cfg.offsetX100 / 100.0, 2);
  Serial.print(";");
#if USAR_DS18B20
  Serial.println("D");
#else
  Serial.println("T");
#endif
}

// DADOS;temp;tempo_s;estado;ciclos;modoTempo;velocidade;alarme;seqRestantes;seqProximoEm_s
// estado: P = parado, D = direita, E = esquerda, S = emergência
// modoTempo: P = potenciômetro, W = definido pelo painel
void enviarDados(float temperatura, unsigned long decimos) {
  if (millis() - ultimoEnvio < 500) return;
  ultimoEnvio = millis();

  char estado;
  if (emergenciaAtiva) estado = 'S';
  else if (!motorLigado) estado = 'P';
  else if (direcao == 1) estado = 'D';
  else estado = 'E';

  unsigned long seqProxS = 0;
  if (seqAtiva && !motorLigado) {
    long falta = (long)(seqProximoMs - millis());
    if (falta > 0) seqProxS = falta / 1000;
  }

  Serial.print("DADOS;");
  Serial.print(temperatura, 1);
  Serial.print(";");
  Serial.print(decimos / 10.0, 1);
  Serial.print(";");
  Serial.print(estado);
  Serial.print(";");
  Serial.print(ciclos);
  Serial.print(";");
  Serial.print(cfg.tempoRemoto ? 'W' : 'P');
  Serial.print(";");
  Serial.print(cfg.velocidade);
  Serial.print(";");
  Serial.print(alarmeTemp ? 1 : 0);
  Serial.print(";");
  Serial.print(seqAtiva ? seqRestantes : 0);
  Serial.print(";");
  Serial.println(seqProxS);
}


// =============================
// COMUNICAÇÃO: RECEBIMENTO
//
//  D / E            ciclo para direita / esquerda
//  S / L            emergência / liberar
//  T<seg>           tempo em segundos (painel)        P = volta ao potenciômetro
//  V<10-100>        velocidade em %
//  R<seg>           rampa de partida/parada (0 a 5)
//  M<seg>           tempo máximo permitido (10 a 3600)
//  A<at>,<min>,<max>,<parar>   alarme de temperatura (at e parar: 0/1)
//  C<graus>         ajuste (offset) de calibração da temperatura
//  Q<D|E|A>,<n>,<intervalo_s>  sequência de n ciclos (A = alternar)
//  X                cancela a sequência
//  G                pede versão e configuração
// =============================
String campo(const String &s, int n) {
  int ini = 0;
  for (int i = 0; i < n; i++) {
    int v = s.indexOf(',', ini);
    if (v < 0) return "";
    ini = v + 1;
  }
  int fim = s.indexOf(',', ini);
  if (fim < 0) fim = s.length();
  return s.substring(ini, fim);
}

void processarComando(String cmd) {
  char tipo = cmd.charAt(0);
  String args = cmd.substring(1);

  switch (tipo) {

    case 'D':
      comandoWeb = 1;
      break;

    case 'E':
      comandoWeb = 2;
      break;

    case 'S':
      emergenciaWeb = true;
      break;

    case 'L':
      emergenciaWeb = false;
      break;

    case 'T': {
      float seg = args.toFloat();
      long d = (long)(seg * 10.0 + 0.5);
      long maxD = (long)cfg.tempoMaxS * 10L;
      d = constrain(d, 1, maxD);
      cfg.tempoRemotoDecimos = d;
      cfg.tempoRemoto = true;
      salvarConfig();
      enviarConfig();
      break;
    }

    case 'P':
      cfg.tempoRemoto = false;
      salvarConfig();
      enviarConfig();
      break;

    case 'V': {
      int v = constrain(args.toInt(), 10, 100);
      cfg.velocidade = v;
      salvarConfig();
      enviarConfig();
      break;
    }

    case 'R': {
      float seg = args.toFloat();
      int d = (int)(seg * 10.0 + 0.5);
      cfg.rampaDecimos = constrain(d, 0, 50);
      salvarConfig();
      enviarConfig();
      break;
    }

    case 'M': {
      int s = constrain(args.toInt(), 10, 3600);
      cfg.tempoMaxS = s;
      if (cfg.tempoRemotoDecimos > cfg.tempoMaxS * 10UL) {
        cfg.tempoRemotoDecimos = cfg.tempoMaxS * 10UL;
      }
      salvarConfig();
      enviarConfig();
      break;
    }

    case 'A': {
      int ativo = campo(args, 0).toInt();
      float tmin = campo(args, 1).toFloat();
      float tmax = campo(args, 2).toFloat();
      int parar = campo(args, 3).toInt();
      if (tmin >= tmax) {
        Serial.println("RECUSADO;faixa de temperatura invalida");
        break;
      }
      cfg.alarmeAtivo = (ativo != 0);
      cfg.tMinX10 = (int)(tmin * 10.0 + (tmin < 0 ? -0.5 : 0.5));
      cfg.tMaxX10 = (int)(tmax * 10.0 + (tmax < 0 ? -0.5 : 0.5));
      cfg.alarmeParar = (parar != 0);
      salvarConfig();
      enviarConfig();
      break;
    }

    case 'C': {
      float off = constrain(args.toFloat(), -10.0, 10.0);
      cfg.offsetX100 = (int)(off * 100.0 + (off < 0 ? -0.5 : 0.5));
      salvarConfig();
      enviarConfig();
      break;
    }

    case 'Q': {
      String modo = campo(args, 0);
      int n = campo(args, 1).toInt();
      float intervalo = campo(args, 2).toFloat();
      char m = modo.charAt(0);

      if (motorLigado || seqAtiva || emergenciaAtiva || bloqueioAlarme) {
        Serial.println("RECUSADO;sequencia nao pode iniciar agora");
        break;
      }
      if (n < 1 || (m != 'D' && m != 'E' && m != 'A')) {
        Serial.println("RECUSADO;sequencia invalida");
        break;
      }
      n = constrain(n, 1, 999);
      intervalo = constrain(intervalo, 0.0, 3600.0);

      seqModo = (m == 'D') ? 1 : (m == 'E') ? 2 : 3;
      seqProximaDir = (m == 'E') ? 2 : 1;
      seqRestantes = n;
      seqIntervaloMs = (unsigned long)(intervalo * 1000.0);
      seqProximoMs = millis();
      seqAtiva = true;

      Serial.print("SEQUENCIA iniciada;");
      Serial.print(n);
      Serial.println(" ciclos");
      break;
    }

    case 'X':
      if (seqAtiva) {
        cancelarSequencia();
        Serial.println("SEQUENCIA cancelada;pelo painel");
      }
      break;

    case 'G':
      enviarVersao();
      enviarConfig();
      break;
  }
}

void lerComandosSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      bufferSerial.trim();
      if (bufferSerial.length() > 0) {
        processarComando(bufferSerial);
      }
      bufferSerial = "";
    }
    else if (bufferSerial.length() < 24) {
      bufferSerial += c;
    }
  }
}


// =============================
// SETUP
// =============================
void setup() {

#if USAR_WATCHDOG
  wdt_disable();
#endif

  // Motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pararMotor();

  // Botões
  pinMode(BOTAO_DIREITA, INPUT_PULLUP);
  pinMode(BOTAO_ESQUERDA, INPUT_PULLUP);
  pinMode(BOTAO_STOP, INPUT_PULLUP);

  // Configuração salva
  carregarConfig();

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Controle Motor");
  lcd.setCursor(0, 1);
  lcd.print("v");
  lcd.print(VERSAO_FW);
  lcd.print(" Iniciando..");

  Serial.begin(9600);

#if USAR_DS18B20
  sensorDS.begin();
  sensorDS.setWaitForConversion(false);
  sensorDS.requestTemperatures();
#endif

  delay(1500);

#if USAR_DS18B20
  ultimaLeituraDS = 0;
#endif

  lcd.clear();

  Serial.println("================================");
  Serial.println("   SISTEMA DE CONTROLE MOTOR");
  Serial.println("================================");
  Serial.println("Sistema iniciado.");
  enviarVersao();
  enviarConfig();
  Serial.println();

#if USAR_WATCHDOG
  wdt_enable(WDTO_2S);
#endif
}


// =============================
// LOOP
// =============================
void loop() {

#if USAR_WATCHDOG
  wdt_reset();
#endif

  // Comandos vindos do painel
  lerComandosSerial();

  // ---------- Tempo (potenciômetro ou painel) ----------
  int valorPot = analogRead(POT_TEMPO);

  unsigned long decimos;
  if (cfg.tempoRemoto) {
    decimos = cfg.tempoRemotoDecimos;
  } else {
    decimos = map(valorPot, 0, 1023, 1, 300);     // 0,1 s a 30,0 s
  }
  unsigned long maxDecimos = cfg.tempoMaxS * 10UL;
  if (decimos > maxDecimos) decimos = maxDecimos;

  // ---------- Temperatura ----------
  float temperatura = lerTemperatura();

  // ---------- Alarme de temperatura ----------
  bool semLeitura = isnan(temperatura);
  bool foraFaixa = false;
  if (cfg.alarmeAtivo) {
    if (semLeitura) foraFaixa = true;
    else if (temperatura < cfg.tMinX10 / 10.0 || temperatura > cfg.tMaxX10 / 10.0) foraFaixa = true;
  }
  alarmeTemp = foraFaixa;
  bloqueioAlarme = foraFaixa && cfg.alarmeParar;

  if (alarmeTemp && !alarmeAnterior) {
    Serial.println("ALARME;temperatura fora da faixa");
  }
  alarmeAnterior = alarmeTemp;

  if (bloqueioAlarme) {
    if (motorLigado) {
      pararMotor();
      motorLigado = false;
      direcao = 0;
      cancelarSequencia();
      Serial.println("INTERROMPIDO;alarme de temperatura");
    }
    else if (seqAtiva) {
      cancelarSequencia();
      Serial.println("SEQUENCIA cancelada;alarme de temperatura");
    }
  }

  // ---------- Parada de emergência (botão S3 ou painel) ----------
  if (digitalRead(BOTAO_STOP) == LOW || emergenciaWeb) {

    pararMotor();
    motorLigado = false;
    direcao = 0;
    comandoWeb = 0;
    cancelarSequencia();

    if (!emergenciaAtiva) {
      emergenciaAtiva = true;

      Serial.println();
      Serial.println("!!! PARADA DE EMERGENCIA !!!");

      Serial.print("Ciclos concluidos: ");
      Serial.println(ciclos);

      Serial.print("Temperatura: ");
      Serial.print(temperatura, 1);
      Serial.println(" C");

      Serial.println("==============================");
    }

    lcd.setCursor(0, 0);
    lcd.print("!!! PARADA !!!  ");

    lcd.setCursor(0, 1);
    lcd.print("STOP T:");
    lcd.print(temperatura, 1);
    lcd.print((char)223);
    lcd.print("C   ");

    enviarDados(temperatura, decimos);
    delay(30);
    return;
  }

  emergenciaAtiva = false;


  // ---------- Botões S1 / S2 e comandos D / E do painel ----------
  int estadoDireita = digitalRead(BOTAO_DIREITA);
  int estadoEsquerda = digitalRead(BOTAO_ESQUERDA);

  bool pedidoDireita = (estadoAnteriorDireita == HIGH && estadoDireita == LOW)
                       || comandoWeb == 1;
  bool pedidoEsquerda = (estadoAnteriorEsquerda == HIGH && estadoEsquerda == LOW)
                        || comandoWeb == 2;

  if (pedidoDireita) {
    tentarIniciar(1, decimos, temperatura);
  }
  else if (pedidoEsquerda) {
    tentarIniciar(2, decimos, temperatura);
  }

  comandoWeb = 0;
  estadoAnteriorDireita = estadoDireita;
  estadoAnteriorEsquerda = estadoEsquerda;


  // ---------- Sequência de ciclos ----------
  if (seqAtiva && !motorLigado && (long)(millis() - seqProximoMs) >= 0) {
    iniciarCiclo(seqProximaDir, decimos, temperatura);
    if (seqRestantes > 0) seqRestantes--;
    if (seqModo == 3) seqProximaDir = (seqProximaDir == 1) ? 2 : 1;
  }


  // ---------- Controle do motor (rampa e fim do ciclo) ----------
  if (motorLigado) {

    unsigned long decorrido = millis() - inicioMotor;

    if (decorrido >= duracaoCicloMs) {

      pararMotor();
      motorLigado = false;

      int direcaoFinal = direcao;
      direcao = 0;
      ciclos++;

      Serial.println();

      Serial.print("Ciclo ");
      Serial.print(ciclos);
      Serial.println(" concluido.");

      Serial.print("Direcao: ");
      Serial.println(direcaoFinal == 1 ? "DIREITA" : "ESQUERDA");

      Serial.print("Tempo programado: ");
      imprimirTempo(decimosCiclo);
      Serial.println(" s");

      Serial.print("Temperatura final: ");
      Serial.print(temperatura, 1);
      Serial.println(" C");

      Serial.println("------------------------------");

      if (seqAtiva) {
        if (seqRestantes == 0) {
          seqAtiva = false;
          Serial.println("SEQUENCIA concluida;todos os ciclos");
        } else {
          seqProximoMs = millis() + seqIntervaloMs;
        }
      }
    }
    else {
      atualizarMotor(decorrido);
    }
  }


  // ---------- LCD - linha 1 ----------
  lcd.setCursor(0, 0);
  lcd.print("Tempo: ");
  lcd.print(decimos / 10);
  lcd.print(".");
  lcd.print(decimos % 10);
  lcd.print("s  ");

  // ---------- LCD - linha 2 ----------
  lcd.setCursor(0, 1);

  if (!motorLigado) lcd.print("P");
  else if (direcao == 1) lcd.print("D");
  else lcd.print("E");

  lcd.print(alarmeTemp ? "!" : " ");
  lcd.print("C");

  if (ciclos < 10) lcd.print("00");
  else if (ciclos < 100) lcd.print("0");

  lcd.print(ciclos);

  lcd.print(" T");
  lcd.print(temperatura, 1);
  lcd.print("C  ");


  // ---------- Envio de dados ao painel (a cada 500 ms) ----------
  enviarDados(temperatura, decimos);

  delay(30);
}
