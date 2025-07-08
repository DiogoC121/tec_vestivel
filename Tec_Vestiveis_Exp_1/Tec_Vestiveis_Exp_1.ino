// Inclusão das Bibliotecas
#include <Wire.h>  // Biblioteca para comunicação I2C
#include <math.h>  // Para funções matemáticas como atan2 e sqrt
// A biblioteca Arduino.h (incluída implicitamente) já define RAD_TO_DEG e M_PI
#define buzzer 9   // Definição do pino do buzzer
#define ledPin 13   // Definição do pino do led

// --- Protótipos das Funções ---
void configuracao();
void requisicao(); // Função para ler e processar os dados brutos do MPU6050
// Função Kalman: filtra uma medição (accelAngle) usando uma taxa de mudança (gyroRate)
float kalman(float accelAngle, float gyroRate, float dt, float Q_angle, float R_measure, float& x_est, float& P);
// Função para limitar um ângulo entre -360 e +360 graus
float fixAngle(float angle);

// Endereço I2C do sensor MPU-6050
const int MPU = 0x68;

// Variáveis para armazenar valores brutos do sensor (16 bits)
int16_t AccX_raw_val, AccY_raw_val, AccZ_raw_val;
int16_t GyrX_raw_val, GyrY_raw_val, GyrZ_raw_val;
int16_t Temp_raw_val; 

// Variáveis para armazenar valores processados (escalados e com média) do sensor
float AccX, AccY, AccZ;
float Temp;
float GyrX, GyrY, GyrZ;

// Variáveis para armazenar ângulos do Acelerômetro (sem filtro)
float AccAngleX, AccAngleY; // Ângulos derivados do acelerômetro (Pitch e Roll)

// Variáveis para armazenar ângulos filtrados pelo Kalman
float KalmanAngleX = 0.0; // Ângulo de Pitch filtrado
float KalmanAngleY = 0.0; // Ângulo de Roll filtrado
float KalmanAngleZ = 0.0; // Ângulo de Yaw filtrado (atenção: Yaw do MPU-6050 sozinho tem deriva significativa)

// Variáveis de estado para o filtro de Kalman para cada ângulo (Pitch, Roll, Yaw)
// Pitch (X-axis)
float Q_angle_X = 0.001;   // Variância do ruído do processo do ângulo X (ajustar para suavização vs. responsividade)
float R_measure_X = 0.03;  // Variância do ruído da medição do ângulo X (confiança na medição do acelerômetro)
float x_est_X = 0.0;       // Estimativa de estado inicial para Pitch
float P_X = 1.0;           // Covariância de erro inicial para Pitch

// Roll (Y-axis)
float Q_angle_Y = 0.001;   // Variância do ruído do processo do ângulo Y
float R_measure_Y = 0.03;  // Variância da medição do ângulo Y
float x_est_Y = 0.0;       // Estimativa de estado inicial para Roll
float P_Y = 1.0;           // Covariância de erro inicial para Roll

// Yaw (Z-axis) - Note: Yaw from MPU-6050 alone will drift significantly
float Q_angle_Z = 0.001;   // Variância do ruído do processo do ângulo Z
float R_measure_Z = 0.03;  // Variância da medição do ângulo Z (menos confiável apenas com giroscópio)
float x_est_Z = 0.0;       // Estimativa de estado inicial para Yaw
float P_Z = 1.0;           // Covariância de erro inicial para Yaw

// Variáveis para controle de tempo para o cálculo do delta time (dt)
unsigned long lastUpdateTime = 0; // Armazena o último tempo de atualização do loop

// Constantes de escala e limiares
const float POSTURE_THRESHOLD_DEG = 15.0; // Limiar de desvio postural em graus para acionar o alerta
// REMOVIDO: const float RAD_TO_DEG = 180.0 / M_PI; // Já definido em Arduino.h
const float ACCEL_SCALE_FACTOR = 16384.0; // Fator de escala para acelerômetro +/- 2g (2g = 16384 LSB/g)
const float GYRO_SCALE_FACTOR = 16.4;     // Fator de escala para giroscópio +/- 2000 dps (2000 dps = 16.4 LSB/dps)

// Variáveis para controle do alerta não-bloqueante
unsigned long lastAlertTriggerTime = 0; // Armazena o último momento em que um alerta completo foi iniciado
const long ALERT_COOLDOWN = 2000;       // Tempo mínimo (em ms) entre alertas completos (2 segundos)
bool isAlertActive = false;             // Flag para indicar se uma sequência de alerta está ativa
unsigned long toneStartTime = 0;        // Armazena o tempo de início do tom atual
int currentToneStep = 0;                // 0: inativo, 1: primeiro tom, 2: segundo tom
const long TONE_DURATION = 500;         // Duração de cada tom (em ms)

// Variáveis para controle da impressão serial não-bloqueante
unsigned long lastPrintTime = 0;        // Armazena o último momento em que os dados foram impressos
const long PRINT_INTERVAL = 1000;       // Intervalo de tempo (em ms) entre as impressões no Serial Monitor (1 segundo)

void setup() {
  // Configura o pino do buzzer como saída
  pinMode(buzzer, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
  digitalWrite(ledPin, HIGH);

  // Inicializa a comunicação serial com taxa de 9600 bps
  Serial.begin(9600);

  // Inicializa a comunicação I2C
  Wire.begin();

  // Configuração do MPU6050:
  // 1. Sair do modo sleep (registrador PWR_MGMT_1, 0x6B)
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Registrador PWR_MGMT_1
  Wire.write(0x00); // Escreve 0 para sair do modo sleep
  Wire.endTransmission(true);

  // 2. Configuração do fundo de escala do giroscópio para +/- 2000 °/s (0x18)
  // O registrador GYRO_CONFIG (0x1B) controla o fundo de escala do giroscópio.
  // 0x18 = +/- 2000 °/s (sensibilidade de 16.4 LSB/°/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // Registrador GYRO_CONFIG
  Wire.write(0x18); // Define o fundo de escala para +/- 2000 °/s
  Wire.endTransmission(true);

  // 3. Configuração do fundo de escala do acelerômetro para +/- 2g (0x00)
  // O registrador ACCEL_CONFIG (0x1C) controla o fundo de escala do acelerômetro.
  // 0x00 = +/- 2g (sensibilidade de 16384 LSB/g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C); // Registrador ACCEL_CONFIG
  Wire.write(0x00); // Define o fundo de escala para +/- 2g
  Wire.endTransmission(true);
  
  Serial.println("MPU6050 Configurado.");

  // Calibração: Inicializa os ângulos do filtro de Kalman com a primeira leitura do acelerômetro
  Serial.println("Calibrando MPU6050... Mantenha o sensor parado.\n");
  
  // Faz uma leitura inicial para preencher AccX, AccY, AccZ
  // (A função requisicao() já faz a média e a escalagem)
  requisicao(); 
  
  // Calcula os ângulos iniciais do acelerômetro para inicializar o Kalman
  AccAngleX = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG; // Usando RAD_TO_DEG do Arduino.h
  AccAngleY = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG; // Usando RAD_TO_DEG do Arduino.h
  
  // Inicializa as estimativas de estado do Kalman com os ângulos do acelerômetro
  x_est_X = AccAngleX;
  x_est_Y = AccAngleY;
  x_est_Z = 0; // Yaw é inicializado como 0, pois não há referência gravitacional para ele
  
  // Inicializa o lastUpdateTime para o cálculo correto do dt no loop
  lastUpdateTime = millis();

  Serial.println("Calibração do MPU6050 concluída.");
  delay(1000); // Pequeno atraso para exibir a mensagem

  digitalWrite(ledPin, LOW);
}

void loop() {
  unsigned long currentTime = millis(); // Obtém o tempo atual em milissegundos
  float dt = (currentTime - lastUpdateTime) / 1000.0; // Tempo decorrido em segundos
  lastUpdateTime = currentTime; // Atualiza o último tempo de atualização

  // Realiza a requisição dos dados brutos do sensor (com média de 10 leituras)
  requisicao();
  
  // --- Cálculo dos ângulos do Acelerômetro (Pitch e Roll) ---
  // AccAngleX (Pitch): rotação em torno do Y
  AccAngleX = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG; // Usando RAD_TO_DEG do Arduino.h
  // AccAngleY (Roll): rotação em torno do X
  AccAngleY = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG; // Usando RAD_TO_DEG do Arduino.h

  // Aplica o filtro de Kalman para cada ângulo
  // KalmanAngleX e KalmanAngleY são os ângulos de Pitch e Roll filtrados
  KalmanAngleX = kalman(AccAngleX, GyrX, dt, Q_angle_X, R_measure_X, x_est_X, P_X);
  KalmanAngleY = kalman(AccAngleY, GyrY, dt, Q_angle_Y, R_measure_Y, x_est_Y, P_Y);
  // Para Yaw, como não há medição de acelerômetro, passamos 0 para accelAngle
  KalmanAngleZ = kalman(0, GyrZ, dt, Q_angle_Z, R_measure_Z, x_est_Z, P_Z);
  
  // --- Limita os ângulos entre -360 e +360 graus ---
  KalmanAngleX = fixAngle(KalmanAngleX);
  KalmanAngleY = fixAngle(KalmanAngleY);
  KalmanAngleZ = fixAngle(KalmanAngleZ); // Aplicar também para Yaw se desejar

  // Lógica de Detecção de Postura Inadequada e Alerta (não-bloqueante)
  // O buzzer é acionado se o valor absoluto do ângulo de Pitch (KalmanAngleX) ou
  // do ângulo de Roll (KalmanAngleY) exceder o limiar definido.
  // Variáveis para controle do alerta não-bloqueante
  unsigned long lastAlertTriggerTime = 0; // Armazena o último momento em que um alerta completo foi iniciado
  const long ALERT_COOLDOWN = 2000;       // Tempo mínimo (em ms) entre alertas completos (2 segundos)
  bool isAlertActive = false;             // Flag para indicar se uma sequência de alerta está ativa
  unsigned long toneStartTime = 0;        // Armazena o tempo de início do tom atual
  int currentToneStep = 0;                // 0: inativo, 1: primeiro tom, 2: segundo tom
  const long TONE_DURATION = 500;         // Duração de cada tom (em ms)

  if (abs(KalmanAngleX) > POSTURE_THRESHOLD_DEG || abs(KalmanAngleY) > POSTURE_THRESHOLD_DEG) {
    // Se não houver um alerta ativo E o tempo de cooldown desde o último alerta completo terminou
    if (!isAlertActive && (currentTime - lastAlertTriggerTime >= ALERT_COOLDOWN)) {
      isAlertActive = true;       // Ativa o estado de alerta
      lastAlertTriggerTime = currentTime; // Registra o início deste novo ciclo de alerta
      currentToneStep = 1;        // Inicia com o primeiro tom
      toneStartTime = currentTime; // Registra o início do primeiro tom
      tone(buzzer, 294);          // Toca a nota Ré (294 Hz) - não-bloqueante
      Serial.print("ALERTA: Postura Inadequada! Pitch: ");
      Serial.print(KalmanAngleX);
      Serial.print(" | Roll: ");
      Serial.println(KalmanAngleY);
    }
  } else {
    // Postura está boa: desliga o buzzer e reseta o estado de alerta
    if (isAlertActive) { // Só desliga se estava ativo para evitar chamadas desnecessárias
      noTone(buzzer);
      isAlertActive = false;
      currentToneStep = 0;
    }
  }

  // Gerenciamento da sequência de tons do buzzer (não-bloqueante)
  if (isAlertActive) {
    if (currentToneStep == 1 && (currentTime - toneStartTime >= TONE_DURATION)) {
      // Tempo para o primeiro tom terminou, passa para o segundo
      noTone(buzzer);            // Para o primeiro tom
      tone(buzzer, 262);         // Inicia o segundo tom (Dó)
      toneStartTime = currentTime; // Reinicia o tempo para o segundo tom
      currentToneStep = 2;       // Avança para o segundo passo
    } else if (currentToneStep == 2 && (currentTime - toneStartTime >= TONE_DURATION)) {
      // Tempo para o segundo tom terminou, finaliza a sequência de tons
      noTone(buzzer);            // Para o segundo tom
      isAlertActive = false;     // Finaliza o ciclo de alerta
      currentToneStep = 0;       // Reseta o passo do tom
    }
  }

  // Lógica para impressão serial não-bloqueante
  // Imprime os dados do sensor apenas a cada PRINT_INTERVAL (1 segundo, por exemplo)
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = currentTime; // Atualiza o último tempo de impressão

    // Saída para monitoramento serial
    Serial.println("\n--- Leituras do Sensor ---");
    Serial.print("Aceleracao (g):\nX: "); Serial.print(AccX);
    Serial.print(" Y: "); Serial.print(AccY);
    Serial.print(" Z: "); Serial.println(AccZ);
    
    Serial.print("Giro (dps):\nX: "); Serial.print(GyrX);
    Serial.print(" Y: "); Serial.print(GyrY);
    Serial.print(" Z: "); Serial.println(GyrZ);
    
    Serial.print("Temperatura: "); Serial.print(Temp); Serial.println("ºC");

    Serial.print("Pitch (Angulo X): "); Serial.print(KalmanAngleX);
    Serial.print(" | Roll (Angulo Y): "); Serial.print(KalmanAngleY);
    //Serial.print(" | Yaw (Angulo Z): "); Serial.print(KalmanAngleZ);
    Serial.println("\n");
  }
}

// --- Função para Requisitar e Processar Dados Brutos do MPU6050 (com média de 10 leituras) ---
void requisicao(){
  // Variáveis para acumular as leituras brutas para a média
  long sumAccX_raw = 0, sumAccY_raw = 0, sumAccZ_raw = 0;
  long sumGyrX_raw = 0, sumGyrY_raw = 0, sumGyrZ_raw = 0;
  long sumTemp_raw = 0;

  const int numReadings = 10; // Número de leituras para a média

  for (int j = 0; j < numReadings; j++) {
    // Comandos para iniciar transmissão de dados
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);     // Posição de memória inicial (ACCEL_XOUT_H)
    Wire.endTransmission(false); // Não para a transmissão, permite leitura contínua
    Wire.requestFrom(MPU, 14, true); // Requisita 14 bytes (AccX,Y,Z, Temp, GyrX,Y,Z)

    // Armazena os valores brutos nas variáveis temporárias e acumula para a soma
    sumAccX_raw += (Wire.read() << 8 | Wire.read());
    sumAccY_raw += (Wire.read() << 8 | Wire.read());
    sumAccZ_raw += (Wire.read() << 8 | Wire.read());
    sumTemp_raw += (Wire.read() << 8 | Wire.read());
    sumGyrX_raw += (Wire.read() << 8 | Wire.read());
    sumGyrY_raw += (Wire.read() << 8 | Wire.read());
    sumGyrZ_raw += (Wire.read() << 8 | Wire.read());
    
    delay(1); // Pequeno atraso entre as leituras para garantir que o sensor tenha tempo
  }

  // Calcula a média dos valores brutos
  AccX_raw_val = sumAccX_raw / numReadings;
  AccY_raw_val = sumAccY_raw / numReadings;
  AccZ_raw_val = sumAccZ_raw / numReadings;
  Temp_raw_val = sumTemp_raw / numReadings;
  GyrX_raw_val = sumGyrX_raw / numReadings;
  GyrY_raw_val = sumGyrY_raw / numReadings;
  GyrZ_raw_val = sumGyrZ_raw / numReadings;

  // Transformação dos valores brutos médios em unidades físicas (g, dps, °C)
  AccX = AccX_raw_val / ACCEL_SCALE_FACTOR;
  AccY = AccY_raw_val / ACCEL_SCALE_FACTOR;
  AccZ = AccZ_raw_val / ACCEL_SCALE_FACTOR;
  Temp = Temp_raw_val / 340.0 + 36.53; // Conversão da temperatura
  
  // Correção de bias do giroscópio e conversão para dps
  GyrX = (GyrX_raw_val / GYRO_SCALE_FACTOR); // Bias será tratado pelo Kalman ou na inicialização
  GyrY = (GyrY_raw_val / GYRO_SCALE_FACTOR);
  GyrZ = (GyrZ_raw_val / GYRO_SCALE_FACTOR);
}

// --- Função Kalman ---
// Esta função implementa um filtro de Kalman 1D para estimar um ângulo.
// Ele usa a medição do acelerômetro para correção e a taxa de giro do giroscópio para predição.
float kalman(float accelAngle, float gyroRate, float dt, float Q_angle, float R_measure, float& x_est, float& P){
  // Passo de Predição
  // O ângulo é previsto com base na estimativa anterior e na taxa de giro do giroscópio
  float x_pred = x_est + gyroRate * dt;
  // A covariância do erro de predição aumenta com o ruído do processo
  float P_pred = P + Q_angle * dt;

  // Passo de Atualização (Correção)
  // O Ganho de Kalman pondera entre a predição e a nova medição do acelerômetro
  float K = P_pred / (P_pred + R_measure); 
  // A estimativa é corrigida usando a diferença entre a medição do acelerômetro e a predição
  x_est = x_pred + K * (accelAngle - x_pred); 
  // A covariância do erro de estimativa é atualizada
  P = (1 - K) * P_pred; 

  return x_est; // Retorna o valor filtrado (estimativa de ângulo)
}

// --- Função para Limitar Ângulos ---
// Esta função limita um ângulo para o intervalo de -360 a +360 graus.
// Se o ângulo exceder 360 ou -360, ele será 'clampado' para esse valor.
float fixAngle(float angle) {
  // A lógica de clamping deve garantir que o ângulo fique DENTRO da faixa.
  // Se o ângulo for exatamente 360 ou -360, ele deve ser mantido.
  // Se ele passar disso, deve ser ajustado.
  // Uma forma comum de manter dentro de uma faixa é usar fmod ou operações de módulo,
  // mas para clamping simples, if/else é suficiente.
  if (angle > 360.0) {
    return 360.0; // Limita ao máximo
  }
  if (angle < -360.0) {
    return -360.0; // Limita ao mínimo
  }
  return angle;
}
