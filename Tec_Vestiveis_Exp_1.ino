// Inclusão das Bibliotecas
#include<Wire.h> 
#include<math.h>
#define buzzer 9
#define ledPin 13

// --- Funções ---
void configuracao();
void requisicao();
float kalman(float input, float& x_est, float& P);
float stopAngle(float angle);

// Endereco I2C do sensor MPU-6050
const int MPU = 0x68;
int i, initC =0;
unsigned long time;

// Variáveis para armazenar valores brutos do sensor (16 bits)
int16_t AccX_raw_val, AccY_raw_val, AccZ_raw_val;
int16_t GyrX_raw_val, GyrY_raw_val, GyrZ_raw_val;
int16_t Temp_raw_val; 

// Variaveis para armazenar valores do sensor - esses valores são palavras binárias de 16bits
float AccX, AccY, AccZ, Temp, GyrX, GyrY, GyrZ;

// Variaveis para armazenar ângulos crus (sem filtro)
float AccAngleX, AccAngleY; // Angulos do acelerômetro
float GyroAngleX, GyroAngleY, GyroAngleZ; // Angulos do giroscópio (integrados)

// Variaveis para armazenar ângulos filtrados pelo Kalman
float KalmanAngleX, KalmanAngleY, KalmanAngleZ = 0.0;
float KalmanAngleXinit, KalmanAngleYinit, KalmanAngleZinit = 0.0;

// Variaveis para o filtro de Kalman para cada ângulo
// Pitch (X-axis)
float Q_angle_X = 0.005; // Variância do processo do ângulo X (ajuste conforme a necessidade de suavização vs. responsividade)
float R_measure_X = 0.07; // Variância da medição do ângulo X (ajuste conforme a confiança na medição do acelerômetro)
float x_est_X = 0.0;
float P_X = 1.0;

// Roll (Y-axis)
float Q_angle_Y = 0.005; // Variância do processo do ângulo Y
float R_measure_Y = 0.07; // Variância da medição do ângulo Y
float x_est_Y = 0.0;
float P_Y = 1.0;

// Yaw (Z-axis) - Note: Yaw from MPU-6050 alone will drift significantly
float Q_angle_Z = 0.005; // Variância do processo do ângulo Z
float R_measure_Z = 0.07; // Variância da medição do ângulo Z (less reliable from gyro alone)
float x_est_Z = 0.0;
float P_Z = 1.0;

unsigned long lastUpdateTime = 0; // Para calcular o delta time para integração do giroscópio

const float POSTURE_THRESHOLD_DEG = 15.0; // Limiar de desvio postural em graus para acionar o alerta
const float ACCEL_SCALE_FACTOR = 16384.0; // Fator de escala para acelerômetro +/- 2g (2g = 16384 LSB/g)
const float GYRO_SCALE_FACTOR = 16.4;     // Fator de escala para giroscópio +/- 2000 dps (2000 dps = 16.4 LSB/dps)

unsigned long lastPrintTime = 0;        // Armazena o último momento em que os dados foram impressos
const long PRINT_INTERVAL = 1000;       // Intervalo de tempo (em ms) entre as impressões no Serial Monitor (1 segundo)

void setup() {
  configuracao();  
  lastUpdateTime = millis(); // Inicializa o lastUpdateTime
}

void loop() {
  unsigned long currentTime = millis(); // Obtém o tempo atual em milissegundos
  float dt = (currentTime - lastUpdateTime) / 1000.0; // Tempo decorrido em segundos
  lastUpdateTime = currentTime;

  requisicao();
  
  Serial.println("\n");
  Serial.println("\n");
  Serial.println("\n");
  Serial.println("\n");
  Serial.println("\n");

  // --- Cálculo dos ângulos do Acelerômetro (Pitch e Roll) ---
  // AccAngleX (Pitch): rotação em torno do Y
  AccAngleX = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
  // AccAngleY (Roll): rotação em torno do X
  AccAngleY = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;

  // Os ângulos do giroscópio são obtidos integrando a taxa de giro ao longo do tempo (dt)
  //GyroAngleX += GyrX * dt;
  //GyroAngleY += GyrY * dt;
  //GyroAngleZ += GyrZ * dt; // drift

  KalmanAngleX = kalman(AccAngleX, GyroAngleX, dt, Q_angle_X, R_measure_X, x_est_X, P_X);
  KalmanAngleY = kalman(AccAngleY, GyroAngleY, dt, Q_angle_Y, R_measure_Y, x_est_Y, P_Y);
  KalmanAngleZ = kalman(0, GyroAngleZ, dt, Q_angle_Z, R_measure_Z, x_est_Z, P_Z);
  if(initC==0){
    initC = 1;
    KalmanAngleXinit = KalmanAngleX;
    KalmanAngleYinit = KalmanAngleY;
    KalmanAngleZinit = KalmanAngleZ;
  }
 
  KalmanAngleX = stopAngle(KalmanAngleX);
  KalmanAngleY = stopAngle(KalmanAngleY);
  KalmanAngleZ = stopAngle(KalmanAngleZ);

  if ((((abs(KalmanAngleX)-abs(KalmanAngleXinit)) > POSTURE_THRESHOLD_DEG) || ((abs(KalmanAngleX)-abs(KalmanAngleXinit)) < -(POSTURE_THRESHOLD_DEG)))
   || 
  (((abs(KalmanAngleY)-abs(KalmanAngleYinit)) >  POSTURE_THRESHOLD_DEG)|| ((abs(KalmanAngleY)-abs(KalmanAngleYinit)) < -(POSTURE_THRESHOLD_DEG))))
  {
    digitalWrite(ledPin, HIGH);
    tone(buzzer,294,500); // RE
    tone(buzzer,292,500); // DO
    Serial.println("ALERTA: Postura Inadequada!");
    //delay(1000);

  } else {
    noTone(buzzer);
    digitalWrite(ledPin, LOW);
  }


// Saída para monitoramento serial
  //if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
  //  lastPrintTime = currentTime; // Atualiza o último tempo de impressão
    Serial.println("--- Leitura dos Sensores ---");
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
    Serial.print("\n");
    Serial.print("Angulo X inicial: "); Serial.print(KalmanAngleXinit);
    Serial.print(" | Angulo Y inicial: "); Serial.print(KalmanAngleYinit);
    Serial.print("\n");
    Serial.print("ROT X: "); Serial.print(abs(KalmanAngleX)-abs(KalmanAngleXinit));
    Serial.print(" | ROT Y: "); Serial.print(abs(KalmanAngleY)-abs(KalmanAngleYinit));
    Serial.print("\n");
  //}
  delay(500);
}

void configuracao(){
  pinMode(buzzer, OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(ledPin, HIGH);

  // Inicialização da porta Serial e sua taxa de trasmissão de dados
  Serial.begin(9600);

  // Inicialização a comunicação i2c com o MPU-6050
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Registrador PWR_MGMT_1 
  Wire.write(0);    // Sair do modo sleep
  Wire.endTransmission(true);

  // seleção do fundo do escala do giroscópio.
  /*
    Wire.write(0b00000000); // fundo de escala em +/-250°/s
    Wire.write(0b00001000); // fundo de escala em +/-500°/s
    Wire.write(0b00010000); // fundo de escala em +/-1000°/s
    Wire.write(0b00011000); // fundo de escala em +/-2000°/s - ESSE
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);         // Registrador GYRO_CONFIG - posição de momeória responsável pelo fundo de escala do giroscópio
  Wire.write(0x00011000);   // escrita no endereço de memória
  Wire.endTransmission();
  
  //   seleção do fundo do escala do acelerômetro.
  /*
      Wire.write(0b00000000); // fundo de escala em +/-2g - ESSE
      Wire.write(0b00001000); // fundo de escala em +/-4g
      Wire.write(0b00010000); // fundo de escala em +/-8g
      Wire.write(0b00011000); // fundo de escala em +/-16g
  */
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);           // Registrador ACCEL_CONFIG - posição de memória responsával pelo ajuste de fundo de escala do acelerômetro
  Wire.write(0b00000000);     // Trocar esse comando para fundo de escala desejado conforme acima
  Wire.endTransmission();
  
  Serial.println("MPU6050 Configurado.");

  // Calibração
  Serial.println("Calibrando MPU6050... Mantenha o sensor parado.\n");
  // Inicializa os ângulos do filtro de Kalman com a primeira leitura do acelerômetro
  requisicao(); // Faz uma leitura inicial para preencher AccX, AccY, AccZ
  AccAngleX = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * RAD_TO_DEG;
  AccAngleY = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * RAD_TO_DEG;
  x_est_X = AccAngleX;
  x_est_Y = AccAngleY;
  x_est_Z = 0;
  delay(1000); 
  Serial.println("Calibração do MPU6050 concluída.");
  delay(1000); 

  digitalWrite(ledPin, LOW);
}

void requisicao(){
  long sumAccX_raw = 0, sumAccY_raw = 0, sumAccZ_raw = 0;
  long sumGyrX_raw = 0, sumGyrY_raw = 0, sumGyrZ_raw = 0;
  long sumTemp_raw = 0;
  for (int i = 0; i < 10; i++) {
    // Comandos para iniciar transmissão de dados
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);     //posição de memória inicial
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14, true); // requisição de 14 palavras de 16 bits. Começando do endereço de memoria indicado acima (0x3B)
    // Armazena o valor das palavras nas variaveis correspondentes
    AccX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     // lê os primeiros 8bits da parte alta, desloca e lê os outros 8 bits da parte baixa
    AccY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)     // faz o mesmo para os outros sensores.
    AccZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Temp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyrX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyrY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyrZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    sumAccX_raw += AccX; 
    sumAccY_raw += AccY, 
    sumAccZ_raw += AccZ;
    sumGyrX_raw += GyrX, 
    sumGyrY_raw += GyrY, 
    sumGyrZ_raw += GyrZ;
    sumTemp_raw += Temp;
    delay(1); // Pequeno atraso entre as leituras para garantir que o sensor tenha tempo

  }
  
  // Calcula a média dos valores brutos
  AccX_raw_val = sumAccX_raw / 10;
  AccY_raw_val = sumAccY_raw / 10;
  AccZ_raw_val = sumAccZ_raw / 10;
  Temp_raw_val = sumTemp_raw / 10;
  GyrX_raw_val = sumGyrX_raw / 10;
  GyrY_raw_val = sumGyrY_raw / 10;
  GyrZ_raw_val = sumGyrZ_raw / 10;

  // Transformação dos valores brutos médios em unidades físicas (g, dps, °C)
  AccX = AccX_raw_val / ACCEL_SCALE_FACTOR;
  AccY = AccY_raw_val / ACCEL_SCALE_FACTOR;
  AccZ = AccZ_raw_val / ACCEL_SCALE_FACTOR;
  GyrX = (GyrX_raw_val / GYRO_SCALE_FACTOR);                                  
  GyrY = (GyrY_raw_val / GYRO_SCALE_FACTOR);
  GyrZ = (GyrZ_raw_val / GYRO_SCALE_FACTOR);
  Temp = Temp_raw_val / 340.0 + 36.53;
}

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

float stopAngle(float angle) {
  if (angle >= 360.0) {
    return 0;
  }
  if (angle <= -360.0) {
    return 0;
  }
  return angle;
}