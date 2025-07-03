#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>

//Driver
#define END 16
#define ENC 17
#define ENB 18
#define ENA 19

#define IN1 32
#define IN2 33
#define IN3 25
#define IN4 26
#define IN5 27
#define IN6 14
#define IN7 12
#define IN8 13
struct MotorConfig {
  int speed;
  int in1, in2;
  int en;
};

MotorConfig motorConfigs[4] = {
  {0, IN1, IN2, ENA},  // motor A
  {0, IN3, IN4, ENB},  // motor B
  {0, IN5, IN6, ENC},  // motor C
  {0, IN7, IN8, END}   // motor D
};

//BMP280
#define BMP280_ADDR 0x76
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t t_fine;

//MQ7
#define MQ7_PIN 35 

//MPU6050
#define MPU6050_ADDR 0x68
float gyro_X_offset = 0, gyro_Y_offset = 0, gyro_Z_offset = 0, acc_X_offset = 0, acc_Y_offset = 0, acc_Z_offset = 0;
int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz, raw_temperature;
float ax, ay, az, gx, gy, gz, temperature;
float pitch = 0, roll = 0, yaw = 0;
unsigned long prevMicros;

//Ultrasonic Sensor
#define TRIG_PIN 23
#define ECHO_PIN 4 
#define TIMEOUT_US 20000
#define SOUND_SPEED 0.0343

//Servo
#define SERVO_PIN 16
#define SERVO_FREQUENCY 50 //Hz
#define SERVO_RESOLUTION 16 //bits

//QMC5883L
#define QMC5883L_ADDR 0x0D
#define QMC_REG_CONTROL 0x09
#define QMC_REG_SET_RESET 0x0B
#define QMC_REG_DATA 0x00
struct MagnetometerData 
{
  int16_t x;
  int16_t y;
  int16_t z;
};

// const char* ssid = "polymath-phone";
// const char* password = "pato-eduardo-walsh-2006";
const char* ssid = "MovistarFibra-2023";
const char* password = "30882709";
// String serverIP = "http://192.168.9.205:3000";  // IP del servidor local con puerto
String serverIP = "http://192.168.1.43:3000";  // IP del servidor local con puerto

const int visualizations = 1;





void setup() 
{
  Wire.begin();
  initialize_pins();
  if (visualizations) Serial.begin(115200);
  initialize_wifi();

  init_bmp280();
  read_calibration_data_bmp280();

  initialize_MPU6050();
  calibrate_MPU6050();
  prevMicros = micros();

  initialize_servo()

}

void loop() 
{
  http_request();
  update_motors();

  // -------------------BMP280----------------------
  float temp = readTemperature();
  float pres_hPa = readPressure() / 100.0;  
  // -------------------MQ7----------------------
  int raw_adc = analogRead(MQ7_PIN);
  float voltage = (raw_adc / 4095.0) * 3.3;  // Convert to voltage

  //MPU6050
  unsigned long now = micros();
  if (now - prevMicros >= 20000) 
  {  
    float dt = (now - prevMicros) / 1000000.0;
    prevMicros = now;
    read_MPU6050();
    process_MPU6050(dt);
    show_values();
  }

  // Ultrasonic sensor
  float distance = measure_distance()

  // Servo
  int angle = 0; //(degrees)
  write_servo_angle(angle);


  // QMC5883L
  MagnetometerData mag = readQMC5883L();
  printData(mag);
  float heading = calculateHeading(mag);
  float aux = heading; 
  heading = 0.00000629833 * heading *heading * heading * heading - 0.00165387 *heading*heading*heading + 0.147703 * heading * heading - 7.89996 * heading + 361.77908;
  if (heading  >= 245)
  {
    heading = 0.00652658*aux*aux + 0.118741 * aux + 266.45308;
  }


  //
  delay(100); 
}

// ---------------- OTHER ----------------------------
void initialize_pins()
{
  //Driver
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(ENC, LOW);
  digitalWrite(END, LOW);
  
  //Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void write_register(uint8_t add, uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(add);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readRegisters(uint8_t add, uint8_t reg, uint8_t* buffer, uint8_t length) 
{
  Wire.beginTransmission(add);
  Wire.write(reg);
  Wire.endTransmission(false);  // reinicio de transmisión (repeated start)
  Wire.requestFrom(add, length);

  for (int i = 0; i < length && Wire.available(); i++) 
  {
    buffer[i] = Wire.read();
  }
}

uint8_t read8(uint8_t reg) 
{
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDR, 1);
  return Wire.read();
}

uint16_t read16(uint8_t reg) 
{
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDR, 2);
  uint16_t value = (Wire.read() | (Wire.read() << 8));
  return value;
}

int16_t readS16(uint8_t reg) 
{
  return (int16_t)read16(reg);
}

uint32_t read24(uint8_t reg) 
{
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDR, 3);
  uint32_t value = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
  return value;
}

// ---------------- WI-FI-----------------------------
void initialize_wifi()
{
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (visualizations) Serial.print(".");
  }
  if (visualizations) Serial.println("Conectado a WiFi.");
}

void http_request()
{
  if(WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    String url = serverIP + "/getMotors";
    http.begin(url);
    int code = http.GET();
    if (code == 200) 
    {
      String response = http.getString();
      if (visualizations) Serial.println("Respuesta del servidor: " + response);
      int i1 = response.indexOf("motorA") + 8;
      int i2 = response.indexOf(",", i1);
      motorConfigs[0].speed = response.substring(i1, i2).toInt();
      i1 = response.indexOf("motorB") + 8;
      i2 = response.indexOf(",", i1);
      motorConfigs[1].speed = response.substring(i1, i2).toInt();
      i1 = response.indexOf("motorC") + 8;
      i2 = response.indexOf(",", i1);
      motorConfigs[2].speed = response.substring(i1, i2).toInt();
      i1 = response.indexOf("motorD") + 8;
      i2 = response.indexOf("}", i1);
      motorConfigs[3].speed = response.substring(i1, i2).toInt();
      if (visualizations)
      {
        Serial.print("motorA: "); Serial.println(motorConfigs[0].speed);
        Serial.print("motorB: "); Serial.println(motorConfigs[1].speed);
        Serial.print("motorC: "); Serial.println(motorConfigs[2].speed);
        Serial.print("motorD: "); Serial.println(motorConfigs[3].speed);
      }
    }
    http.end();
  }
}
// ----------------------MOTORS-----------------------


void set_motor_direction_and_speed(MotorConfig& motor, int value) 
{
  int pwm = abs(value);
  bool reverse = value < 0;

  digitalWrite(motor.in1, reverse ? LOW : HIGH);
  digitalWrite(motor.in2, reverse ? HIGH : LOW);
  analogWrite(motor.en, constrain(pwm, 0, 255));
}

void update_motors() 
{
  for (int i = 0; i < 4; ++i) 
  {
    set_motor_direction_and_speed(motorConfigs[i], motorConfigs[i].speed);
  }
}

//-------------- BMP280-------------------------------
void read_calibration_data_bmp280() 
{
  dig_T1 = read16(0x88);
  dig_T2 = readS16(0x8A);
  dig_T3 = readS16(0x8C);
  dig_P1 = read16(0x8E);
  dig_P2 = readS16(0x90);
  dig_P3 = readS16(0x92);
  dig_P4 = readS16(0x94);
  dig_P5 = readS16(0x96);
  dig_P6 = readS16(0x98);
  dig_P7 = readS16(0x9A);
  dig_P8 = readS16(0x9C);
  dig_P9 = readS16(0x9E);
}

void init_bmp280() 
{
  write_register(BMP280_ADDR, 0xF4, 0x27); // ctrl_meas: Temp x1, Press x1, Normal mode
  write_register(BMP280_ADDR, 0xF5, 0xA0); // config: standby 1000ms, filter off
}

float read_temperature_bmp280() 
{
  int32_t adc_T = read24(0xFA) >> 4;

  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                   ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                 ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T / 100.0;
}

float read_ressure_bmp280() 
{
  int32_t adc_P = read24(0xF7) >> 4;

  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
         ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 == 0) return 0; // avoid division by zero

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 256.0;
}

//------------------MPU6050------------------------------

void initialize_mpu6050() 
{
  write_register(MPU6050_ADDR, 0x6B, 0x00);
  write_register(MPU6050_ADDR, 0x1B, 0x08);
  write_register(MPU6050_ADDR, 0x1C, 0x10);
  write_register(MPU6050_ADDR, 0x1A, 0x04);
}

void calibrate_mpu6050()
{
  // Calibrating gyroscope using 3 thousand samples.
  const int samples = 3000;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  for (int i = 0; i < samples ; ++i)
  {
    read_mpu6050();
    sum_gx += raw_gx;
    sum_gy += raw_gy;
    sum_gz += raw_gz;
    sum_ax += raw_ax;
    sum_ay += raw_ay;
    sum_az += raw_az;
    delayMicroseconds(500);
  }
  // Calculate the mean of the 3 thousand samples.
  gyro_X_offset = sum_gx / samples;
  gyro_Y_offset = sum_gy / samples;
  gyro_Z_offset = sum_gz / samples;
  acc_X_offset  = sum_ax  / samples;
  acc_Y_offset  = sum_ay  / samples;
  acc_Z_offset  = sum_az  / samples - 4096;
}

void read_mpu6050() 
{
  // The gyroscope/accelerometer data is found from the addresses 3B to 14.
  Wire.beginTransmission(MPU6050_ADDR);       // Start communication
  Wire.write(0x3B);                             // Ask for the register 0x3B (AcX)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);         // Ask for 14 registers.

  raw_ax = Wire.read() << 8 | Wire.read();          // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  raw_ay = Wire.read() << 8 | Wire.read();          // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  raw_az = Wire.read() << 8 | Wire.read();          // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  raw_temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  raw_gx = Wire.read() << 8 | Wire.read();          // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  raw_gy = Wire.read() << 8 | Wire.read();          // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  raw_gz = Wire.read() << 8 | Wire.read();          // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
}

void process_mpu6050(float dt) 
{
  ax = (raw_ax - acc_X_offset) / 4096.0; 
  ay = (raw_ay - acc_Y_offset) / 4096.0; 
  az = (raw_az - acc_Z_offset) / 4096.0; 

  gx = (raw_gx - gyro_X_offset) / 65.5; // ±500 °/s → 65.5 LSB/(°/s)
  gy = (raw_gy - gyro_Y_offset) / 65.5;
  gz = (raw_gz - gyro_Z_offset) / 65.5;
  temperature = (raw_temperature / 340.0) + 36.53;

  // Integrating the angular speed gives us the delta angle.
  pitch += gx * dt;
  roll += gy * dt;
  yaw += gz * dt;

  float acc_total = sqrt(ax*ax + ay*ay + az*az);
  float pitch_acc = 0, roll_acc = 0, yaw_acc = 0;
  if (fabs(acc_total - 1.0) < 0.3) // ±0.3 g tolerance
  { 
    pitch_acc = atan2(ay, az) * 57.2958; // ° = radians * 180/π
    roll_acc  = atan2(-ax, sqrt(ay*ay + az*az)) * 57.2958;
  }

  // Complementary filter (very high Kp in gyroscope , Low Ki in accelerometer).
  const float alpha = 0.98;
  if (pitch_acc != 0 && roll_acc != 0) 
  {
    pitch = alpha*(pitch) + (1 - alpha)*pitch_acc;
    roll  = alpha*(roll)  + (1 - alpha)*roll_acc;
  }
}

void show_values_mpu6050()
{
  Serial.print("Pitch: "); Serial.print(pitch, 1);
  Serial.print("°, Roll: "); Serial.print(roll, 1);
  Serial.print("°, Yaw: "); Serial.print(yaw, 1);
  Serial.print("°, Temperature: "); Serial.println(temperature, 1);
}

//------------------Ultrasonic Sensor------------------------------
float measure_distance()
{
  long duration;
  float distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);
  distance = duration * SOUND_SPEED  / 2; // 343m/s = 0.0343 cm/us
  return distance;
}

void show_values_ultrasonic_sensor()
{
  Serial.println();
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

//------------------Servo-----------------------------------------
void initialize_servo()
{
  ledcAttach(SERVO_PIN, SERVO_FREQUENCY, SERVO_RESOLUTION); //Channel 0, 50 Hz, 16-bit resolution
}

void write_servo_angle(int angle) 
{
  angle = constrain(angle, -90, 90);
  int duty = map(angle, -90, 90, 1638, 8192);
  ledcWrite(SERVO_PIN, duty);
}


//------------------QMC5883L-------------------------------------
void initQMC5883L() 
{
  Wire.begin();
  write_register(QMC5883L_ADDR, QMC_REG_SET_RESET, 0x01);
  write_register(QMC5883L_ADDR, QMC_REG_CONTROL, 0b00011101);
}


MagnetometerData readQMC5883L() 
{
  uint8_t raw[6];
  readRegisters(QMC5883L_ADDR, QMC_REG_DATA, raw, 6);

  MagnetometerData data;
  data.x = (int16_t)((raw[1] << 8) | raw[0]);
  data.y = (int16_t)((raw[3] << 8) | raw[2]);
  data.z = (int16_t)((raw[5] << 8) | raw[4]);
  return data;
}

float calculateHeading(const MagnetometerData& data)
{
  float heading = atan2((float)data.y, -(float)data.x);
  return (180 * heading) / PI;
}

void print_cardinal(float heading) 
{
  if (heading < 22.5 || heading >= 337.5) Serial.println("Direction: N");
  else if (heading < 67.5) Serial.println("Direction: NE");
  else if (heading < 112.5) Serial.println("Direction: E");
  else if (heading < 157.5) Serial.println("Direction: SE");
  else if (heading < 202.5) Serial.println("Direction: S");
  else if (heading < 247.5) Serial.println("Direction: SW");
  else if (heading < 292.5) Serial.println("Direction: W");
  else Serial.println("Direction: NW");
}

void print_data_qmc5883l(const MagnetometerData& data) 
{
  Serial.print("Mag X: "); Serial.print(data.x);
  Serial.print("\tY: "); Serial.print(data.y);
  Serial.print("\tZ: "); Serial.println(data.z);
}