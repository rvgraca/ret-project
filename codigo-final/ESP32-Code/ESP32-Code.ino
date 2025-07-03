#include <WiFi.h>
#include <HTTPClient.h>
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


// const char* ssid = "polymath-phone";
// const char* password = "pato-eduardo-walsh-2006";
const char* ssid = "MovistarFibra-2023";
const char* password = "30882709";
// String serverIP = "http://192.168.9.205:3000";  // IP del servidor local con puerto
String serverIP = "http://192.168.1.43:3000";  // IP del servidor local con puerto

const int visualizations = 1;

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



void setup() 
{
  initialize_pins();
  if (visualizations) Serial.begin(115200);
  initialize_wifi();
}

void loop() 
{
  http_request();
  update_motors();
  delay(100); 
}

void initialize_pins()
{
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
}

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