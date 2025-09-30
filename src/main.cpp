#include <WiFi.h>
#include <ESP32Servo.h>

// WiFi
const char* ssid = "Robot";
const char* password = "12345678";

// Pinler
#define MOTOR_A 27
#define MOTOR_B 14
#define SERVO_PIN 13

// PWM ayarları
#define PWM_CHANNEL 0
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8  // 0-255

Servo servo;
WiFiServer server(80);

int motorHiz = 0;
int servoAci = 90;

void setup() {
  Serial.begin(115200);

  // Motor PWM
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_A, PWM_CHANNEL);
  pinMode(MOTOR_B, OUTPUT);

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(90);

  // WiFi
  WiFi.softAP(ssid, password);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  
  if (client) {
    String request = client.readStringUntil('\r');
    Serial.println(request);

    // Motor kontrol
    if (request.indexOf("GET /motor") != -1) {
      if (request.indexOf("ileri") != -1) {
        digitalWrite(MOTOR_B, LOW);
        ledcWrite(PWM_CHANNEL, 255);   // tam hız
        motorHiz = 255;
        Serial.println("Motor İLERİ");
      }
      else if (request.indexOf("geri") != -1) {
        digitalWrite(MOTOR_B, HIGH);
        ledcWrite(PWM_CHANNEL, 255);   // tam hız
        motorHiz = -255;
        Serial.println("Motor GERİ");
      }
      else if (request.indexOf("dur") != -1) {
        ledcWrite(PWM_CHANNEL, 0);
        motorHiz = 0;
        Serial.println("Motor DUR");
      }
    }
    
    // Servo kontrol
    else if (request.indexOf("GET /servo") != -1) {
      if (request.indexOf("sol") != -1) {
        servoAci = 0;
        servo.write(servoAci);
        Serial.println("Servo SOL: 0°");
      }
      else if (request.indexOf("sag") != -1) {
        servoAci = 180;
        servo.write(servoAci);
        Serial.println("Servo SAĞ: 180°");
      }
      else if (request.indexOf("orta") != -1) {
        servoAci = 90;
        servo.write(servoAci);
        Serial.println("Servo ORTA: 90°");
      }
    }

    // HTTP yanıtı
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    
    // HTML arayüz
    client.println("<!DOCTYPE html>");
    client.println("<html lang='tr'>");
    client.println("<head>");
    client.println("<meta charset='UTF-8'>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
    client.println("<title>Sufi Kontrol</title>");
    client.println("<style>");
    client.println("body { font-family: Arial, sans-serif; text-align: center; margin: 20px; background-color: #f0f0f0; }");
    client.println(".container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 0 10px rgba(0,0,0,0.1); }");
    client.println(".btn { padding: 20px 30px; margin: 10px; font-size: 18px; border: none; border-radius: 5px; cursor: pointer; transition: all 0.3s; user-select:none; }");
    client.println(".motor-btn { background-color: #4CAF50; color: white; }");
    client.println(".motor-btn:active { background-color: #45a049; }");
    client.println(".servo-btn { background-color: #2196F3; color: white; }");
    client.println(".servo-btn:active { background-color: #0b7dda; }");
    client.println("</style>");
    client.println("</head>");
    client.println("<body>");
    client.println("<div class='container'>");
    client.println("<h1>🤖 Sufi Kontrol Paneli</h1>");
    
    // Motor kontrolü
    client.println("<h2>🎯 Motor</h2>");
    client.println("<button class='btn motor-btn' "
                   "onmousedown=\"fetch('/motor?ileri'); event.preventDefault();\" "
                   "onmouseup=\"fetch('/motor?dur')\" "
                   "onmouseleave=\"fetch('/motor?dur')\" "
                   "ontouchstart=\"fetch('/motor?ileri'); event.preventDefault();\" "
                   "ontouchend=\"fetch('/motor?dur')\" "
                   "ontouchcancel=\"fetch('/motor?dur')\">İLERİ</button><br>");
    
    client.println("<button class='btn motor-btn' "
                   "onmousedown=\"fetch('/motor?geri'); event.preventDefault();\" "
                   "onmouseup=\"fetch('/motor?dur')\" "
                   "onmouseleave=\"fetch('/motor?dur')\" "
                   "ontouchstart=\"fetch('/motor?geri'); event.preventDefault();\" "
                   "ontouchend=\"fetch('/motor?dur')\" "
                   "ontouchcancel=\"fetch('/motor?dur')\">GERİ</button>");
    
    // Servo kontrolü
    client.println("<h2>🎮 Servo</h2>");
    client.println("<button class='btn servo-btn' "
                   "onmousedown=\"fetch('/servo?sol')\" "
                   "onmouseup=\"fetch('/servo?orta')\" "
                   "ontouchstart=\"fetch('/servo?sol'); event.preventDefault();\" "
                   "ontouchend=\"fetch('/servo?orta')\" "
                   "ontouchcancel=\"fetch('/servo?orta')\">SOL</button>");
    
    client.println("<button class='btn servo-btn' "
                   "onmousedown=\"fetch('/servo?sag')\" "
                   "onmouseup=\"fetch('/servo?orta')\" "
                   "ontouchstart=\"fetch('/servo?sag'); event.preventDefault();\" "
                   "ontouchend=\"fetch('/servo?orta')\" "
                   "ontouchcancel=\"fetch('/servo?orta')\">SAĞ</button>");
    
    client.println("</div>");
    client.println("</body>");
    client.println("</html>");
    
    client.stop();
  }
}
