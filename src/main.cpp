#include <WiFi.h>
#include <ESP32Servo.h>

// WiFi bilgileri
const char* ssid = "Robot";
const char* password = "12345678";

// L298N Motor Pinleri
#define IN1 27
#define IN2 14
#define ENA 26  // Enable pini (motoru aktif etmek için HIGH yapılacak)

// Servo Pin
#define SERVO_PIN 13

Servo servo;
WiFiServer server(80);

int servoAci = 90;

void setup() {
  Serial.begin(115200);

  // Motor pinleri
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH); // motor sürücüsünü sürekli aktif et

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(90);

  // WiFi AP
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
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        Serial.println("Motor İLERİ");
      }
      else if (request.indexOf("geri") != -1) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        Serial.println("Motor GERİ");
      }
      else if (request.indexOf("dur") != -1) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        Serial.println("Motor DUR");
      }
    }

    // Servo kontrol
    else if (request.indexOf("GET /servo") != -1) {
      if (request.indexOf("sol") != -1) {
        servoAci = 0;
        servo.write(servoAci);
        Serial.println("Servo SOL");
      }
      else if (request.indexOf("sag") != -1) {
        servoAci = 180;
        servo.write(servoAci);
        Serial.println("Servo SAĞ");
      }
      else if (request.indexOf("orta") != -1) {
        servoAci = 90;
        servo.write(servoAci);
        Serial.println("Servo ORTA");
      }
    }

    // HTTP yanıtı
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
    client.println("<title>Robot Kontrol</title>");
    client.println("<style>");
    client.println("*{user-select:none !important; } body {font-family: Arial; text-align: center; }");
    client.println(".btn { padding: 20px 30px; margin: 10px; font-size: 18px; border-radius: 5px; cursor: pointer; -webkit-touch-callout: none; -webkit-user-select: none; -khtml-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none; }");
    client.println(".motor { background: #4CAF50; color: white; }");
    client.println(".servo { background: #2196F3; color: white; }");
    client.println("</style></head><body>");
    client.println("<h1>🤖 Robot Kontrol</h1>");

    // Motor butonları (basılı tutunca çalışır, bırakınca durur)
    client.println("<h2>Motor</h2>");
    client.println("<button class='btn motor' "
                  "ontouchstart=\"fetch('/motor?ileri')\" "
                  "ontouchend=\"fetch('/motor?dur')\" "
                  "onmousedown=\"fetch('/motor?ileri')\" "
                  "onmouseup=\"fetch('/motor?dur')\" "
                  "onmouseleave=\"fetch('/motor?dur')\" "
                  "oncontextmenu=\"return false;\">İLERİ</button><br>");
    client.println("<button class='btn motor' "
                  "ontouchstart=\"fetch('/motor?geri')\" "
                  "ontouchend=\"fetch('/motor?dur')\" "
                  "onmousedown=\"fetch('/motor?geri')\" "
                  "onmouseup=\"fetch('/motor?dur')\" "
                  "onmouseleave=\"fetch('/motor?dur')\" "
                  "oncontextmenu=\"return false;\">GERİ</button>");

    // Servo butonları
    client.println("<h2>Servo</h2>");
    client.println("<button class='btn servo' "
                  "ontouchstart=\"fetch('/servo?sol')\" "
                  "ontouchend=\"fetch('/servo?orta')\" "
                  "onmousedown=\"fetch('/servo?sol')\" "
                  "onmouseup=\"fetch('/servo?orta')\" "
                  "oncontextmenu=\"return false;\">SOL</button>");
    client.println("<button class='btn servo' "
                  "ontouchstart=\"fetch('/servo?sag')\" "
                  "ontouchend=\"fetch('/servo?orta')\" "
                  "onmousedown=\"fetch('/servo?sag')\" "
                  "onmouseup=\"fetch('/servo?orta')\" "
                  "oncontextmenu=\"return false;\">SAĞ</button>");

    client.println("</body></html>");
    client.stop();
  }
}
