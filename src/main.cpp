#include <WiFi.h>
#include <ESP32Servo.h>

// WiFi
const char* ssid = "Robot";
const char* password = "12345678";

// Pinler
#define MOTOR_A 14
#define MOTOR_B 27  
#define SERVO_PIN 13

Servo servo;
WiFiServer server(80);

int motorHiz = 0;
int servoAci = 90;

void setup() {
  Serial.begin(115200);
  
  // Motor
  pinMode(MOTOR_A, OUTPUT);
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
      int hiz = 0;
      if (request.indexOf("ileri") != -1) hiz = 255;
      else if (request.indexOf("geri") != -1) hiz = -255;
      
      if (hiz > 0) {
        digitalWrite(MOTOR_A, HIGH);
        digitalWrite(MOTOR_B, LOW);
      } else if (hiz < 0) {
        digitalWrite(MOTOR_A, LOW); 
        digitalWrite(MOTOR_B, HIGH);
      } else {
        digitalWrite(MOTOR_A, LOW);
        digitalWrite(MOTOR_B, LOW);
      }
      motorHiz = hiz;
    }
    
    // Servo kontrol
    else if (request.indexOf("GET /servo") != -1) {
      if (request.indexOf("sol") != -1) servoAci = 0;
      else if (request.indexOf("sag") != -1) servoAci = 180;
      else if (request.indexOf("orta") != -1) servoAci = 90;
      
      servo.write(servoAci);
    }

    // HTML gönder
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();
    client.println("<html>");
    client.println("<head><title>Robot Kontrol</title>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1'>");
    client.println("<style>");
    client.println("body { font-family: Arial; text-align: center; }");
    client.println(".btn { padding: 15px 25px; margin: 10px; font-size: 18px; }");
    client.println("</style></head>");
    client.println("<body>");
    client.println("<h1>Robot Kontrol</h1>");
    
    client.println("<h2>Motor</h2>");
    client.println("<a href='/motor?ileri'><button class='btn'>İLERİ</button></a><br>");
    client.println("<a href='/motor?dur'><button class='btn'>DUR</button></a><br>");
    client.println("<a href='/motor?geri'><button class='btn'>GERİ</button></a>");
    
    client.println("<h2>Servo</h2>");
    client.println("<a href='/servo?sol'><button class='btn'>SOL</button></a>");
    client.println("<a href='/servo?orta'><button class='btn'>ORTA</button></a>");
    client.println("<a href='/servo?sag'><button class='btn'>SAĞ</button></a>");
    
    client.println("</body></html>");
    
    client.stop();
  }
}