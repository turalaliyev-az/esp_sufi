#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== WiFi Ayarları ====================
const char* ssid = "Robot";
const char* password = "12345678";

// ------ ULTRA SONIK SENSÖR PINLERİ ------
#define TRIG_PIN 5
#define ECHO_PIN 18
#define TRIG_PIN_2 19
#define ECHO_PIN_2 4
#define TRIG_PIN_3 2
#define ECHO_PIN_3 23
#define TRIG_PIN_4 15  // 4. sensör için yeni pinler
#define ECHO_PIN_4 2

//------- max distance -------
#define MAX_DISTANCE 200
#define SoundSpeed 0.0343
#define CM_TO_INCH 0.393701

// ==================== PID AYARLARI ====================
struct PIDController {
  double Kp, Ki, Kd;
  double setpoint;
  double integral;
  double previous_error;
  double output;
  unsigned long last_time;
  double output_min, output_max;
  String name;
};

// ==================== Mesafe Sensörü Yapılandırması ====================
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  float distance;
  float filtered_distance;
  String name;
  PIDController pid;
};

UltrasonicSensor sensors[4] = {
  {TRIG_PIN, ECHO_PIN, 0, 0, "Ön Sol", {0.8, 0.1, 0.05, 25.0, 0, 0, 0, 0, 0, 100, "Ön Sol PID"}},
  {TRIG_PIN_2, ECHO_PIN_2, 0, 0, "Ön Sağ", {0.8, 0.1, 0.05, 25.0, 0, 0, 0, 0, 0, 100, "Ön Sağ PID"}},
  {TRIG_PIN_3, ECHO_PIN_3, 0, 0, "Arka", {1.0, 0.2, 0.1, 30.0, 0, 0, 0, 0, 0, 100, "Arka PID"}},
  {TRIG_PIN_4, ECHO_PIN_4, 0, 0, "Yan", {0.6, 0.05, 0.02, 20.0, 0, 0, 0, 0, 0, 100, "Yan PID"}}
};

// ==================== Otonom Mod Ayarları ====================
bool autonomousMode = false;
const float OBSTACLE_DISTANCE = 30.0;
const float CRITICAL_DISTANCE = 15.0;
const float TARGET_FOLLOW_DISTANCE = 25.0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 50;

// ==================== Motor PID Kontrol ====================
PIDController motorPID = {2.0, 0.5, 0.2, TARGET_FOLLOW_DISTANCE, 0, 0, 0, 0, -255, 255, "Motor PID"};
bool wallFollowingMode = false;

// ==================== L298N Motor Pinleri ====================
#define IN1 27
#define IN2 14
#define ENA 26

// ==================== Yerel Servo Pin ====================
#define LOCAL_SERVO_PIN 13

// ==================== PCA9685 Ayarları ====================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVO_MIN  150
#define SERVO_MAX  600

// ==================== Global Nesneler ====================
Servo localServo;
WiFiServer server(80);

// ==================== Servo Kontrol Sistemi ====================
struct PcaServo {
  uint8_t channel;
  uint16_t currentAngle;
  uint16_t targetAngle;
  uint16_t resetAngle;
  uint16_t minAngle;
  uint16_t maxAngle;
  String name;
};

PcaServo availableServos[16] = {
  {8, 100, 100, 100, 100, 150, "Kol_Servo_2"},
  {4, 72, 72, 72, 72, 122, "Kol_Servo_1"},
  {6, 126, 126, 126, 68, 126, "Bacak_Servo_1"},
  {9, 50, 50, 50, 50, 120, "Bacak_Servo_2"},
  {10, 130, 130, 130, 130, 160, "Baş_Servo"},
  {13, 92, 92, 92, 40, 92, "Kuyruk_Servo"},
  {11, 133, 133, 133, 105, 133, "Göz_Servo"},
  {12, 50, 50, 50, 50, 145, "Ağız_Servo"},
  {0, 90, 90, 90, 0, 180, "Kanal_0"},
  {1, 90, 90, 90, 0, 180, "Kanal_1"},
  {2, 90, 90, 90, 0, 180, "Kanal_2"},
  {3, 90, 90, 90, 0, 180, "Kanal_3"},
  {5, 90, 90, 90, 0, 180, "Kanal_5"},
  {7, 90, 90, 90, 0, 180, "Kanal_7"},
  {14, 90, 90, 90, 0, 180, "Kanal_14"},
  {15, 90, 90, 90, 0, 180, "Kanal_15"}
};

const int TOTAL_CHANNELS = 16;
const int ACTIVE_SERVO_COUNT = 8;

// ==================== SERİ HABERLEŞME ====================
const uint16_t SERIAL_BUFFER_SIZE = 512;
char serialBuffer[512];
uint16_t bufferIndex = 0;
bool commandReady = false;

// ==================== Değişkenler ====================
int localServoAngle = 90;
const int LOCAL_SERVO_HOME_ANGLE = 90;
unsigned long lastPcaUpdate = 0;
const unsigned long PCA_UPDATE_INTERVAL = 20;

// Yeni eklenen değişkenler
unsigned long lastSerialActivity = 0;
const unsigned long RESET_TIMEOUT = 3000;
bool resetExecuted = false;

// PID Debug değişkenleri
bool pidDebug = false;
unsigned long lastPidDebug = 0;
const unsigned long PID_DEBUG_INTERVAL = 1000;

// ==================== Ön Fonksiyon Bildirimleri ====================
void printHelp();
void printServoStatus();
void resetAllServos();
void resetOnlyServos();
void sendWebPage(WiFiClient &client);
void scanI2CDevices();
bool setPcaServoAngleByChannel(uint8_t channel, int angle);
void processFlexibleCommand(const char* command);
void clearSerialBuffer();
void setMotorForward();
void setMotorBackward();
void stopMotor();
void setMotorSpeed(int speed);
void setLocalServo(int angle);
void printResetPositions();
void setupServo(uint8_t index, uint8_t channel, uint16_t startAngle, uint16_t minAngle, uint16_t maxAngle, String name = "");
void runServoTest();
void autoResetIfNoActivity();
void setupUltrasonicSensors();
float readUltrasonicDistance(int trigPin, int echoPin);
void updateAllSensors();
void autonomousMovement();
void setAutonomousMode(bool enabled);
void handleWebClient(WiFiClient &client);
double computePID(PIDController& pid, double input);
void initializePIDs();
void updateFilteredDistances();
void wallFollowing();
void printPidDebug();
void handlePidUpdate(WiFiClient &client, String request);
void resetPidParameters();
void sendPidSettingsPage(WiFiClient &client);
void updatePidFromWeb(String pidName, double kp, double ki, double kd, double setpoint);
String getValue(String data, String separator, String terminator = " ");
void handleLocalServoPress(int angle);
void handleLocalServoRelease();
void checkSerial();
void processSerialCommand();

// ==================== PID FONKSİYONLARI ====================

void initializePIDs() {
  Serial.println("🎯 PID Kontrolcüleri Başlatılıyor...");
  
  unsigned long now = millis();
  for (int i = 0; i < 4; i++) {
    sensors[i].pid.last_time = now;
    sensors[i].pid.integral = 0;
    sensors[i].pid.previous_error = 0;
    
    Serial.printf("✅ %s Sensör PID: Kp=%.2f, Ki=%.2f, Kd=%.2f, Setpoint=%.1fcm\n",
                  sensors[i].name.c_str(),
                  sensors[i].pid.Kp,
                  sensors[i].pid.Ki, 
                  sensors[i].pid.Kd,
                  sensors[i].pid.setpoint);
  }
  
  motorPID.last_time = now;
  motorPID.integral = 0;
  motorPID.previous_error = 0;
  
  Serial.println("✅ Motor PID Kontrolcüsü Başlatıldı");
}

void resetPidParameters() {
  Serial.println("🔄 PID Parametreleri Varsayılan Değerlere Sıfırlanıyor...");
  
  // Sensör PID'lerini sıfırla
  sensors[0].pid = {0.8, 0.1, 0.05, 25.0, 0, 0, 0, 0, 0, 100, "Ön Sol PID"};
  sensors[1].pid = {0.8, 0.1, 0.05, 25.0, 0, 0, 0, 0, 0, 100, "Ön Sağ PID"};
  sensors[2].pid = {1.0, 0.2, 0.1, 30.0, 0, 0, 0, 0, 0, 100, "Arka PID"};
  sensors[3].pid = {0.6, 0.05, 0.02, 20.0, 0, 0, 0, 0, 0, 100, "Yan PID"};
  
  // Motor PID'ini sıfırla
  motorPID = {2.0, 0.5, 0.2, TARGET_FOLLOW_DISTANCE, 0, 0, 0, 0, -255, 255, "Motor PID"};
  
  initializePIDs();
  Serial.println("✅ Tüm PID parametreleri sıfırlandı!");
}

void updatePidFromWeb(String pidName, double kp, double ki, double kd, double setpoint) {
  Serial.printf("🔧 PID Güncelleme: %s -> Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%.1f\n",
                pidName.c_str(), kp, ki, kd, setpoint);
  
  if (pidName == "motor") {
    motorPID.Kp = kp;
    motorPID.Ki = ki;
    motorPID.Kd = kd;
    motorPID.setpoint = setpoint;
    Serial.println("✅ Motor PID güncellendi!");
  }
  else {
    for (int i = 0; i < 4; i++) {
      if (sensors[i].pid.name == pidName) {
        sensors[i].pid.Kp = kp;
        sensors[i].pid.Ki = ki;
        sensors[i].pid.Kd = kd;
        sensors[i].pid.setpoint = setpoint;
        Serial.printf("✅ %s sensör PID güncellendi!\n", sensors[i].name.c_str());
        break;
      }
    }
  }
}

double computePID(PIDController& pid, double input) {
  unsigned long now = millis();
  double dt = (now - pid.last_time) / 1000.0;
  
  if (dt <= 0) return pid.output;
  
  double error = pid.setpoint - input;
  
  // Integral term
  pid.integral += error * dt;
  
  // Integral windup protection
  if (pid.Ki != 0) {
    double max_integral = pid.output_max / pid.Ki;
    if (pid.integral > max_integral) pid.integral = max_integral;
    if (pid.integral < -max_integral) pid.integral = -max_integral;
  }
  
  // Derivative term
  double derivative = (error - pid.previous_error) / dt;
  
  // PID output
  double output = (pid.Kp * error) + (pid.Ki * pid.integral) + (pid.Kd * derivative);
  
  // Output clamping
  if (output > pid.output_max) output = pid.output_max;
  if (output < pid.output_min) output = pid.output_min;
  
  pid.previous_error = error;
  pid.last_time = now;
  pid.output = output;
  
  return output;
}

void updateFilteredDistances() {
  for (int i = 0; i < 4; i++) {
    double pid_output = computePID(sensors[i].pid, sensors[i].distance);
    sensors[i].filtered_distance = sensors[i].distance + (pid_output * 0.1);
    sensors[i].filtered_distance = constrain(sensors[i].filtered_distance, 0, MAX_DISTANCE);
    
    if (pidDebug && millis() - lastPidDebug > PID_DEBUG_INTERVAL) {
      Serial.printf("🔧 %s: Ham=%.1fcm, Filtre=%.1fcm, PID=%.2f\n",
                    sensors[i].name.c_str(),
                    sensors[i].distance,
                    sensors[i].filtered_distance,
                    pid_output);
    }
  }
  
  if (pidDebug && millis() - lastPidDebug > PID_DEBUG_INTERVAL) {
    lastPidDebug = millis();
  }
}

// ==================== WEB PID KONTROL FONKSİYONLARI ====================

String getValue(String data, String separator, String terminator) {
  int startIndex = data.indexOf(separator);
  if (startIndex == -1) return "";
  startIndex += separator.length();
  
  int endIndex = data.indexOf(terminator, startIndex);
  if (endIndex == -1) endIndex = data.length();
  
  return data.substring(startIndex, endIndex);
}

void handlePidUpdate(WiFiClient &client, String request) {
  // PID güncelleme isteğini parse et
  String pidName = getValue(request, "name=", "&");
  double kp = getValue(request, "kp=", "&").toFloat();
  double ki = getValue(request, "ki=", "&").toFloat();
  double kd = getValue(request, "kd=", "&").toFloat();
  double setpoint = getValue(request, "setpoint=", " ").toFloat();
  
  updatePidFromWeb(pidName, kp, ki, kd, setpoint);
  
  // JSON yanıt gönder
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:application/json");
  client.println("Connection: close");
  client.println();
  client.println("{\"status\":\"success\", \"message\":\"PID parameters updated\"}");
}

void sendPidSettingsPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  
  client.println("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  client.println("<title>PID Ayarları - Robot Kontrol</title>");
  client.println("<style>");
  client.println("body {font-family: Arial; text-align: center; background: #f0f0f0; margin: 0; padding: 20px;}");
  client.println(".container {background: white; padding: 20px; margin: 15px; border-radius: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);}");
  client.println(".pid-form {display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin: 20px 0;}");
  client.println(".form-group {text-align: left;}");
  client.println("label {display: block; margin-bottom: 5px; font-weight: bold; color: #333;}");
  client.println("input {width: 100%; padding: 10px; border: 2px solid #ddd; border-radius: 8px; font-size: 16px;}");
  client.println(".btn {padding: 15px 25px; margin: 10px; font-size: 16px; border-radius: 10px; cursor: pointer; border: none; background: #2196F3; color: white; font-weight: bold;}");
  client.println(".btn:hover {background: #1976D2;}");
  client.println(".reset-btn {background: #FF5722;}");
  client.println(".back-btn {background: #607D8B;}");
  client.println(".pid-section {border: 2px solid #2196F3; border-radius: 10px; padding: 15px; margin: 15px 0;}");
  client.println("</style>");
  client.println("<script>");
  client.println("function updatePid(pidName) {");
  client.println("  const kp = document.getElementById(pidName + '-kp').value;");
  client.println("  const ki = document.getElementById(pidName + '-ki').value;");
  client.println("  const kd = document.getElementById(pidName + '-kd').value;");
  client.println("  const setpoint = document.getElementById(pidName + '-setpoint').value;");
  client.println("  ");
  client.println("  fetch('/pid?name=' + pidName + '&kp=' + kp + '&ki=' + ki + '&kd=' + kd + '&setpoint=' + setpoint)");
  client.println("    .then(response => response.json())");
  client.println("    .then(data => {");
  client.println("      alert('✅ ' + pidName + ' PID ayarları güncellendi!');");
  client.println("    });");
  client.println("}");
  client.println("function resetAllPids() {");
  client.println("  if(confirm('Tüm PID ayarları varsayılan değerlere sıfırlanacak. Emin misiniz?')) {");
  client.println("    fetch('/resetpid')");
  client.println("      .then(response => response.json())");
  client.println("      .then(data => {");
  client.println("        alert('✅ Tüm PID ayarları sıfırlandı! Sayfa yeniden yükleniyor...');");
  client.println("        location.reload();");
  client.println("      });");
  client.println("  }");
  client.println("}");
  client.println("</script>");
  client.println("</head><body>");
  
  client.println("<h1>🎯 PID Kontrol Ayarları</h1>");
  client.println("<div class='container'>");
  
  // Motor PID Ayarları
  client.println("<div class='pid-section'>");
  client.println("<h2>🏎️ Motor PID Ayarları</h2>");
  client.println("<div class='pid-form'>");
  client.println("<div class='form-group'><label>Kp (Oransal):</label><input type='number' step='0.1' id='motor-kp' value='" + String(motorPID.Kp, 3) + "'></div>");
  client.println("<div class='form-group'><label>Ki (Integral):</label><input type='number' step='0.01' id='motor-ki' value='" + String(motorPID.Ki, 3) + "'></div>");
  client.println("<div class='form-group'><label>Kd (Türev):</label><input type='number' step='0.01' id='motor-kd' value='" + String(motorPID.Kd, 3) + "'></div>");
  client.println("<div class='form-group'><label>Setpoint (cm):</label><input type='number' step='1' id='motor-setpoint' value='" + String(motorPID.setpoint, 1) + "'></div>");
  client.println("</div>");
  client.println("<button class='btn' onclick='updatePid(\"motor\")'>Motor PID Güncelle</button>");
  client.println("</div>");
  
  // Sensör PID Ayarları
  for (int i = 0; i < 4; i++) {
    client.println("<div class='pid-section'>");
    client.println("<h2>📡 " + sensors[i].name + " Sensör PID</h2>");
    client.println("<div class='pid-form'>");
    client.println("<div class='form-group'><label>Kp (Oransal):</label><input type='number' step='0.1' id='" + sensors[i].pid.name + "-kp' value='" + String(sensors[i].pid.Kp, 3) + "'></div>");
    client.println("<div class='form-group'><label>Ki (Integral):</label><input type='number' step='0.01' id='" + sensors[i].pid.name + "-ki' value='" + String(sensors[i].pid.Ki, 3) + "'></div>");
    client.println("<div class='form-group'><label>Kd (Türev):</label><input type='number' step='0.01' id='" + sensors[i].pid.name + "-kd' value='" + String(sensors[i].pid.Kd, 3) + "'></div>");
    client.println("<div class='form-group'><label>Setpoint (cm):</label><input type='number' step='1' id='" + sensors[i].pid.name + "-setpoint' value='" + String(sensors[i].pid.setpoint, 1) + "'></div>");
    client.println("</div>");
    client.println("<button class='btn' onclick='updatePid(\"" + sensors[i].pid.name + "\")'>" + sensors[i].name + " PID Güncelle</button>");
    client.println("</div>");
  }
  
  // Kontrol Butonları
  client.println("<div style='margin-top: 30px;'>");
  client.println("<button class='btn reset-btn' onclick='resetAllPids()'>🔁 Tüm PID Ayarlarını Sıfırla</button>");
  client.println("<button class='btn back-btn' onclick='window.location.href=\"/\"'>🔙 Ana Sayfaya Dön</button>");
  client.println("</div>");
  
  client.println("</div>"); // container kapat
  
  // Açıklama
  client.println("<div class='container'>");
  client.println("<h3>🎓 PID Parametreleri Açıklaması</h3>");
  client.println("<div style='text-align: left;'>");
  client.println("<p><strong>Kp (Oransal Kazanç):</strong> Mevcut hata ile orantılı tepki. Büyük Kp = Hızlı tepki, küçük Kp = Yavaş tepki</p>");
  client.println("<p><strong>Ki (Integral Kazanç):</strong> Geçmiş hataların toplamı. Sabit hataları giderir, çok büyükse osilasyon yapar</p>");
  client.println("<p><strong>Kd (Türev Kazanç):</strong> Hatadaki değişim oranı. Aşımı (overshoot) azaltır, sistemi stabilize eder</p>");
  client.println("<p><strong>Setpoint:</strong> Hedef mesafe (cm). Robotun bu mesafeyi korumaya çalışacağı değer</p>");
  client.println("</div>");
  client.println("</div>");
  
  client.println("</body></html>");
}

// ==================== WEB SERVER İŞLEME ====================
void handleWebClient(WiFiClient &client) {
  String request = client.readStringUntil('\r');
  Serial.println("🌐 Web İsteği: " + request);

  // PID ayar sayfası
  if (request.indexOf("GET /pidsettings") != -1) {
    sendPidSettingsPage(client);
    return;
  }
  
  // PID güncelleme isteği
  if (request.indexOf("GET /pid?") != -1) {
    handlePidUpdate(client, request);
    return;
  }
  
  // PID reset isteği
  if (request.indexOf("GET /resetpid") != -1) {
    resetPidParameters();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:application/json");
    client.println("Connection: close");
    client.println();
    client.println("{\"status\":\"success\", \"message\":\"All PID parameters reset to default\"}");
    return;
  }
  
  // Otonom mod kontrolü
  if (request.indexOf("GET /autonomous?enable=true") != -1) {
    setAutonomousMode(true);
  }
  else if (request.indexOf("GET /autonomous?enable=false") != -1) {
    setAutonomousMode(false);
  }
  // Servo kontrolü
  else if (request.indexOf("GET /servo?press=") != -1) {
    if (request.indexOf("sol") != -1) {
      handleLocalServoPress(0);
    }
    else if (request.indexOf("sag") != -1) {
      handleLocalServoPress(180);
    }
    else if (request.indexOf("orta") != -1) {
      handleLocalServoPress(90);
    }
  }
  else if (request.indexOf("GET /servo?release") != -1) {
    handleLocalServoRelease();
  }
  // Motor kontrolü
  else if (request.indexOf("GET /motor") != -1) {
    setAutonomousMode(false); // Manuel kontrol için otonom modu kapat
    if (request.indexOf("ileri") != -1) setMotorForward();
    else if (request.indexOf("geri") != -1) setMotorBackward();
    else if (request.indexOf("dur") != -1) stopMotor();
  }
  // Reset işlemleri
  else if (request.indexOf("GET /reset") != -1) {
    resetAllServos();
  }
  else if (request.indexOf("GET /resetservo") != -1) {
    resetOnlyServos();
  }
  // Sensör verileri
  else if (request.indexOf("GET /sensors") != -1) {
    updateAllSensors();
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:application/json");
    client.println("Connection: close");
    client.println();
    client.print("{\"sensors\":[");
    for (int i = 0; i < 4; i++) {
      client.printf("{\"name\":\"%s\",\"distance\":%.1f,\"filtered\":%.1f}", sensors[i].name.c_str(), sensors[i].distance, sensors[i].filtered_distance);
      if (i < 3) client.print(",");
    }
    client.print("],\"autonomousMode\":");
    client.print(autonomousMode ? "true" : "false");
    client.print("}");
    return;
  }

  // Ana sayfayı gönder
  sendWebPage(client);
}

void sendWebPage(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html><html><head><meta charset='UTF-8'>");
  client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
  client.println("<title>Robot Kontrol</title>");
  client.println("<style>");
  client.println("*{user-select:none; -webkit-tap-highlight-color: transparent; }");
  client.println("body {font-family: Arial; text-align: center; background: #f0f0f0; margin: 0; padding: 20px; }");
  client.println(".btn { padding: 25px 35px; margin: 12px; font-size: 20px; border-radius: 15px; cursor: pointer; border: none; transition: all 0.2s; font-weight: bold; }");
  client.println(".btn:active { transform: scale(0.92); opacity: 0.9; }");
  client.println(".servo-btn { background: linear-gradient(145deg, #2196F3, #1976D2); color: white; box-shadow: 0 6px 0 #0D47A1; }");
  client.println(".servo-btn:active { box-shadow: 0 2px 0 #0D47A1; transform: translateY(4px) scale(0.95); }");
  client.println(".motor { background: #4CAF50; color: white; }");
  client.println(".reset { background: #FF5722; color: white; }");
  client.println(".autonomous { background: #9C27B0; color: white; }");
  client.println(".sensor { background: #FF9800; color: white; }");
  client.println(".pid-btn { background: #E91E63; color: white; }");
  client.println(".info { background: #607D8B; color: white; padding: 15px; margin: 15px; border-radius: 10px; }");
  client.println(".container { background: white; padding: 20px; margin: 15px; border-radius: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }");
  client.println(".status { font-size: 18px; margin: 10px; padding: 10px; background: #E3F2FD; border-radius: 8px; }");
  client.println(".autonomous-active { background: #4CAF50 !important; color: white !important; }");
  client.println(".autonomous-inactive { background: #f44336 !important; color: white !important; }");
  client.println(".sensor-data { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin: 15px 0; }");
  client.println(".sensor-item { background: #FFF3E0; padding: 10px; border-radius: 8px; font-size: 16px; }");
  client.println("</style>");
  client.println("<script>");
  client.println("let sensorUpdateInterval;");
  client.println("function servoPress(direction) {");
  client.println("  fetch('/servo?press=' + direction);");
  client.println("  updateServoStatus(direction + ' (Basılı)');");
  client.println("}");
  client.println("function servoRelease() {");
  client.println("  fetch('/servo?release');");
  client.println("  updateServoStatus('Orta (Serbest)');");
  client.println("}");
  client.println("function updateServoStatus(status) {");
  client.println("  document.getElementById('servoStatus').innerText = 'Servo Durum: ' + status;");
  client.println("}");
  client.println("function setAutonomousMode(enabled) {");
  client.println("  fetch('/autonomous?enable=' + enabled);");
  client.println("  document.getElementById('autonomousStatus').innerText = 'Otonom Mod: ' + (enabled ? 'AKTİF' : 'PASİF');");
  client.println("  document.getElementById('autonomousStatus').className = enabled ? 'status autonomous-active' : 'status autonomous-inactive';");
  client.println("  document.getElementById('autoBtn').style.display = enabled ? 'none' : 'block';");
  client.println("  document.getElementById('manualBtn').style.display = enabled ? 'block' : 'none';");
  client.println("}");
  client.println("function updateSensors() {");
  client.println("  fetch('/sensors')");
  client.println("    .then(response => response.json())");
  client.println("    .then(data => {");
  client.println("      document.getElementById('sensorData').innerHTML = '';");
  client.println("      data.sensors.forEach(sensor => {");
  client.println("        const item = document.createElement('div');");
  client.println("        item.className = 'sensor-item';");
  client.println("        item.innerHTML = `<strong>${sensor.name}:</strong> ${sensor.distance.toFixed(1)} cm (Filtre: ${sensor.filtered.toFixed(1)} cm)`;");
  client.println("        document.getElementById('sensorData').appendChild(item);");
  client.println("      });");
  client.println("      if(data.autonomousMode) {");
  client.println("        setAutonomousMode(true);");
  client.println("      }");
  client.println("    });");
  client.println("}");
  client.println("// Touch events for mobile support");
  client.println("function addTouchEvents(buttonId, direction) {");
  client.println("  var btn = document.getElementById(buttonId);");
  client.println("  btn.addEventListener('touchstart', function(e) { e.preventDefault(); servoPress(direction); });");
  client.println("  btn.addEventListener('touchend', function(e) { e.preventDefault(); servoRelease(); });");
  client.println("  btn.addEventListener('touchcancel', function(e) { e.preventDefault(); servoRelease(); });");
  client.println("}");
  client.println("document.addEventListener('DOMContentLoaded', function() {");
  client.println("  addTouchEvents('btnSol', 'sol');");
  client.println("  addTouchEvents('btnSag', 'sag');");
  client.println("  addTouchEvents('btnOrta', 'orta');");
  client.println("  setAutonomousMode(false);");
  client.println("  updateSensors();");
  client.println("  sensorUpdateInterval = setInterval(updateSensors, 2000);");
  client.println("});");
  client.println("</script>");
  client.println("</head><body>");
  client.println("<h1>🤖 Robot Kontrol Paneli</h1>");

  // PID Ayarları Butonu
  client.println("<div class='container'>");
  client.println("<h2>🎯 PID Kontrol</h2>");
  client.println("<button class='btn pid-btn' onclick='window.location.href=\"/pidsettings\"'>🔧 PID AYARLARINI DÜZENLE</button>");
  client.println("</div>");

  // Otonom Mod Kontrolü
  client.println("<div class='container'>");
  client.println("<h2>🚀 Otonom Mod Kontrolü</h2>");
  client.println("<div class='status' id='autonomousStatus'>Otonom Mod: PASİF</div>");
  client.println("<button id='autoBtn' class='btn autonomous' onclick='setAutonomousMode(true)'>OTONOM MODU BAŞLAT</button>");
  client.println("<button id='manualBtn' class='btn autonomous' onclick='setAutonomousMode(false)' style='display:none'>OTONOM MODU DURDUR</button>");
  client.println("</div>");

  // Sensör Verileri
  client.println("<div class='container'>");
  client.println("<h2>📊 Sensör Verileri</h2>");
  client.println("<div class='sensor-data' id='sensorData'></div>");
  client.println("<button class='btn sensor' onclick='updateSensors()'>SENSÖRLERİ GÜNCELLE</button>");
  client.println("</div>");

  // Yerel servo kontrol bölümü
  client.println("<div class='container'>");
  client.println("<h2>🎮 Yerel Servo Kontrol (Basılı Tut)</h2>");
  client.println("<div class='status' id='servoStatus'>Servo Durum: Orta (Serbest)</div>");
  client.println("<div>");
  client.println("<button id='btnSol' class='btn servo-btn' ");
  client.println("onmousedown='servoPress(\"sol\")' ");
  client.println("onmouseup='servoRelease()' ");
  client.println("onmouseleave='servoRelease()' ");
  client.println(">SOL (0°)</button>");
  
  client.println("<button id='btnOrta' class='btn servo-btn' ");
  client.println("onmousedown='servoPress(\"orta\")' ");
  client.println("onmouseup='servoRelease()' ");
  client.println("onmouseleave='servoRelease()' ");
  client.println(">ORTA (90°)</button>");
  
  client.println("<button id='btnSag' class='btn servo-btn' ");
  client.println("onmousedown='servoPress(\"sag\")' ");
  client.println("onmouseup='servoRelease()' ");
  client.println("onmouseleave='servoRelease()' ");
  client.println(">SAĞ (180°)</button>");
  client.println("</div>");
  client.println("<p style='color: #666; font-size: 14px; margin-top: 15px;'>");
  client.println("💡 Butonlara basılı tutun → servo hareket eder<br>");
  client.println("Butonu bırakın → servo orta pozisyona döner");
  client.println("</p>");
  client.println("</div>");

  // Motor kontrol bölümü
  client.println("<div class='container'>");
  client.println("<h2>🏎️ Motor Kontrol</h2>");
  client.println("<button class='btn motor' ontouchstart=\"fetch('/motor?ileri')\" ontouchend=\"fetch('/motor?dur')\" onmousedown=\"fetch('/motor?ileri')\" onmouseup=\"fetch('/motor?dur')\">İLERİ</button><br>");
  client.println("<button class='btn motor' ontouchstart=\"fetch('/motor?geri')\" ontouchend=\"fetch('/motor?dur')\" onmousedown=\"fetch('/motor?geri')\" onmouseup=\"fetch('/motor?dur')\">GERİ</button>");
  client.println("</div>");

  // Reset bölümü
  client.println("<div class='container'>");
  client.println("<h2>🔄 Sistem Kontrol</h2>");
  client.println("<button class='btn reset' onclick=\"fetch('/reset')\">TÜM SİSTEMİ RESETLE</button><br>");
  client.println("<button class='btn reset' onclick=\"fetch('/resetservo')\">SADECE SERVOLARI RESETLE</button>");
  client.println("</div>");

  // Bilgi bölümü
  client.println("<div class='info'>");
  client.println("<strong>🎯 KULLANIM KILAVUZU</strong><br>");
  client.println("• PID Ayarları: Web üzerinden gerçek zamanlı PID kontrolü<br>");
  client.println("• Otonom Mod: Robot engellerden kaçarak kendi hareket eder<br>");
  client.println("• Yerel Servo: Butonlara basılı tutun → servo hareket eder<br>");
  client.println("• Motor: Basılı tutun → motor çalışır, bırakın → motor durur<br>");
  client.println("• Sensörler: 4 ultrasonik sensör ile PID filtrelenmiş mesafe ölçümü<br>");
  client.println("• Reset: Tüm sistem veya sadece servoları sıfırlar");
  client.println("</div>");

  client.println("</body></html>");
}

// ==================== OTONOM MOD FONKSİYONLARI ====================

void setAutonomousMode(bool enabled) {
  if (autonomousMode != enabled) {
    autonomousMode = enabled;
    wallFollowingMode = false;
    
    if (enabled) {
      Serial.println("🚀 OTONOM MOD AKTİF - PID Kontrollü Hareket");
      stopMotor();
      initializePIDs();
    } else {
      Serial.println("🔴 OTONOM MOD PASİF - Manuel kontrol aktif");
      stopMotor();
    }
  }
}

void setWallFollowingMode(bool enabled) {
  wallFollowingMode = enabled;
  if (enabled) {
    Serial.println("🧱 DUVAR TAKİP MODU AKTİF");
    motorPID.setpoint = TARGET_FOLLOW_DISTANCE;
  }
}

void updateAllSensors() {
  for (int i = 0; i < 4; i++) {
    sensors[i].distance = readUltrasonicDistance(sensors[i].trigPin, sensors[i].echoPin);
  }
  updateFilteredDistances();
}

void wallFollowing() {
  float left_dist = sensors[0].filtered_distance;
  float right_dist = sensors[1].filtered_distance;
  float front_dist = fmin(left_dist, right_dist);
  
  double motor_output = computePID(motorPID, front_dist);
  
  if (pidDebug && millis() - lastPidDebug > PID_DEBUG_INTERVAL) {
    Serial.printf("🏎️ Motor PID: Mesafe=%.1fcm, Setpoint=%.1fcm, Output=%.1f\n",
                  front_dist, motorPID.setpoint, motor_output);
  }
  
  if (abs(motor_output) < 10) {
    stopMotor();
    Serial.println("🎯 Hedef mesafede - Duruyor");
  } else if (motor_output > 0) {
    setMotorSpeed(constrain(motor_output, 0, 255));
    Serial.printf("🚀 İleri git - Hız: %.0f\n", motor_output);
  } else {
    setMotorSpeed(constrain(motor_output, -255, 0));
    Serial.printf("🔁 Geri git - Hız: %.0f\n", -motor_output);
  }
}

void autonomousMovement() {
  updateAllSensors();
  
  float front_left = sensors[0].filtered_distance;
  float front_right = sensors[1].filtered_distance;
  float back = sensors[2].filtered_distance;
  float side = sensors[3].filtered_distance;
  
  float min_front = fmin(front_left, front_right);
  
  // Kritik mesafe kontrolü - acil dur
  if (min_front < CRITICAL_DISTANCE) {
    Serial.println("🚨 KRİTİK ENGELE - ACİL DUR!");
    stopMotor();
    delay(500);
    
    setMotorBackward();
    delay(300);
    stopMotor();
    delay(200);
    
    // Servo ile etrafa bak
    setLocalServo(0);
    delay(300);
    float left_space = sensors[0].filtered_distance;
    
    setLocalServo(180);
    delay(300);
    float right_space = sensors[1].filtered_distance;
    
    setLocalServo(90);
    delay(300);
    
    if (left_space > right_space && left_space > OBSTACLE_DISTANCE) {
      Serial.println("↩️ SOLA DÖNÜYORUM (PID Optimize)");
      setLocalServo(0);
      setMotorForward();
      delay(400);
    } else if (right_space > OBSTACLE_DISTANCE) {
      Serial.println("↪️ SAĞA DÖNÜYORUM (PID Optimize)");
      setLocalServo(180);
      setMotorForward();
      delay(400);
    } else {
      Serial.println("🔁 GERİ DÖNÜYORUM (PID Optimize)");
      setMotorBackward();
      delay(600);
    }
    
    setLocalServo(90);
    stopMotor();
    delay(200);
    return;
  }
  
  // Duvar takip modu
  if (wallFollowingMode && min_front < 50.0) {
    wallFollowing();
    return;
  }
  
  // Normal engel algılama
  if (min_front < OBSTACLE_DISTANCE) {
    Serial.println("⚠️ ENGEL ALGILANDI - PID İLE YÖN DEĞİŞTİRİYORUM");
    stopMotor();
    delay(200);
    
    float left_quality = front_left - (sensors[3].filtered_distance * 0.3);
    float right_quality = front_right - (sensors[3].filtered_distance * 0.3);
    
    if (left_quality > right_quality && left_quality > 10) {
      Serial.println("↩️ SOLA DÖNÜYORUM (PID Kalite)");
      setLocalServo(0);
      setMotorForward();
      delay(400);
    } else if (right_quality > 10) {
      Serial.println("↪️ SAĞA DÖNÜYORUM (PID Kalite)");
      setLocalServo(180);
      setMotorForward();
      delay(400);
    } else {
      Serial.println("🔁 GERİ DÖNÜYORUM (PID Kalite)");
      setMotorBackward();
      delay(500);
    }
    
    setLocalServo(90);
    stopMotor();
    delay(200);
  } else {
    Serial.println("✅ YOL AÇIK - PID İLE İLERİ GİDİYORUM");
    setMotorForward();
  }
}

// ==================== SERVO KURULUM FONKSİYONU ====================
void setupServo(uint8_t index, uint8_t channel, uint16_t startAngle, uint16_t minAngle, uint16_t maxAngle, String name) {
  if (index < TOTAL_CHANNELS) {
    availableServos[index].channel = channel;
    availableServos[index].currentAngle = startAngle;
    availableServos[index].targetAngle = startAngle;
    availableServos[index].resetAngle = startAngle;
    availableServos[index].minAngle = minAngle;
    availableServos[index].maxAngle = maxAngle;
    availableServos[index].name = name;
    
    uint16_t pulse = map(startAngle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel, 0, pulse);
    
    Serial.printf("✅ Servo %d -> Kanal %d: %d° (%s) [%d°-%d°]\n", 
                  index, channel, startAngle, name.c_str(), minAngle, maxAngle);
  }
}

// ==================== YARDIMCI FONKSİYONLAR ====================
uint16_t angleToPWM(uint16_t angle) {
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

void writePcaServo(uint8_t channel, uint16_t angle) {
  uint16_t pulse = angleToPWM(angle);
  pwm.setPWM(channel, 0, pulse);
}

void setupAllServos() {
  setupServo(0, 8, 100, 100, 150, "Kol_Servo_2");
  setupServo(1, 4, 72, 72, 122, "Kol_Servo_1");
  setupServo(2, 6, 126, 68, 126, "Bacak_Servo_1");
  setupServo(4, 10, 130, 130, 160, "Baş_Servo");
  setPcaServoAngleByChannel(9, 90);
  setupServo(5, 13, 92, 40, 92, "Kuyruk_Servo");
  setupServo(6, 11, 133, 105, 133, "Göz_Servo");
  setupServo(7, 12, 50, 50, 145, "Ağız_Servo");
}

void updatePcaServos() {
  unsigned long currentTime = millis();
  if (currentTime - lastPcaUpdate >= PCA_UPDATE_INTERVAL) {
    lastPcaUpdate = currentTime;
    
    for (uint8_t i = 0; i < TOTAL_CHANNELS; i++) {
      if (availableServos[i].currentAngle != availableServos[i].targetAngle) {
        if (availableServos[i].currentAngle < availableServos[i].targetAngle) {
          availableServos[i].currentAngle++;
        } else {
          availableServos[i].currentAngle--;
        }
        
        availableServos[i].currentAngle = constrain(availableServos[i].currentAngle,
                                                   availableServos[i].minAngle,
                                                   availableServos[i].maxAngle);
        writePcaServo(availableServos[i].channel, availableServos[i].currentAngle);
      }
    }
  }
}

void clearSerialBuffer() {
  bufferIndex = 0;
  serialBuffer[0] = '\0';
  commandReady = false;
  while (Serial.available() > 0) {
    Serial.read();
  }
}

bool setPcaServoAngleByChannel(uint8_t channel, int angle) {
  for (int i = 0; i < TOTAL_CHANNELS; i++) {
    if (availableServos[i].channel == channel) {
      if (angle < availableServos[i].minAngle) {
        Serial.printf("❌ HATA: Kanal %d için minimum açı %d°! Girilen: %d°\n", 
                      channel, availableServos[i].minAngle, angle);
        return false;
      }
      
      if (angle > availableServos[i].maxAngle) {
        Serial.printf("❌ HATA: Kanal %d için maksimum açı %d°! Girilen: %d°\n", 
                      channel, availableServos[i].maxAngle, angle);
        return false;
      }
      
      availableServos[i].targetAngle = angle;
      Serial.printf("✅ Kanal %d -> %d° (%s)\n", channel, angle, availableServos[i].name.c_str());
      return true;
    }
  }
  
  Serial.printf("❌ HATA: Kanal %d bulunamadı!\n", channel);
  return false;
}

// ==================== I2C Scanner ====================
void scanI2CDevices() {
  Serial.println("\n🔍 I2C Taraması Başlıyor...");
  byte error, address;
  int foundDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("✅ I2C Cihaz Bulundu: 0x%02X", address);
      
      if (address == 0x40) Serial.println(" -> PCA9685 Servo Sürücü");
      else if (address == 0x68) Serial.println(" -> MPU6050 Gyro");
      else if (address == 0x76) Serial.println(" -> BME280 Sensör");
      else if (address == 0x3C) Serial.println(" -> OLED Ekran");
      else if (address == 0x27) Serial.println(" -> LCD Ekran");
      else Serial.println(" -> Bilinmeyen Cihaz");
      
      foundDevices++;
    }
  }

  if (foundDevices == 0) {
    Serial.println("❌ Hiç I2C cihazı bulunamadı!");
  } else {
    Serial.printf("📊 Toplam %d I2C cihazı bulundu\n", foundDevices);
  }
}

// ==================== MOTOR KONTROL FONKSİYONLARI ====================
void setMotorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
  Serial.println("🚀 Motor İLERİ");
}

void setMotorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255);
  Serial.println("🔁 Motor GERİ");
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  Serial.println("🛑 Motor DUR");
}

void setMotorSpeed(int speed) {
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
    Serial.printf("🏎️ Motor İLERİ - Hız: %d (PID Kontrollü)\n", speed);
  } else if (speed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -speed);
    Serial.printf("🔁 Motor GERİ - Hız: %d (PID Kontrollü)\n", -speed);
  } else {
    stopMotor();
    Serial.println("🛑 Motor DUR");
  }
}

// ==================== YEREL SERVO KONTROLÜ ====================
void setLocalServo(int angle) {
  localServoAngle = constrain(angle, 0, 180);
  localServo.write(localServoAngle);
  Serial.printf("⚙️ Yerel Servo: %d°\n", localServoAngle);
}

void handleLocalServoPress(int angle) {
  setLocalServo(angle);
  Serial.printf("🔘 Yerel Servo Basılı: %d°\n", angle);
}

void handleLocalServoRelease() {
  setLocalServo(LOCAL_SERVO_HOME_ANGLE);
  Serial.printf("🔘 Yerel Servo Bırakıldı: %d° (Home)\n", LOCAL_SERVO_HOME_ANGLE);
}

// ==================== ULTRASONİK SENSÖR FONKSİYONLARI ====================
void setupUltrasonicSensors() {
  for (int i = 0; i < 4; i++) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
  }
  Serial.println("✅ 4 Ultrasonik Sensör Başlatıldı");
} 

float readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); 
  
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) {
    return MAX_DISTANCE;
  }
  
  float distanceCM = (duration * SoundSpeed) / 2;
  
  // Basit medyan filtre
  static float last_distance = MAX_DISTANCE;
  if (distanceCM > MAX_DISTANCE || distanceCM < 2) {
    distanceCM = last_distance;
  } else {
    last_distance = distanceCM;
  }
  
  return constrain(distanceCM, 2, MAX_DISTANCE);
}

// ==================== RESET FONKSİYONLARI ====================
void resetAllServos() {
  Serial.println("\n🔄 TÜM SİSTEM RESETLENİYOR...");
  setAutonomousMode(false);
  stopMotor();
  setLocalServo(LOCAL_SERVO_HOME_ANGLE);
  Serial.printf("✅ Yerel servo home pozisyonuna getirildi: %d°\n", LOCAL_SERVO_HOME_ANGLE);
  resetOnlyServos();
  resetPidParameters();
  Serial.println("🎉 TÜM SİSTEM RESET TAMAMLANDI!\n");
}

void resetOnlyServos() {
  Serial.println("⚙️ Tüm servolar reset pozisyonlarına getiriliyor...");
  
  setPcaServoAngleByChannel(8, 100);
  delay(100);
  setPcaServoAngleByChannel(4, 72);
  delay(100);
  setPcaServoAngleByChannel(6, 126);
  delay(100);
  setPcaServoAngleByChannel(9, 90);
  delay(100);
  setPcaServoAngleByChannel(10, 130);
  delay(100);
  setPcaServoAngleByChannel(13, 92);
  delay(100);
  setPcaServoAngleByChannel(11, 133);
  delay(100);
  setPcaServoAngleByChannel(12, 50);
  
  Serial.printf("✅ %d servo reset pozisyonlarına getirildi\n", 8);
}

// ==================== OTOMATİK RESET KONTROLÜ ====================
void autoResetIfNoActivity() {
  unsigned long currentTime = millis();
  
  if (!resetExecuted && (currentTime - lastSerialActivity >= RESET_TIMEOUT)) {
    Serial.println("⏰ Seri port zaman aşımı - Otomatik resetleniyor...");
    resetAllServos();
    resetExecuted = true;
    lastSerialActivity = currentTime;
  }
}

// ==================== SERİ HABERLEŞME FONKSİYONLARI ====================
void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    lastSerialActivity = millis();
    resetExecuted = false;
    
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) {
        serialBuffer[bufferIndex] = '\0';
        commandReady = true;
        return;
      }
    } 
    else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[bufferIndex++] = c;
    } 
    else {
      clearSerialBuffer();
      return;
    }
  }
}

void processSerialCommand() {
  if (commandReady) {
    Serial.printf("🔧 Seri Komut: %s\n", serialBuffer);
    
    String command = String(serialBuffer);
    command.toLowerCase();
    
    // PID Debug komutları
    if (command == "pid debug" || command == "piddebug") {
      pidDebug = !pidDebug;
      Serial.printf("🔧 PID Debug: %s\n", pidDebug ? "AKTİF" : "PASİF");
    }
    else if (command == "pid status" || command == "pidstatus") {
      printPidDebug();
    }
    else if (command == "wall follow" || command == "duvar") {
      setWallFollowingMode(true);
    }
    else if (command == "otonom" || command == "autonomous" || command == "auto") {
      setAutonomousMode(true);
    }
    else if (command == "manuel" || command == "manual" || command == "stop auto") {
      setAutonomousMode(false);
    }
    else if (command == "sensor" || command == "sensors" || command == "mesafe") {
      updateAllSensors();
      Serial.println("\n📊 SENSÖR MESAFELERİ (PID Filtreli):");
      for (int i = 0; i < 4; i++) {
        Serial.printf("  %s: %.1fcm (Filtre: %.1fcm)\n", 
                     sensors[i].name.c_str(), 
                     sensors[i].distance,
                     sensors[i].filtered_distance);
      }
    }
    else if (command == "reset" || command == "sifirla" || command == "sıfırla") {
      resetAllServos();
    }
    else if (command == "reset servo" || command == "servo reset") {
      resetOnlyServos();
    }
    else if (command == "motor ileri" || command == "ileri" || command == "forward") {
      setAutonomousMode(false);
      setMotorForward();
    }
    else if (command == "motor geri" || command == "geri" || command == "backward") {
      setAutonomousMode(false);
      setMotorBackward();
    }
    else if (command == "motor dur" || command == "dur" || command == "stop") {
      stopMotor();
    }
    else if (command == "help" || command == "yardim" || command == "yardım") {
      printHelp();
    }
    else if (command == "status" || command == "durum") {
      printServoStatus();
    }
    else if (command == "scan" || command == "tarama") {
      scanI2CDevices();
    }
    else if (command == "test") {
      runServoTest();
    }
    else if (command == "clear" || command == "temizle") {
      clearSerialBuffer();
      Serial.println("🧹 Buffer temizlendi!");
    }
    else {
      Serial.println("❌ Bilinmeyen komut! 'help' yazarak kullanılabilir komutları görün.");
    }
    
    clearSerialBuffer();
  }
}

void printPidDebug() {
  Serial.println("\n🎯 PID DURUM RAPORU:");
  Serial.println("====================");
  
  for (int i = 0; i < 4; i++) {
    Serial.printf("📡 %s Sensör:\n", sensors[i].name.c_str());
    Serial.printf("   Setpoint: %.1fcm\n", sensors[i].pid.setpoint);
    Serial.printf("   Kp: %.3f, Ki: %.3f, Kd: %.3f\n", 
                  sensors[i].pid.Kp, sensors[i].pid.Ki, sensors[i].pid.Kd);
    Serial.printf("   Integral: %.3f, Previous Error: %.3f\n",
                  sensors[i].pid.integral, sensors[i].pid.previous_error);
    Serial.printf("   Output: %.3f\n", sensors[i].pid.output);
  }
  
  Serial.printf("\n🏎️ Motor PID:\n");
  Serial.printf("   Setpoint: %.1fcm\n", motorPID.setpoint);
  Serial.printf("   Kp: %.3f, Ki: %.3f, Kd: %.3f\n", 
                motorPID.Kp, motorPID.Ki, motorPID.Kd);
  Serial.printf("   Output Range: [%.0f, %.0f]\n", motorPID.output_min, motorPID.output_max);
}

void printHelp() {
  Serial.println("\n🎯 SERİ KONTROL SİSTEMİ - OTONOM MOD DESTEKLİ");
  Serial.println("==============================================");
  Serial.println("🚀 OTONOM KOMUTLARI:");
  Serial.println("  otonom/auto      - Otonom modu başlat");
  Serial.println("  manuel/manual    - Manuel moda geç");
  Serial.println("  wall follow      - Duvar takip modu");
  Serial.println("🎯 PID KOMUTLARI:");
  Serial.println("  pid debug        - PID debug modunu aç/kapat");
  Serial.println("  pid status       - PID parametrelerini göster");
  Serial.println("📡 SENSÖR KOMUTLARI:");
  Serial.println("  sensor/mesafe    - Sensör verilerini göster");
  Serial.println("🏎️ MOTOR KOMUTLARI:");
  Serial.println("  motor ileri      - Motoru ileri sür");
  Serial.println("  motor geri       - Motoru geri sür");
  Serial.println("  motor dur        - Motoru durdur");
  Serial.println("🔧 SİSTEM KOMUTLARI:");
  Serial.println("  reset            - Tüm sistemi resetle");
  Serial.println("  reset servo      - Sadece servoları resetle");
  Serial.println("  status           - Sistem durumunu göster");
  Serial.println("  scan             - I2C cihazlarını tara");
  Serial.println("  help             - Bu yardım mesajını göster");
  Serial.println("🌐 WEB KONTROL: http://" + WiFi.softAPIP().toString());
  Serial.println("   PID Ayarları: /pidsettings");
}

void printServoStatus() {
  Serial.printf("📊 Yerel Servo: %d° (Home: %d°)\n", localServoAngle, LOCAL_SERVO_HOME_ANGLE);
  Serial.printf("📡 Otonom Mod: %s\n", autonomousMode ? "AKTİF" : "PASİF");
  Serial.printf("🎯 Duvar Takip: %s\n", wallFollowingMode ? "AKTİF" : "PASİF");
  
  updateAllSensors();
  Serial.println("📊 Sensör Mesafeleri (PID Filtreli):");
  for (int i = 0; i < 4; i++) {
    Serial.printf("  %s: %.1f cm (Filtre: %.1f cm)\n", sensors[i].name.c_str(), sensors[i].distance, sensors[i].filtered_distance);
  }
}

void runServoTest() {
  Serial.println("🧪 Servo Test Modu...");
  setAutonomousMode(false);
  
  handleLocalServoPress(0);
  delay(1000);
  handleLocalServoRelease();
  delay(500);
  handleLocalServoPress(180);
  delay(1000);
  handleLocalServoRelease();
  delay(500);
  handleLocalServoPress(90);
  delay(1000);
  handleLocalServoRelease();
}

void printResetPositions() {
  Serial.println("\n🔄 Servo Reset Pozisyonları:");
  if (!Serial) {
    resetOnlyServos();
  }
}

// ==================== SETUP ve LOOP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n🤖 PID KONTROLLÜ OTONOM ROBOT SİSTEMİ");
  Serial.println("=====================================");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
  
  setupUltrasonicSensors();
  stopMotor();

  localServo.attach(LOCAL_SERVO_PIN);
  setLocalServo(LOCAL_SERVO_HOME_ANGLE);

  Wire.begin();
  scanI2CDevices();
  
  bool pcaStarted = pwm.begin();
  if (pcaStarted) {
    Serial.println("✅ PCA9685 başarıyla başlatıldı");
    pwm.setPWMFreq(60);
    setupAllServos();
  } else {
    Serial.println("❌ PCA9685 başlatılamadı!");
  }

  // PID Kontrolcülerini Başlat
  initializePIDs();

  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("📡 AP IP: ");
  Serial.println(IP);

  server.begin();
  
  lastSerialActivity = millis();
  
  delay(1000);
  Serial.println("\n🎉 SİSTEM HAZIR!");
  Serial.println("🌐 Web arayüzü: http://" + IP.toString());
  Serial.println("🔧 PID Ayarları: http://" + IP.toString() + "/pidsettings");
  Serial.println("💡 Seri porttan 'help' yazarak tüm komutları görebilirsiniz");
  Serial.println("⏰ 3 saniye seri port aktivitesi olmazsa otomatik reset\n");
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    handleWebClient(client);
    client.stop();
  }

  checkSerial();
  processSerialCommand();
  updatePcaServos();
  autoResetIfNoActivity();
  
  // Otonom mod aktifse hareketi kontrol et
  if (autonomousMode) {
    autonomousMovement();
  }
  
  // Sensörleri düzenli oku
  unsigned long currentTime = millis();
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    updateAllSensors();
    lastSensorRead = currentTime;
  }
  
  delay(10);
}