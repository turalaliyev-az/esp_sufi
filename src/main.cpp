 #include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== WiFi Ayarları ====================
const char* ssid = "Robot";
const char* password = "12345678";

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
void setLocalServo(int angle);
void printResetPositions();
void setupServo(uint8_t index, uint8_t channel, uint16_t startAngle, uint16_t minAngle, uint16_t maxAngle, String name = "");
void runServoTest();
// ==================== Servo Kurulum Fonksiyonu ====================
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

// ==================== Yardımcı Fonksiyonlar ====================
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
  setupServo(3, 9, 50, 50, 120, "Boyun_Servo_2");
  setupServo(4, 10, 130, 130, 160, "Baş_Servo");
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
  Serial.println("🚀 Motor İLERİ");
}

void setMotorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  Serial.println("🔁 Motor GERİ");
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  Serial.println("🛑 Motor DUR");
}

// ==================== YEREL SERVO KONTROLÜ ====================
void setLocalServo(int angle) {
  localServoAngle = constrain(angle, 0, 180);
  localServo.write(localServoAngle);
  Serial.printf("⚙️ Yerel Servo: %d°\n", localServoAngle);
}

// ==================== RESET FONKSİYONLARI ====================
void resetAllServos() {
  Serial.println("\n🔄 TÜM SİSTEM RESETLENİYOR...");
  stopMotor();
  setLocalServo(LOCAL_SERVO_HOME_ANGLE);
  Serial.printf("✅ Yerel servo home pozisyonuna getirildi: %d°\n", LOCAL_SERVO_HOME_ANGLE);
  resetOnlyServos();
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

// ==================== WEB SERVO KONTROL FONKSİYONLARI ====================
void handleLocalServoPress(int angle) {
  setLocalServo(angle);
  Serial.printf("🔘 Yerel Servo Basılı: %d°\n", angle);
}

void handleLocalServoRelease() {
  setLocalServo(LOCAL_SERVO_HOME_ANGLE);
  Serial.printf("🔘 Yerel Servo Bırakıldı: %d° (Home)\n", LOCAL_SERVO_HOME_ANGLE);
}

// ==================== GELİŞTİRİLMİŞ KOMUT İŞLEME ====================
void processFlexibleCommand(const char* command) {
  Serial.printf("🔧 İşleniyor: %s\n", command);
  
  char buffer[512];
  size_t commandLen = strlen(command);
  if (commandLen >= sizeof(buffer)) {
    Serial.println("❌ Komut çok uzun!");
    return;
  }
  strcpy(buffer, command);
  
  // Tüm komutları boşluklardan böl ve teker teker işle
  char *token = strtok(buffer, " ");
  bool commandProcessed = false;
  
  while (token != NULL) {
    String cmdStr = String(token);
    cmdStr.toLowerCase();
    cmdStr.trim();
    
    Serial.printf("🔍 Komut parçası: %s\n", cmdStr.c_str());
    
    // RESET komutları
    if (cmdStr == "reset" || cmdStr == "sifirla" || cmdStr == "sıfırla") {
      resetAllServos();
      commandProcessed = true;
    }
    else if (cmdStr == "reset servo" || cmdStr == "servo reset") {
      resetOnlyServos();
      commandProcessed = true;
    }
    
    // Local servo komutları - "local:120" formatı
    else if (cmdStr.startsWith("local:")) {
      int angle = cmdStr.substring(6).toInt();
      if (angle >= 0 && angle <= 180) {
        setLocalServo(angle);
        commandProcessed = true;
      } else {
        Serial.println("❌ HATA: Local servo için açı 0-180 arası olmalı!");
      }
    }
    
    // Motor komutları
    else if (cmdStr == "motor ileri" || cmdStr == "ileri" || cmdStr == "forward") {
      setMotorForward();
      commandProcessed = true;
    }
    else if (cmdStr == "motor geri" || cmdStr == "geri" || cmdStr == "backward") {
      setMotorBackward();
      commandProcessed = true;
    }
    else if (cmdStr == "motor dur" || cmdStr == "dur" || cmdStr == "stop") {
      stopMotor();
      commandProcessed = true;
    }
    
    // Yerel servo komutları
    else if (cmdStr == "servo sol" || cmdStr == "sol" || cmdStr == "left") {
      setLocalServo(0);
      commandProcessed = true;
    }
    else if (cmdStr == "servo sag" || cmdStr == "sag" || cmdStr == "right") {
      setLocalServo(180);
      commandProcessed = true;
    }
    else if (cmdStr == "servo orta" || cmdStr == "orta" || cmdStr == "center") {
      setLocalServo(90);
      commandProcessed = true;
    }
    
    // Sayısal servo komutu (servo 90)
    else if (cmdStr.startsWith("servo ")) {
      int angle = cmdStr.substring(6).toInt();
      if (angle >= 0 && angle <= 180) {
        setLocalServo(angle);
        commandProcessed = true;
      }
    }
    
    // Sistem komutları
    else if (cmdStr == "help" || cmdStr == "yardim" || cmdStr == "yardım") {
      printHelp();
      commandProcessed = true;
    }
    else if (cmdStr == "status" || cmdStr == "durum") {
      printServoStatus();
      commandProcessed = true;
    }
    else if (cmdStr == "scan" || cmdStr == "tarama") {
      scanI2CDevices();
      commandProcessed = true;
    }
    else if (cmdStr == "test") {
      runServoTest();
      commandProcessed = true;
    }
    else if (cmdStr == "clear" || cmdStr == "temizle") {
      clearSerialBuffer();
      Serial.println("🧹 Buffer temizlendi!");
      commandProcessed = true;
    }
    
    // PCA9685 servo komutları - "kanal,açı" formatı
    else {
      int channel, angle;
      if (sscanf(token, "%d,%d", &channel, &angle) == 2) {
        if (setPcaServoAngleByChannel(channel, angle)) {
          commandProcessed = true;
        }
      }
    }
    
    token = strtok(NULL, " ");
  }
  
  if (!commandProcessed) {
    Serial.println("❌ Bilinmeyen komut! 'help' yazarak kullanılabilir komutları görün.");
  }
}

void checkSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
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
    processFlexibleCommand(serialBuffer);
    clearSerialBuffer();
  }
}

// ==================== WEB SERVER İŞLEME ====================
void handleWebClient(WiFiClient &client) {
  String request = client.readStringUntil('\r');
  Serial.println("🌐 Web İsteği: " + request);

  if (request.indexOf("GET /servo?press=") != -1) {
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
  else if (request.indexOf("GET /motor") != -1) {
    if (request.indexOf("ileri") != -1) setMotorForward();
    else if (request.indexOf("geri") != -1) setMotorBackward();
    else if (request.indexOf("dur") != -1) stopMotor();
  }
  else if (request.indexOf("GET /reset") != -1) {
    resetAllServos();
  }
  else if (request.indexOf("GET /resetservo") != -1) {
    resetOnlyServos();
  }

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
  client.println(".info { background: #607D8B; color: white; padding: 15px; margin: 15px; border-radius: 10px; }");
  client.println(".servo-container { background: white; padding: 20px; margin: 15px; border-radius: 15px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); }");
  client.println(".servo-status { font-size: 18px; margin: 10px; padding: 10px; background: #E3F2FD; border-radius: 8px; }");
  client.println("</style>");
  client.println("<script>");
  client.println("function servoPress(direction) {");
  client.println("  fetch('/servo?press=' + direction);");
  client.println("  updateServoStatus(direction + ' (Basılı)');");
  client.println("}");
  client.println("function servoRelease() {");
  client.println("  fetch('/servo?release');");
  client.println("  updateServoStatus('Orta (Serbest)');");
  client.println("}");
  client.println("function updateServoStatus(status) {");
  client.println("  document.getElementById('servoStatus').innerText = 'Durum: ' + status;");
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
  client.println("});");
  client.println("</script>");
  client.println("</head><body>");
  client.println("<h1>🤖 Robot Kontrol Paneli</h1>");

  // Yerel servo kontrol bölümü
  client.println("<div class='servo-container'>");
  client.println("<h2>🎮 Yerel Servo Kontrol (Basılı Tut)</h2>");
  client.println("<div class='servo-status' id='servoStatus'>Durum: Orta (Serbest)</div>");
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
  client.println("<div class='servo-container'>");
  client.println("<h2>🏎️ Motor Kontrol</h2>");
  client.println("<button class='btn motor' ontouchstart=\"fetch('/motor?ileri')\" ontouchend=\"fetch('/motor?dur')\" onmousedown=\"fetch('/motor?ileri')\" onmouseup=\"fetch('/motor?dur')\">İLERİ</button><br>");
  client.println("<button class='btn motor' ontouchstart=\"fetch('/motor?geri')\" ontouchend=\"fetch('/motor?dur')\" onmousedown=\"fetch('/motor?geri')\" onmouseup=\"fetch('/motor?dur')\">GERİ</button>");
  client.println("</div>");

  // Reset bölümü
  client.println("<div class='servo-container'>");
  client.println("<h2>🔄 Sistem Kontrol</h2>");
  client.println("<button class='btn reset' onclick=\"fetch('/reset')\">TÜM SİSTEMİ RESETLE</button><br>");
  client.println("<button class='btn reset' onclick=\"fetch('/resetservo')\">SADECE SERVOLARI RESETLE</button>");
  client.println("</div>");

  // Bilgi bölümü
  client.println("<div class='info'>");
  client.println("<strong>🎯 KULLANIM KILAVUZU</strong><br>");
  client.println("• Yerel Servo: Butonlara basılı tutun → servo hareket eder<br>");
  client.println("• Motor: Basılı tutun → motor çalışır, bırakın → motor durur<br>");
  client.println("• Reset: Tüm sistem veya sadece servoları sıfırlar");
  client.println("</div>");

  client.println("</body></html>");
}

// ==================== DİĞER FONKSİYONLAR ====================
void printHelp() {
  Serial.println("\n🎯 SERİ KONTROL SİSTEMİ");
  Serial.println("Web arayüzünde butonlara basılı tutunca servo hareket eder!");
  Serial.println("Butonları bırakınca servo orta pozisyona (90°) döner.\n");
}

void printServoStatus() {
  Serial.printf("📊 Yerel Servo: %d° (Home: %d°)\n", localServoAngle, LOCAL_SERVO_HOME_ANGLE);
}

void runServoTest() {
  Serial.println("🧪 Servo Test Modu...");
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

// ==================== SETUP ve LOOP ====================
void setup() {
  Serial.begin(115200);
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);
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
  }

  WiFi.softAP(ssid, password);
  Serial.print("📡 AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  
  delay(1000);
  Serial.println("🤖 ÇOKLU KOMUT SİSTEMİ HAZIR!");
  Serial.println("💡 Örnek: 'local:100 forward' → Servo 100° ve motor ileri");
  Serial.println("💡 Örnek: '4,90 6,110 forward' → 2 servo + motor ileri\n");
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
}