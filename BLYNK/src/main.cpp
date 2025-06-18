// Cấu hình Blynk 
#define BLYNK_TEMPLATE_ID "TMPL68X01awgQ"
#define BLYNK_TEMPLATE_NAME "warning"
#define BLYNK_AUTH_TOKEN "_tt7N8T4LKbdd6HOmkOYpzJV1uC2PExQ"

#include <Arduino.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <TimeLib.h>

#include <SPI.h>
#include <SD.h>

#include <string>

// Chân kết nối
#define LED_PIN 2    
#define RX_PIN 16 // chân RX2 
#define TX_PIN 17 // chân TX2

#define SD_CS 5  // Chân CS 
// Chân Miso    19
// Chân Mosi    23
// Chân SCK     18

File myFile;

#define BAUDRATE 9600

// WiFi
char ssid[] = "xieop"; // Thay tên mạng vào
char pass[] = "pppppppp"; // Thay cả pass luôn nhá

bool wifiConnected = false; // Này là trạng thái wifi

void sendAlert(String);
void checkWiFiConnection();
void blinkLED(int times, int duration);
String getCurrentTime();
void writeToFile(const char* filename, const char* content);
void readFromFile(const char* filename);

void setup() {
  Serial.begin(BAUDRATE);
  Serial2.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN); 

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Kết nối WiFi với timeout
  Serial.println("\nDang ket noi wifi...");
  WiFi.begin(ssid, pass);
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    blinkLED(1, 200); 
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nKet noi wifi thanh cong");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_PIN, HIGH); 
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    
    // Kết nối Blynk
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(3000); // Timeout 3 giây
    if (Blynk.connected()) {
      Serial.println("Ket noi blynk thanh cong");
    } else {
      Serial.println("Loi ket noi thanh cong");
    }
  } else {
    Serial.println("\nLoi ket noi wifi");
    blinkLED(5, 200); 
  }
  Blynk.sendInternal("rtc", "sync"); // Đồng bộ thời gian

  Serial.print("Dang khoi tao the SD");

  if (!SD.begin(SD_CS)) {
    Serial.println("Khong the khoi tao the SD");
    return;
  }
  Serial.println("Khoi tao SD thanh cong");

}

void loop() {
  if (wifiConnected) {
    Blynk.run();
  }
  
  checkWiFiConnection(); 
  
  // Nhận dữ liệu từ STM32
  if (Serial2.available() > 0) {
    String receivedData = Serial2.readStringUntil('\n');
    receivedData.trim();

    String message_warning = getCurrentTime() + " " + receivedData;
    writeToFile("/test.txt", message_warning.c_str());
    if (wifiConnected && Blynk.connected()) {
      sendAlert(message_warning);
    } else {
      Serial.println("Khong gui duoc canh bao - Mat ket noi");
    }
  
    Serial.print("Chuoi nhan tu STM32: '");
    Serial.print(receivedData);
    Serial.println("'");

    receivedData = "";
    // readFromFile("/test.txt");
  }

}

void sendAlert(String message_warning) {
  Serial.println("Da gui canh bao");
  digitalWrite(LED_PIN, HIGH);
  
  Blynk.logEvent("alert", message_warning);  
  Blynk.virtualWrite(V0, message_warning);
  
  
  digitalWrite(LED_PIN, LOW);
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) { // Kiểm tra wifi sau 5s
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      Serial.println("Mat ket noi wifi");
      blinkLED(3, 200);
    } else if (!Blynk.connected()) {
      Serial.println("Mat ket noi Blynk!");
      Blynk.connect();
    }
  }
}

void blinkLED(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(duration);
  }
}

// Lấy thời gian hiện tại
String getCurrentTime() {
  time_t localTime = now();

  char timeStr[9]; 
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", hour(localTime), minute(localTime), second(localTime));
  return String(timeStr);
}

// Hàm ghi nội dung vào file
void writeToFile(const char* filename, const char* content) {
  myFile = SD.open(filename, FILE_APPEND);
  if (myFile) {
    Serial.print("Ghi vào ");
    Serial.print(filename);
    Serial.println("...");
    myFile.println(content);
    myFile.close();
    Serial.println("Đã ghi xong.");
  } else {
    Serial.print("Không thể mở file ");
    Serial.print(filename);
    Serial.println(" để ghi.");
  }
  delay(5000);
}

// Hàm đọc nội dung từ file
void readFromFile(const char* filename) {
  myFile = SD.open(filename);
  if (myFile) {
    Serial.print("Nội dung trong ");
    Serial.print(filename);
    Serial.println(":");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
    Serial.println(); // Xuống dòng sau khi đọc xong
  } else {
    Serial.print("Không thể mở file ");
    Serial.print(filename);
    Serial.println(" để đọc.");
  }
}

BLYNK_WRITE(InternalPinRTC) {
  long unixTime = param.asLong();
  Serial.print("Unix time received: ");
  Serial.println(unixTime);
  
  if (unixTime > 0) {
    setTime(unixTime); // Thiết lập thời gian hệ thống
    
    // In thời gian dạng HH:MM:SS DD/MM/YYYY
    Serial.print("Time: ");
    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());
    Serial.print(":");
    Serial.print(second());
    Serial.print(" ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.println(year());
  }
}