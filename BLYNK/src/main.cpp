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

#define SD_CS 5  // Chân CS kết nối với SD Card

File myFile;

#define BAUDRATE 9600
#define TIME_ZONE_OFFSET 7 * 3600 // GMT+7

// WiFi
char ssid[] = "xieop"; // Thay tên mạng vào
char pass[] = "pppppppp"; // Thay cả pass luôn nhá

bool wifiConnected = false; // Này là trạng thái wifi
unsigned long lastSyncTime = 0;

void sendAlert(String);
String getTimeStamp();
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
  Serial.println("\nĐang kết nối WiFi...");
  WiFi.begin(ssid, pass);
  
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    blinkLED(1, 200); // Nhấp nháy LED khi đang kết nối
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println("\nKết nối WiFi thành công!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    digitalWrite(LED_PIN, HIGH); // Sáng đèn khi kết nối thành công
    delay(1000);
    digitalWrite(LED_PIN, LOW);
    
    // Kết nối Blynk
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(3000); // Timeout 3 giây
    if (Blynk.connected()) {
      Serial.println("Kết nối Blynk thành công!");
    } else {
      Serial.println("Lỗi kết nối Blynk!");
    }
  } else {
    Serial.println("\nLỗi kết nối WiFi!");
    blinkLED(5, 200); // Báo lỗi bằng LED
  }
  Blynk.sendInternal("rtc", "sync"); // Yêu cầu đồng bộ thời gian

  Serial.print("Đang khởi tạo thẻ SD...");

  if (!SD.begin(SD_CS)) {
    Serial.println("Lỗi: không thể khởi tạo thẻ SD!");
    return;
  }
  Serial.println("Đã khởi tạo thẻ SD thành công.");

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
      Serial.println("Không thể gửi cảnh báo - Mất kết nối!");
    }
  
    Serial.print("Received from STM32: '");
    Serial.print(receivedData);
    Serial.println("'");

    receivedData = ""; // Reset biến nhận dữ liệu
    readFromFile("/test.txt");
  }

}

void sendAlert(String message_warning) {
  Serial.println("Da gui canh bao");
  digitalWrite(LED_PIN, HIGH);
  
  Blynk.logEvent("alert", message_warning);  
  Blynk.virtualWrite(V0, message_warning);
  
  
  digitalWrite(LED_PIN, LOW);
}

String getTimeStamp() {
  return String(millis() / 1000) + "s";
}

void checkWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 5000) { // Kiểm tra mỗi 5s
    lastCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      wifiConnected = false;
      Serial.println("Mất kết nối WiFi!");
      blinkLED(3, 200);
    } else if (!Blynk.connected()) {
      Serial.println("Mất kết nối Blynk!");
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

// Hàm lấy thời gian hiện tại (định dạng HH:MM:SS)
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