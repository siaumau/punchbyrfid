#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ===== WiFi 設定 =====
const char* ssid = "RFID-Punch-System";  // AP 名稱
const char* password = "12345689";       // AP 密碼 (至少8位)

// ===== Web 服務器 =====
ESP8266WebServer server(80);

// ===== HW-364A OLED (硬體焊接，不能改) =====
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,
    /* reset=*/ U8X8_PIN_NONE,
    /* clock=*/ 12,  // GPIO12 = D6
    /* data=*/  14); // GPIO14 = D5

// ===== HW-364A 按鈕和蜂鳴器 =====
#define BUTTON_PIN 13  // GPIO13 = D7
#define BUZZER_PIN 2   // GPIO2 = D4

// ===== RFID 腳位定義 =====
#define RST_PIN  5     // D1
#define SS_PIN   15    // D8
#define MOSI_PIN 0     // D3 (GPIO0)
#define MISO_PIN 16    // D0 (GPIO16)
#define SCK_PIN  4     // D2 (GPIO4)

// ===== MFRC522 暫存器定義 =====
#define CommandReg      0x01
#define ComIEnReg       0x02
#define DivIEnReg       0x03
#define ComIrqReg       0x04
#define DivIrqReg       0x05
#define ErrorReg        0x06
#define Status1Reg      0x07
#define Status2Reg      0x08
#define FIFODataReg     0x09
#define FIFOLevelReg    0x0A
#define BitFramingReg   0x0D
#define CollReg         0x0E
#define ModeReg         0x11
#define TxControlReg    0x14
#define TxASKReg        0x15
#define VersionReg      0x37

// ===== MFRC522 命令 =====
#define PCD_Idle        0x00
#define PCD_Mem         0x01
#define PCD_CalcCRC     0x03
#define PCD_Transmit    0x04
#define PCD_Receive     0x08
#define PCD_Transceive  0x0C
#define PCD_MFAuthent   0x0E
#define PCD_SoftReset   0x0F

// ===== PICC 命令 =====
#define PICC_REQIDL     0x26
#define PICC_REQALL     0x52
#define PICC_ANTICOLL   0x93
#define PICC_SELECTTAG  0x93
#define PICC_AUTHENT1A  0x60
#define PICC_AUTHENT1B  0x61
#define PICC_READ       0x30
#define PICC_WRITE      0xA0
#define PICC_HALT       0x50

String lastUID = "";
bool writeMode = false;
byte uid[5] = {0};
byte uidSize = 0;

// ===== 軟體 SPI 函式 =====
void spiBegin() {
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(SCK_PIN, OUTPUT);
  pinMode(SS_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  
  digitalWrite(SS_PIN, HIGH);
  digitalWrite(SCK_PIN, LOW);
  digitalWrite(RST_PIN, HIGH);
}

byte spiTransfer(byte data) {
  byte result = 0;
  
  for (int i = 7; i >= 0; i--) {
    digitalWrite(MOSI_PIN, (data >> i) & 0x01);
    delayMicroseconds(2);
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(2);
    
    if (digitalRead(MISO_PIN)) {
      result |= (1 << i);
    }
    
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2);
  }
  
  return result;
}

// ===== RC522 基本函式 =====
void writeRegister(byte reg, byte value) {
  digitalWrite(SS_PIN, LOW);
  spiTransfer((reg << 1) & 0x7E);
  spiTransfer(value);
  digitalWrite(SS_PIN, HIGH);
}

byte readRegister(byte reg) {
  digitalWrite(SS_PIN, LOW);
  spiTransfer(((reg << 1) & 0x7E) | 0x80);
  byte value = spiTransfer(0x00);
  digitalWrite(SS_PIN, HIGH);
  return value;
}

void setBitMask(byte reg, byte mask) {
  byte tmp = readRegister(reg);
  writeRegister(reg, tmp | mask);
}

void clearBitMask(byte reg, byte mask) {
  byte tmp = readRegister(reg);
  writeRegister(reg, tmp & (~mask));
}

void antennaOn() {
  byte temp = readRegister(TxControlReg);
  if (!(temp & 0x03)) {
    setBitMask(TxControlReg, 0x03);
  }
}

void reset() {
  writeRegister(CommandReg, PCD_SoftReset);
  delay(50);
  
  while (readRegister(CommandReg) & 0x10);
  
  writeRegister(0x2A, 0x8D);
  writeRegister(0x2B, 0x3E);
  writeRegister(0x2D, 0x1E);
  writeRegister(0x2C, 0x00);
  writeRegister(0x15, 0x40);
  writeRegister(ModeReg, 0x3D);
  
  antennaOn();
}

byte communicate(byte command, byte *sendData, byte sendLen, byte *backData, byte *backLen) {
  byte irqEn = 0x00;
  byte waitIRq = 0x00;
  
  switch (command) {
    case PCD_MFAuthent:
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    case PCD_Transceive:
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    default:
      break;
  }
  
  writeRegister(ComIEnReg, irqEn | 0x80);
  clearBitMask(ComIrqReg, 0x80);
  setBitMask(FIFOLevelReg, 0x80);
  writeRegister(CommandReg, PCD_Idle);
  
  for (byte i = 0; i < sendLen; i++) {
    writeRegister(FIFODataReg, sendData[i]);
  }
  
  writeRegister(CommandReg, command);
  
  if (command == PCD_Transceive) {
    setBitMask(BitFramingReg, 0x80);
  }
  
  uint16_t i = 2000;
  byte n;
  do {
    n = readRegister(ComIrqReg);
    i--;
  } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));
  
  clearBitMask(BitFramingReg, 0x80);
  
  if (i == 0) {
    return 1; // Timeout
  }
  
  byte error = readRegister(ErrorReg);
  if (error & 0x1B) {
    return 2; // Error
  }
  
  if (backData && backLen) {
    n = readRegister(FIFOLevelReg);
    *backLen = n;
    for (byte i = 0; i < n; i++) {
      backData[i] = readRegister(FIFODataReg);
    }
  }
  
  return 0;
}

byte request(byte reqMode, byte *tagType) {
  writeRegister(BitFramingReg, 0x07);
  
  byte buff[1] = {reqMode};
  byte backLen;
  byte status = communicate(PCD_Transceive, buff, 1, tagType, &backLen);
  
  if (status != 0 || backLen != 2) {
    return 1;
  }
  
  return 0;
}

byte antiCollision(byte *serNum) {
  writeRegister(BitFramingReg, 0x00);
  
  byte buff[2] = {PICC_ANTICOLL, 0x20};
  byte backLen;
  byte status = communicate(PCD_Transceive, buff, 2, serNum, &backLen);
  
  if (status == 0 && backLen == 5) {
    return 0;
  }
  
  return 1;
}

byte authenticate(byte authMode, byte blockAddr, byte *sectorKey, byte *serNum) {
  byte buff[12];
  
  buff[0] = authMode;
  buff[1] = blockAddr;
  for (byte i = 0; i < 6; i++) {
    buff[i + 2] = sectorKey[i];
  }
  for (byte i = 0; i < 4; i++) {
    buff[i + 8] = serNum[i];
  }
  
  byte status = communicate(PCD_MFAuthent, buff, 12, NULL, NULL);
  
  if (status != 0 || !(readRegister(Status2Reg) & 0x08)) {
    return 1;
  }
  
  return 0;
}

byte readBlock(byte blockAddr, byte *recvData) {
  byte buff[4] = {PICC_READ, blockAddr, 0, 0};
  byte backLen;
  
  byte status = communicate(PCD_Transceive, buff, 2, recvData, &backLen);
  
  if (status != 0 || backLen != 16) {
    return 1;
  }
  
  return 0;
}

byte writeBlock(byte blockAddr, byte *writeData) {
  byte buff[4] = {PICC_WRITE, blockAddr, 0, 0};
  byte backLen;
  byte recvData[16];
  
  byte status = communicate(PCD_Transceive, buff, 2, recvData, &backLen);
  
  if (status != 0 || backLen != 1 || (recvData[0] & 0x0F) != 0x0A) {
    return 1;
  }
  
  byte sendBuff[18];
  for (byte i = 0; i < 16; i++) {
    sendBuff[i] = writeData[i];
  }
  
  status = communicate(PCD_Transceive, sendBuff, 16, recvData, &backLen);
  
  if (status != 0 || backLen != 1 || (recvData[0] & 0x0F) != 0x0A) {
    return 1;
  }
  
  return 0;
}

void halt() {
  byte buff[4] = {PICC_HALT, 0, 0, 0};
  communicate(PCD_Transceive, buff, 2, NULL, NULL);
}

// ===== 蜂鳴器 =====
void beep(int duration) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepSuccess() {
  beep(100);
  delay(50);
  beep(100);
}

void beepError() {
  beep(500);
}

// ===== OLED 顯示 =====
void showOLED(String line1, String line2 = "", String line3 = "") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  if (line1.length() > 0) u8g2.drawStr(0, 12, line1.c_str());
  if (line2.length() > 0) u8g2.drawStr(0, 28, line2.c_str());
  if (line3.length() > 0) u8g2.drawStr(0, 44, line3.c_str());
  
  u8g2.sendBuffer();
}

String uidToString() {
  String result = "";
  for (byte i = 0; i < uidSize; i++) {
    if (uid[i] < 0x10) result += "0";
    result += String(uid[i], HEX);
    if (i < uidSize - 1) result += ":";
  }
  result.toUpperCase();
  return result;
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== HW-364A + RC522 (Full) ===");

  u8g2.begin();
  showOLED("HW-364A", "RFID System", "Init...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("Init Soft SPI...");
  spiBegin();
  delay(100);

  Serial.println("Reset RFID...");
  digitalWrite(RST_PIN, LOW);
  delay(50);
  digitalWrite(RST_PIN, HIGH);
  delay(50);

  reset();
  delay(100);

  byte version = readRegister(VersionReg);
  Serial.print("MFRC522 Version: 0x");
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF) {
    showOLED("ERROR!", "RFID fail", "");
    beepError();
    while(1);
  }

  // 設定WiFi AP模式
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP 地址: ");
  Serial.println(IP);

  // 設定網頁服務器路由
  server.on("/", handleRoot);
  server.on("/read", handleRead);
  server.on("/write", handleWrite);
  server.on("/mode", handleMode);
  server.on("/status", handleStatus);

  server.begin();
  Serial.println("Web 服務器開始運行");
  Serial.print("連接到 WiFi AP: ");
  Serial.print(ssid);
  Serial.print(" 密碼: ");
  Serial.println(password);
  Serial.print("在網頁瀏覽器輸入: http://");
  Serial.println(IP);

  beepSuccess();
  showOLED("Ready!", "WiFi AP ON", "Connect to RFID");
}

// ===== 網頁服務器處理函數 =====
void handleRoot() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>RFID 打卡系統</title>";
  html += "<style>body { font-family: Arial, sans-serif; text-align: center; }";
  html += ".container { max-width: 600px; margin: 0 auto; padding: 20px; }";
  html += ".btn { display: inline-block; padding: 10px 20px; margin: 10px; ";
  html += "background-color: #4CAF50; color: white; text-decoration: none; ";
  html += "border-radius: 4px; border: none; cursor: pointer; font-size: 16px; }";
  html += ".btn-read { background-color: #2196F3; }";
  html += ".btn-write { background-color: #FF9800; }";
  html += "input[type='text'] { width: 80%; padding: 10px; margin: 10px; font-size: 16px; }";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<h1>RFID 打卡系統</h1>";
  html += "<p>IP: " + WiFi.softAPIP().toString() + "</p>";

  html += "<h3>模式設定</h3>";
  html += "<a class='btn' href='/mode?set=read'>設定為讀取模式</a>";
  html += "<a class='btn' href='/mode?set=write'>設定為寫入模式</a><br>";

  html += "<h3>讀取卡片</h3>";
  html += "<a class='btn btn-read' href='/read'>讀取卡片</a><br>";

  html += "<h3>寫入卡片</h3>";
  html += "<form action='/write' method='GET'>";
  html += "<input type='text' name='data' placeholder='輸入要寫入的資料' maxlength='16'>";
  html += "<input type='submit' class='btn btn-write' value='寫入卡片'>";
  html += "</form>";

  html += "<h3>狀態查詢</h3>";
  html += "<a class='btn' href='/status'>查詢狀態</a><br>";

  html += "<p><a href='/'>重新整理</a></p>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

void handleRead() {
  // 執行讀取操作
  byte tagType[2];
  if (request(PICC_REQIDL, tagType) == 0) {
    byte serNum[5];
    if (antiCollision(serNum) == 0) {
      for (byte i = 0; i < 4; i++) {
        uid[i] = serNum[i];
      }
      uidSize = 4;

      String cardUID = uidToString();

      byte key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      byte blockAddr = 4;

      if (authenticate(PICC_AUTHENT1A, blockAddr, key, uid) == 0) {
        byte data[16];
        if (readBlock(blockAddr, data) == 0) {
          String text = "";
          for (int i = 0; i < 16; i++) {
            if (data[i] == 0) break;
            text += (char)data[i];
          }

          showOLED("Read OK!", cardUID, text);
          beep(100);

          String response = "{ \"success\": true, \"uid\": \"" + cardUID + "\", \"data\": \"" + text + "\" }";
          server.send(200, "application/json", response);
        } else {
          showOLED("Read FAIL", cardUID, "");
          beepError();
          server.send(200, "application/json", "{ \"success\": false, \"error\": \"Read failed\" }");
        }
      } else {
        showOLED("Auth FAIL", cardUID, "");
        beepError();
        server.send(200, "application/json", "{ \"success\": false, \"error\": \"Authentication failed\" }");
      }
      halt();
    } else {
      server.send(200, "application/json", "{ \"success\": false, \"error\": \"No card detected\" }");
    }
  } else {
    server.send(200, "application/json", "{ \"success\": false, \"error\": \"No card detected\" }");
  }
}

void handleWrite() {
  if(server.hasArg("data")) {
    String dataToWrite = server.arg("data");

    // 執行寫入操作
    byte tagType[2];
    if (request(PICC_REQIDL, tagType) == 0) {
      byte serNum[5];
      if (antiCollision(serNum) == 0) {
        for (byte i = 0; i < 4; i++) {
          uid[i] = serNum[i];
        }
        uidSize = 4;

        String cardUID = uidToString();

        byte key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        byte blockAddr = 4;

        if (authenticate(PICC_AUTHENT1A, blockAddr, key, uid) == 0) {
          byte data[16] = {0};
          for (int i = 0; i < dataToWrite.length() && i < 16; i++) {
            data[i] = dataToWrite[i];
          }

          if (writeBlock(blockAddr, data) == 0) {
            showOLED("Write OK!", cardUID, dataToWrite);
            beepSuccess();

            String response = "{ \"success\": true, \"uid\": \"" + cardUID + "\", \"data\": \"" + dataToWrite + "\" }";
            server.send(200, "application/json", response);
          } else {
            showOLED("Write FAIL", cardUID, "");
            beepError();
            server.send(200, "application/json", "{ \"success\": false, \"error\": \"Write failed\" }");
          }
        } else {
          showOLED("Auth FAIL", cardUID, "");
          beepError();
          server.send(200, "application/json", "{ \"success\": false, \"error\": \"Authentication failed\" }");
        }
        halt();
      } else {
        server.send(200, "application/json", "{ \"success\": false, \"error\": \"No card detected\" }");
      }
    } else {
      server.send(200, "application/json", "{ \"success\": false, \"error\": \"No card detected\" }");
    }
  } else {
    server.send(200, "application/json", "{ \"success\": false, \"error\": \"No data provided\" }");
  }
}

void handleMode() {
  if(server.hasArg("set")) {
    String mode = server.arg("set");
    if(mode == "read") {
      writeMode = false;
      showOLED("READ MODE", "Tap card", "");
      server.send(200, "application/json", "{ \"success\": true, \"mode\": \"read\" }");
    } else if(mode == "write") {
      writeMode = true;
      showOLED("WRITE MODE", "Tap card", "");
      server.send(200, "application/json", "{ \"success\": true, \"mode\": \"write\" }");
    } else {
      server.send(200, "application/json", "{ \"success\": false, \"error\": \"Invalid mode\" }");
    }
  } else {
    server.send(200, "application/json", "{ \"success\": false, \"error\": \"No mode specified\" }");
  }
}

void handleStatus() {
  String status = "{";
  status += "\"ssid\":\"" + String(ssid) + "\",";
  status += "\"ip\":\"" + WiFi.softAPIP().toString() + "\",";
  status += "\"mode\":\"" + String(writeMode ? "write" : "read") + "\",";
  status += "\"last_uid\":\"" + lastUID + "\"";
  status += "}";
  server.send(200, "application/json", status);
}

// ===== LOOP =====
void loop() {
  // 處理網頁請求
  server.handleClient();

  // 處理串口命令
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.startsWith("WRITE:")) {
      // 解析寫入命令，例如: WRITE:00100
      String dataToWrite = command.substring(6); // 移除 "WRITE:" 前綴

      // 查找卡片並寫入
      byte tagType[2];
      if (request(PICC_REQIDL, tagType) == 0) {
        byte serNum[5];
        if (antiCollision(serNum) == 0) {
          for (byte i = 0; i < 4; i++) {
            uid[i] = serNum[i];
          }
          uidSize = 4;

          String cardUID = uidToString();
          Serial.println("Card UID: " + cardUID);

          // 執行寫入操作
          byte key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          byte blockAddr = 4;

          if (authenticate(PICC_AUTHENT1A, blockAddr, key, uid) == 0) {
            byte data[16] = {0};
            for (int i = 0; i < dataToWrite.length() && i < 16; i++) {
              data[i] = dataToWrite[i];
            }

            if (writeBlock(blockAddr, data) == 0) {
              showOLED("Write OK!", cardUID, dataToWrite);
              beepSuccess();
              Serial.println("Write Success: " + dataToWrite);
            } else {
              showOLED("Write FAIL", cardUID, "");
              beepError();
              Serial.println("Write Failed");
            }
          } else {
            showOLED("Auth FAIL", cardUID, "");
            beepError();
            Serial.println("Authentication Failed");
          }
          halt();
        } else {
          Serial.println("No card detected for write operation");
        }
      } else {
        Serial.println("No card detected for write operation");
      }
    }
    else if (command == "READ") {
      // 執行讀取命令
      byte tagType[2];
      if (request(PICC_REQIDL, tagType) == 0) {
        byte serNum[5];
        if (antiCollision(serNum) == 0) {
          for (byte i = 0; i < 4; i++) {
            uid[i] = serNum[i];
          }
          uidSize = 4;

          String cardUID = uidToString();
          Serial.println("Card UID: " + cardUID);

          byte key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
          byte blockAddr = 4;

          if (authenticate(PICC_AUTHENT1A, blockAddr, key, uid) == 0) {
            byte data[16];

            if (readBlock(blockAddr, data) == 0) {
              String text = "";
              for (int i = 0; i < 16; i++) {
                if (data[i] == 0) break;
                text += (char)data[i];
              }

              showOLED("Read OK!", cardUID, text);
              beep(100);
              Serial.println("Data: " + text);
            } else {
              showOLED("Read FAIL", cardUID, "");
              beepError();
              Serial.println("Read failed");
            }
          } else {
            showOLED("Auth FAIL", cardUID, "");
            beepError();
            Serial.println("Authentication Failed");
          }
          halt();
        } else {
          Serial.println("No card detected for read operation");
        }
      } else {
        Serial.println("No card detected for read operation");
      }
    }
    else if (command == "MODE:READ") {
      writeMode = false;
      showOLED("READ MODE", "Tap card", "");
      Serial.println("Switched to READ mode");
    }
    else if (command == "MODE:WRITE") {
      writeMode = true;
      showOLED("WRITE MODE", "Tap card", "");
      Serial.println("Switched to WRITE mode");
    }
  }

  // 按鈕控制仍然有效
  if (digitalRead(BUTTON_PIN) == LOW) {
    writeMode = !writeMode;
    beep(50);
    showOLED(writeMode ? "WRITE MODE" : "READ MODE", "Tap card", "");
    Serial.println(writeMode ? "-> WRITE" : "-> READ");
    delay(300);
    while (digitalRead(BUTTON_PIN) == LOW);
  }

  // 如果不是通過串口命令操作，則執行正常的讀卡流程
  if (!Serial.available()) {  // 當沒有串口命令時
    byte tagType[2];
    if (request(PICC_REQIDL, tagType) != 0) {
      delay(50);
      return;
    }

    byte serNum[5];
    if (antiCollision(serNum) != 0) {
      delay(50);
      return;
    }

    for (byte i = 0; i < 4; i++) {
      uid[i] = serNum[i];
    }
    uidSize = 4;

    lastUID = uidToString();
    Serial.println("Card UID: " + lastUID);
    beep(50);

    if (writeMode) {
      showOLED("Writing...", lastUID, "");

      byte key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      byte blockAddr = 4;

      if (authenticate(PICC_AUTHENT1A, blockAddr, key, uid) != 0) {
        showOLED("Auth FAIL", lastUID, "");
        beepError();
        Serial.println("Auth failed");
      } else {
        byte data[16] = {0};
        String text = "HELLO_AVEI";
        for (int i = 0; i < text.length() && i < 16; i++) {
          data[i] = text[i];
        }

        if (writeBlock(blockAddr, data) == 0) {
          showOLED("Write OK!", lastUID, text);
          beepSuccess();
          Serial.println("Wrote: " + text);
        } else {
          showOLED("Write FAIL", lastUID, "");
          beepError();
          Serial.println("Write failed");
        }
      }
    } else {
      showOLED("Reading...", lastUID, "");

      byte key[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      byte blockAddr = 4;

      if (authenticate(PICC_AUTHENT1A, blockAddr, key, uid) != 0) {
        showOLED("Auth FAIL", lastUID, "");
        beepError();
        Serial.println("Auth failed");
      } else {
        byte data[16];

        if (readBlock(blockAddr, data) == 0) {
          String text = "";
          for (int i = 0; i < 16; i++) {
            if (data[i] == 0) break;
            text += (char)data[i];
          }

          showOLED("Read OK!", lastUID, text);
          beep(100);
          Serial.println("Data: " + text);
        } else {
          showOLED("Read FAIL", lastUID, "");
          beepError();
          Serial.println("Read failed");
        }
      }
    }

    halt();
    delay(5000);
    showOLED(writeMode ? "WRITE MODE" : "READ MODE", "Tap card", "");
  }
}