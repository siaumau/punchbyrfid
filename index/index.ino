#include <U8g2lib.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ===== WiFi 設定 =====
const char* ssid = "RFID-Punch-System";
const char* password = "123456789";

// ===== Web 服務器 =====
ESP8266WebServer server(80);

// ===== HW-364A OLED (硬體 I2C) =====
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 12, /* data=*/ 14);

// ===== 硬體腳位定義 =====
#define BUTTON_PIN 13 
#define BUZZER_PIN 2 
#define RST_PIN    5 
#define SS_PIN     15 
#define MOSI_PIN   0 
#define MISO_PIN   16 
#define SCK_PIN    4 

// ===== MFRC522 暫存器與命令 =====
#define CommandReg    0x01
#define ComIEnReg     0x02
#define ComIrqReg     0x04
#define ErrorReg      0x06
#define Status2Reg    0x08
#define FIFODataReg   0x09
#define FIFOLevelReg  0x0A
#define BitFramingReg 0x0D
#define ModeReg       0x11
#define TxControlReg  0x14
#define VersionReg    0x37

#define PCD_Idle       0x00
#define PCD_Transceive 0x0C
#define PCD_MFAuthent  0x0E
#define PCD_SoftReset  0x0F

#define PICC_REQIDL    0x26
#define PICC_ANTICOLL  0x93
#define PICC_AUTHENT1A 0x60
#define PICC_READ      0x30
#define PICC_WRITE     0xA0
#define PICC_HALT      0x50

// ===== 全域變數 =====
String lastUID = "";
bool writeMode = false;
bool isProcessing = false; 
byte uid[5] = {0};
byte uidSize = 0;

// ===== 軟體 SPI 實作 =====
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
    digitalWrite(SCK_PIN, HIGH);
    delayMicroseconds(2);
    if (digitalRead(MISO_PIN)) result |= (1 << i);
    digitalWrite(SCK_PIN, LOW);
    delayMicroseconds(2);
  }
  return result;
}

// ===== RC522 寄存器操作 =====
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
  writeRegister(reg, readRegister(reg) | mask);
}

void clearBitMask(byte reg, byte mask) {
  writeRegister(reg, readRegister(reg) & (~mask));
}

void reset() {
  digitalWrite(RST_PIN, LOW); delay(20); digitalWrite(RST_PIN, HIGH); delay(20);
  writeRegister(CommandReg, PCD_SoftReset);
  delay(50);
  writeRegister(0x2A, 0x8D);
  writeRegister(0x2B, 0x3E);
  writeRegister(0x2D, 0x1E);
  writeRegister(0x2C, 0x00);
  writeRegister(0x15, 0x40);
  writeRegister(ModeReg, 0x3D);
  setBitMask(TxControlReg, 0x03); // 開啟天線
}

byte communicate(byte command, byte *sendData, byte sendLen, byte *backData, byte *backLen) {
  byte irqEn = (command == PCD_MFAuthent) ? 0x12 : 0x77;
  byte waitIRq = (command == PCD_MFAuthent) ? 0x10 : 0x30;
  
  writeRegister(ComIEnReg, irqEn | 0x80);
  clearBitMask(ComIrqReg, 0x80);
  setBitMask(FIFOLevelReg, 0x80);
  writeRegister(CommandReg, PCD_Idle);
  
  for (byte i = 0; i < sendLen; i++) writeRegister(FIFODataReg, sendData[i]);
  writeRegister(CommandReg, command);
  if (command == PCD_Transceive) setBitMask(BitFramingReg, 0x80);
  
  uint16_t i = 2000;
  byte n;
  do { n = readRegister(ComIrqReg); i--; } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));
  
  clearBitMask(BitFramingReg, 0x80);
  if (i == 0) return 1; 
  if (readRegister(ErrorReg) & 0x1B) return 2; 
  
  if (backData && backLen) {
    n = readRegister(FIFOLevelReg);
    *backLen = n;
    for (byte j = 0; j < n; j++) backData[j] = readRegister(FIFODataReg);
  }
  return 0;
}

byte request(byte reqMode, byte *tagType) {
  writeRegister(BitFramingReg, 0x07);
  byte backLen;
  return communicate(PCD_Transceive, &reqMode, 1, tagType, &backLen);
}

byte antiCollision(byte *serNum) {
  writeRegister(BitFramingReg, 0x00);
  byte buff[2] = {PICC_ANTICOLL, 0x20};
  byte backLen;
  return communicate(PCD_Transceive, buff, 2, serNum, &backLen);
}

byte authenticate(byte authMode, byte blockAddr, byte *sectorKey, byte *serNum) {
  byte buff[12];
  buff[0] = authMode; buff[1] = blockAddr;
  for (byte i = 0; i < 6; i++) buff[i + 2] = sectorKey[i];
  for (byte i = 0; i < 4; i++) buff[i + 8] = serNum[i];
  
  // 修正點：這裡應使用 PCD_MFAuthent
  if (communicate(PCD_MFAuthent, buff, 12, NULL, NULL) != 0) return 1;
  return (readRegister(Status2Reg) & 0x08) ? 0 : 1;
}

byte readBlock(byte blockAddr, byte *recvData) {
  byte buff[2] = {PICC_READ, blockAddr};
  byte backLen;
  return communicate(PCD_Transceive, buff, 2, recvData, &backLen);
}

byte writeBlock(byte blockAddr, byte *writeData) {
  byte buff[2] = {PICC_WRITE, blockAddr};
  byte backLen, recvData[16];
  if (communicate(PCD_Transceive, buff, 2, recvData, &backLen) != 0) return 1;
  return communicate(PCD_Transceive, writeData, 16, recvData, &backLen);
}

void halt() {
  byte buff[2] = {PICC_HALT, 0x00};
  communicate(PCD_Transceive, buff, 2, NULL, NULL);
  clearBitMask(Status2Reg, 0x08);
  delay(10); // 添加短暫延遲確保卡片完全停止
}

// ===== UI 與 輔助功能 =====
void beep(int d) { digitalWrite(BUZZER_PIN, HIGH); delay(d); digitalWrite(BUZZER_PIN, LOW); }

void showOLED(String l1, String l2 = "", String l3 = "") {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, l1.c_str());
  u8g2.drawStr(0, 28, l2.c_str());
  u8g2.drawStr(0, 44, l3.c_str());
  u8g2.sendBuffer();
}

String uidToString() {
  String res = "";
  for (byte i = 0; i < 4; i++) {
    if (uid[i] < 0x10) res += "0";
    res += String(uid[i], HEX);
    if (i < 3) res += ":";
  }
  res.toUpperCase();
  return res;
}

// ===== Web Server 處理 =====
void handleRoot() {
  String html = "<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<style>body{font-family:sans-serif;text-align:center;padding:20px;} .btn{padding:15px;margin:10px;width:80%;font-size:18px;color:white;border-radius:8px;border:none;} .blue{background:#2196F3;} .orange{background:#FF9800;} #res{margin-top:20px;font-weight:bold;color:#333;}</style>";
  html += "</head><body><h1>RFID 打卡管理</h1>";
  html += "<button class='btn blue' onclick='doAction(\"/read\")'>讀取卡片</button><br>";
  html += "<input type='text' id='d' placeholder='要寫入的資料' style='width:70%;padding:10px;'><br>";
  html += "<button class='btn orange' onclick='doWrite()'>寫入卡片</button>";
  html += "<div id='res'>請靠卡後操作</div>";
  html += "<script>function doAction(url){document.getElementById(\"res\").innerText=\"執行中...\";fetch(url).then(r=>r.json()).then(data=>{if(data.success){document.getElementById(\"res\").innerHTML=\"UID: \"+data.uid+\"<br>內容: \"+data.data;}else{document.getElementById(\"res\").innerText=\"錯誤: \"+data.error;}}).catch(e=>document.getElementById(\"res\").innerText=\"連線失敗\");}";
  html += "function doWrite(){const val=document.getElementById(\"d\").value; if(!val){alert(\"請輸入資料\");return;} doAction(\"/write?data=\"+encodeURIComponent(val));}</script></body></html>";
  server.send(200, "text/html", html);
}

void handleRead() {
  isProcessing = true;

  // 短暫延遲以確保其他處理完成
  delay(50);

  String res = "{\"success\":false,\"error\":\"No Card detected\"}";
  byte type[2], sn[5], data[16];

  // 定義多組常見默認金鑰
  byte keys[][6] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  // 最常見的默認金鑰
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // 全零金鑰
    {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7},  // 常見默認金鑰
    {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5},  // 常見默認金鑰
    {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5},  // 其他常見金鑰
    {0x4D, 0x3A, 0x99, 0xC3, 0x51, 0xDD}   // 其他常見金鑰
  };

  // 重新初始化RFID模組
  reset();

  if (request(PICC_REQIDL, type) == 0 && antiCollision(sn) == 0) {
    for(int i=0; i<4; i++) uid[i]=sn[i];

    // 只返回UID而不嘗試讀取數據，如果認證失敗
    String uidStr = uidToString();

    // 優化：嘗試多個可能的塊，包括扇區1的塊5
    bool auth_ok = false;
    byte blocks_to_try[] = {4, 5, 0, 8, 1, 2, 3}; // 優先嘗試範例中使用的塊5
    int num_blocks = sizeof(blocks_to_try)/sizeof(blocks_to_try[0]);
    int num_keys = sizeof(keys)/sizeof(keys[0]); // 更新金鑰數量計算

    for(int b = 0; b < num_blocks && !auth_ok; b++) {
      byte block = blocks_to_try[b];

      // 避免嘗試控制塊（每個扇區的最後一塊）
      if((block + 1) % 4 == 0) continue;

      reset(); // 每次嘗試前重新初始化
      if(request(PICC_REQIDL, type) == 0 && antiCollision(sn) == 0) {
        for(int i=0; i<4; i++) uid[i]=sn[i]; // 重新獲取UID

        for(int k=0; k<num_keys && !auth_ok; k++) { // 使用動態金鑰數量
          if (authenticate(PICC_AUTHENT1A, block, keys[k], uid) == 0) {
            auth_ok = true;

            // 成功認證後，嘗試讀取數據
            if (readBlock(block, data) == 0) {
              String msg = "";
              for(int i=0; i<16 && data[i]!=0; i++) {
                if(data[i] >= 32 && data[i] <= 126) { // 只添加可打印字符
                  msg += (char)data[i];
                } else {
                  msg += "[" + String(data[i], HEX) + "]"; // 不可打印字符顯示為十六進制
                }
              }
              res = "{\"success\":true,\"uid\":\""+uidStr+"\",\"data\":\""+msg+"\"}";
              beep(100);
            } else {
              // 即使讀取數據失敗，也返回UID
              res = "{\"success\":true,\"uid\":\""+uidStr+"\",\"data\":\"Read data failed, UID only\"}";
              beep(100);
            }
            break; // 成功後立即退出循環
          }
        }
      }
    }

    if (!auth_ok) {
      // 如果所有金鑰和扇區都無法認證，至少返回UID
      res = "{\"success\":true,\"uid\":\""+uidStr+"\",\"data\":\"Card detected but all keys failed\",\"info\":\"Card may be protected or not MIFARE Classic\"}";
      beep(100); // 即使無法讀取數據也發出提示音
    }

    halt();
  }

  server.send(200, "application/json", res);
  isProcessing = false;
}

void handleWrite() {
  isProcessing = true;

  // 短暫延遲以確保其他處理完成
  delay(50);

  String res = "{\"success\":false,\"error\":\"No Card detected\"}";
  String toWrite = server.arg("data");
  byte type[2], sn[5], data[16]={0};

  // 定義多組常見默認金鑰
  byte keys[][6] = {
    {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},  // 最常見的默認金鑰
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},  // 全零金鑰
    {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7},  // 常見默認金鑰
    {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5},  // 常見默認金鑰
    {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5},  // 其他常見金鑰
    {0x4D, 0x3A, 0x99, 0xC3, 0x51, 0xDD}   // 其他常見金鑰
  };

  // 重新初始化RFID模組
  reset();

  if (request(PICC_REQIDL, type) == 0 && antiCollision(sn) == 0) {
    for(int i=0;i<4;i++) uid[i]=sn[i];

    bool write_ok = false;
    // 根據範例代碼，嘗試多個可能的塊，包括扇區1的塊5
    byte blocks_to_try[] = {4, 5, 0, 8, 1, 2, 3}; // 優先嘗試範例中使用的塊5
    int num_blocks = sizeof(blocks_to_try)/sizeof(blocks_to_try[0]);
    int num_keys = sizeof(keys)/sizeof(keys[0]); // 更新金鑰數量計算

    for(int b = 0; b < num_blocks && !write_ok; b++) {
      byte block = blocks_to_try[b];

      // 避免嘗試控制塊（每個扇區的最後一塊）
      if((block + 1) % 4 == 0) continue;

      reset(); // 每次嘗試前重新初始化
      if(request(PICC_REQIDL, type) == 0 && antiCollision(sn) == 0) {
        for(int i=0; i<4; i++) uid[i]=sn[i]; // 重新獲取UID

        for(int k=0; k<num_keys && !write_ok; k++) { // 使用動態金鑰數量
          if (authenticate(PICC_AUTHENT1A, block, keys[k], uid) == 0) {
            // 準備要寫入的數據
            for(int i=0; i<toWrite.length() && i<16; i++) data[i] = toWrite[i];

            // 填充剩餘空間為0
            for(int i=toWrite.length(); i<16; i++) data[i] = 0;

            if (writeBlock(block, data) == 0) {
              res = "{\"success\":true,\"uid\":\""+uidToString()+"\",\"data\":\""+toWrite+"\",\"sector\":"+String(block)+"}";
              beep(100); delay(50); beep(100);
              write_ok = true;
              break;
            }
          }
        }
      }
    }

    if(!write_ok) {
      res = "{\"success\":false,\"error\":\"Auth failed for all keys and blocks\",\"uid\":\""+uidToString()+"\"}";
    }
    halt();
  }
  server.send(200, "application/json", res);
  isProcessing = false;
}

// ===== 主程式 =====
void setup() {
  Serial.begin(115200);
  u8g2.begin();
  spiBegin();
  reset();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/read", handleRead);
  server.on("/write", handleWrite);
  server.begin();
  
  showOLED("WiFi Started", "IP: 192.168.4.1", "Ready!");
  beep(100);
}

void loop() {
  server.handleClient();

  if (isProcessing) return;

  byte type[2], sn[5];
  // 背景持續檢測卡片，僅用於 OLED 顯示
  if (request(PICC_REQIDL, type) == 0 && antiCollision(sn) == 0) {
    for(int i=0; i<4; i++) uid[i] = sn[i];
    lastUID = uidToString();
    showOLED("Card Active", lastUID, "Wait for Web...");
    halt();
    delay(1000); // 增加延遲時間，給用戶更多時間查看UID
  }
}