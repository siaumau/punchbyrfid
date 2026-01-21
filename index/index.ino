#include <U8g2lib.h>

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
  
  beepSuccess();
  showOLED("Ready!", "Tap card", "READ MODE");
  Serial.println("Ready!");
}

// ===== LOOP =====
void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    writeMode = !writeMode;
    beep(50);
    showOLED(writeMode ? "WRITE MODE" : "READ MODE", "Tap card", "");
    Serial.println(writeMode ? "-> WRITE" : "-> READ");
    delay(300);
    while (digitalRead(BUTTON_PIN) == LOW);
  }
  
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