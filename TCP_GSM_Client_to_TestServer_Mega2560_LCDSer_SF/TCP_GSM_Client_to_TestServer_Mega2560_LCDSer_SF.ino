/*
  TCP_Client_to_TestServer
 
 This sketch tries to do a communicate GPS coordinates to a
 Python test server script.
 
 */

#define SALT_LENGTH  32
#define AES_BLOCK_SIZE  16
#define SHA256_KEY_SIZE  32
#define AES256_KEY_SIZE  32
#define SERIAL_RECEIVE_BUFFER_SIZE 128

#include <avr/pgmspace.h>

// These include the cryptographic algorithms from the AVR Crypto library
// http://avrcryptolib.das-labor.org/trac
#include "sha256.h"
#include "hmacsha256.h"
#include "aes.h"

HardwareSerial gprsSerial(Serial);    // http://imall.iteadstudio.com/im140318007.html
HardwareSerial gpsSerial(Serial1);    // http://imall.iteadstudio.com/im120417017.html
HardwareSerial lcdSerial(Serial2);    // https://www.sparkfun.com/products/9395?gclid=CNmLyKKRv8UCFUc2gQod5CkAhA

#define GPRS_SERIAL_BAUD 19200
#define GPS_SERIAL_BAUD 9600
#define LCD_SERIAL_BAUD 9600

const char* webAddress = "gpsserver.example.com";
const char* portNumber = "7000";

/*
const char* apnServer = "epc.tmobile.com";
const char* apnUserName = "";
const char* apnPassWord = "";
*/

const char* apnServer = "wap.cingular";
const char* apnUserName = "wap@cingulargprs.com";
const char* apnPassWord = "cingular1";

unsigned int gprsConnected = 0;
unsigned int gpsConnected = 0;
unsigned long nextUpdate = 0;            // Time to store next time to update in milliseconds
//unsigned long updateInterval = 3600000;  // Do update every hour in milliseconds
unsigned long updateInterval = 60000;  // Do update every minute in milliseconds

unsigned long timeOutInterval = 60000;  // How long until timeout
unsigned long timeOut;                  // If no activity for a minute, times out.

// Storing hash keys in PROGMEM because I was told it was the right thing to do.
prog_char randKey_0[] PROGMEM = "0123456789abcdef";
prog_char randKey_1[] PROGMEM = "123456789abcdef0";
prog_char randKey_2[] PROGMEM = "23456789abcdef01";
prog_char randKey_3[] PROGMEM = "3456789abcdef012";
prog_char randKey_4[] PROGMEM = "456789abcdef0123";
prog_char randKey_5[] PROGMEM = "56789abcdef01234";
prog_char randKey_6[] PROGMEM = "6789abcdef012345";
prog_char randKey_7[] PROGMEM = "789abcdef0123456";
prog_char randKey_8[] PROGMEM = "89abcdef01234567";
prog_char randKey_9[] PROGMEM = "9abcdef012345678";
prog_char randKey_10[] PROGMEM = "abcdef0123456789";
prog_char randKey_11[] PROGMEM = "bcdef0123456789a";
prog_char randKey_12[] PROGMEM = "cdef0123456789ab";
prog_char randKey_13[] PROGMEM = "def0123456789abc";
prog_char randKey_14[] PROGMEM = "ef0123456789abcd";
prog_char randKey_15[] PROGMEM = "f0123456789abcde";
prog_char aesAuthKey[] PROGMEM = "0123456789abdef0123456789abdef";

PROGMEM const char *randKeys[] = {
  randKey_0,
  randKey_1,
  randKey_2,
  randKey_3,
  randKey_4,
  randKey_5,
  randKey_6,
  randKey_7,
  randKey_8,
  randKey_9,
  randKey_10,
  randKey_11,
  randKey_12,
  randKey_13,
  randKey_14,
  randKey_15,
  aesAuthKey
};

char hexDigits[] = "0123456789abcdef";

uint8_t cbcComBlock[AES_BLOCK_SIZE];
uint8_t aesComKey[AES256_KEY_SIZE];

uint8_t aesComKeyIndex = 0;

int doGPSCheckSum(char* bufferPointerIn) {

  int gpsCheckSumInt = 0;

  char funcCheckSum[3];
  char gpsCheckSum[3];
  char* tokenPointer;
  char* checkSumPointer = bufferPointerIn;

  checkSumPointer++;
  tokenPointer = strchr(checkSumPointer, '*');

  if (tokenPointer != NULL) {

    for (int i = 0; i < (tokenPointer-checkSumPointer); i++) {
      gpsCheckSumInt ^= checkSumPointer[i];
    }

    funcCheckSum[2] = '\0';

    funcCheckSum[1] = hexDigits[gpsCheckSumInt&0x0f];
    if ((gpsCheckSumInt&0x0f) >= 10) {
      funcCheckSum[1] &= 0xdf;
    }

    gpsCheckSumInt = gpsCheckSumInt>>4;

    funcCheckSum[0] = hexDigits[gpsCheckSumInt&0x0f];
    if ((gpsCheckSumInt&0x0f) >= 10) {
      funcCheckSum[0] &= 0xdf;
    }

  } 
  else {
    return 0;
  }

  tokenPointer++;
  gpsCheckSum[0] = tokenPointer[0];
  gpsCheckSum[1] = tokenPointer[1];
  gpsCheckSum[2] = '\0';

  if (strcmp(funcCheckSum, gpsCheckSum) == 0) {
    return 1;
  } 
  else {
    return 0;
  }

}

void loadGPRSCharArray(char* charArray, char* arrayTerminators) {

  unsigned char charIndex = 0;
  unsigned char terminatorIndex = 0;

  while (true) {

    if (gprsSerial.available() <= 0) {
      charArray[charIndex] = '\0';
      break;
    }

    charArray[charIndex] = gprsSerial.read();

    for (terminatorIndex = 0; terminatorIndex < (sizeof(arrayTerminators)/sizeof(char)); terminatorIndex++) {
      if (charArray[charIndex] == arrayTerminators[terminatorIndex]) {
        charArray[charIndex] = '\0';
      }
    }

    if (charArray[charIndex] == '\0') {
      break;
    }

    charIndex++;
  }
}

void powerUpOrDown() {
  
  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("Power cycling   "));
  lcdSerial.print(F("GPRS . . .      "));
    
  digitalWrite(9,LOW);
  delay(1000);
  digitalWrite(9,HIGH);
  delay(2000);
  digitalWrite(9,LOW);
  delay(3000);
}

void printGPRSBuffer(char* gprsBuffer, int lineCount) {
  for (int i = 0; i < lineCount; i++) {
    loadGPRSCharArray(gprsBuffer, "\n");
  }
}

int sendGPRSData(char* gprsData) {

  char gprsBuffer[SERIAL_RECEIVE_BUFFER_SIZE];
  size_t dataLength;
  int sendDelay = 3000;

  dataLength = strlen(gprsData);
  dataLength++;

  gprsSerial.print(F("AT+CIPSEND="));
  gprsSerial.println(dataLength);
  delay(sendDelay);

  printGPRSBuffer(gprsBuffer, 1);

  gprsSerial.println(gprsData);
  delay(sendDelay);
  printGPRSBuffer(gprsBuffer, 1);

  while (true) {
    printGPRSBuffer(gprsBuffer, 1);
    if (strcmp(gprsBuffer, "\r") != 0) {
      break;
    } else if (timeOut <= millis()) {
      break;
    }
  }

  if (strcmp(gprsBuffer, "SEND OK\r") == 0) {
    return 1;
  } else {
    return 0;
  }
}

int doCommandHELLO() {

  if (sendGPRSData("HELLO")) {

    aesComKeyIndex = 0;
    delay(1000);

    return 1;

  } 
  else {
    return 0;
  }
}

int doCommandSENDCOORDS() {

  if (doCommandRECVCOORDS()) {
    if (sendGPRSData("OK")) {
      return 1;
    }
  }

  sendGPRSData("ERROR");
  return 0;
}

int doCommandCHALLENGE(char* messageEnc) {

  aes256_ctx_t aesCtx;
  hmac_sha256_ctx_t hashCtx;

  char aesAuthKey[AES256_KEY_SIZE]; 

  char randKey[32];
  int keyLen;

  char gprsBuffer[75];

  uint8_t aesBlock1[] = "0123456789abcdef";
  uint8_t aesBlock2[] = "0123456789abcdef";

  uint8_t hashCBCBlock[] = "0123456789abcdef";

  uint8_t randSalt[] = "0123456789abcdef0123456789abcdef";

  uint8_t hashOut[HMAC_SHA256_BYTES];
  uint8_t hashBuff[HMAC_SHA256_BLOCK_BYTES];

  char hashResponse[] = "RESPONSE 0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";

  strcpy_P(aesAuthKey, (char*)pgm_read_word(&(randKeys[16])));
  aes256_init(aesAuthKey, &aesCtx);

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    aesBlock1[i] = (strchr(hexDigits, messageEnc[(i*2)])-hexDigits)<<4;
    aesBlock1[i] |= (strchr(hexDigits, messageEnc[(i*2)+1])-hexDigits);
    aesBlock2[i] = (strchr(hexDigits, messageEnc[(AES_BLOCK_SIZE*2)+(i*2)])-hexDigits)<<4;
    aesBlock2[i] |= (strchr(hexDigits, messageEnc[(AES_BLOCK_SIZE*2)+(i*2)+1])-hexDigits);
    hashCBCBlock[i] = aesBlock2[i];
  }

  aes256_dec(aesBlock2, &aesCtx);
  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    aesBlock2[i] ^= aesBlock1[i];
  }

  aes256_dec(aesBlock1, &aesCtx);

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    randSalt[i] = aesBlock1[i];
    randSalt[AES_BLOCK_SIZE+i] = aesBlock2[i];
  }

  if (aesComKeyIndex < AES_BLOCK_SIZE) {
    cbcComBlock[aesComKeyIndex] = aesBlock1[aesBlock2[0]%16];
  }
  aesComKey[aesComKeyIndex] = aesBlock2[aesBlock1[0]%16];

  strcpy_P(randKey, (char*)pgm_read_word(&(randKeys[int(aesComKeyIndex/2)])));
  keyLen = int(strchr(randKey, '\0')-randKey);

  hmac_sha256_init(&hashCtx, randKey, (keyLen*8));
  hmac_sha256_lastBlock(&hashCtx, &randSalt, (SALT_LENGTH*8));
  hmac_sha256_final(&hashOut, &hashCtx);

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    aesBlock1[i] = hashOut[i];
    aesBlock2[i] = hashOut[AES_BLOCK_SIZE+i];
  }

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    aesBlock1[i] ^= hashCBCBlock[i];
  }
  aes256_enc(aesBlock1, &aesCtx);

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    aesBlock2[i] ^= aesBlock1[i];
  }
  aes256_enc(aesBlock2, &aesCtx);

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {
    hashResponse[(i*2)+9] = hexDigits[aesBlock1[i]>>4];
    hashResponse[(i*2)+10] = hexDigits[aesBlock1[i]&0xf];
    hashResponse[((i+AES_BLOCK_SIZE)*2)+9] = hexDigits[aesBlock2[i]>>4];
    hashResponse[((i+AES_BLOCK_SIZE)*2)+10] = hexDigits[aesBlock2[i]&0xf];
  }

  if (sendGPRSData(hashResponse)) {
    aesComKeyIndex++;
    return 1;
  } 
  else {
    return 0;
  }
}

int doCommandCHALLENGEPASS() {

  for (int i = 0; i < AES_BLOCK_SIZE; i++) {}
  for (int i = 0; i < AES256_KEY_SIZE; i++) {}

  if (sendGPRSData("OK")) {
    return 1;
  } 
  else {
    return 0;
  }
}

int doCommandBYE() {

  char gprsBuffer[17];

  while (gprsSerial.available()) {
    gprsSerial.read();
  }
  delay(1000);

  gprsSerial.println(F("AT+CIPCLOSE"));
  delay(10000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "CLOSE OK\r") != 0) {} 
  else {}
  delay(1000);

  gprsSerial.end();
  gprsConnected = 0;

  //nextUpdate = millis() + updateInterval;

  return 1;
}

int doCommandCHALLENGEFAIL() {

  if (sendGPRSData("OK")) {

    doCommandBYE();
    nextUpdate = millis() + 1000;

    return 1;

  } 
  else {
    return 0;
  }
}

int doCommandRECVJPG() {

  timeOut = millis() + timeOutInterval;
  while (Serial.available() <= 0) {
    if (millis() >= timeOut) {
      break;
    }
  }

  if (Serial.available() > 0) {

    int charIn;
    char hexOut[] = "XX";

    char bytesLength = 32;
    char lineCounter = 0;

    //client.println(F("RECVJPG"));

    while (true) {

      //String inString = client.readString();
      String inString = "";

      inString.trim();
      if (inString == "OK") {
        break;
      }

      timeOut = millis() + timeOutInterval;
      if (millis() >= timeOut) {
        doCommandBYE();
      }
    }

    while (true) {

      charIn = Serial.parseInt();

      hexOut[0] = hexDigits[charIn>>4];
      hexOut[1] = hexDigits[charIn&0xf];

      if (Serial.available() <= 0) {
        break;
      }

      if (lineCounter < bytesLength) {
        //client.print(hexOut);
        lineCounter++;
      } 
      else {
        //client.println(hexOut);
        lineCounter = 0;
      }
    }
    //client.println(hexOut);

    //client.println(F("ENDRECVJPG"));
  }
}

void copyTokenToCharPtr(char* chrPtr, int ptrLen, char* tokenPointer, char* nextTokenPointer) {
  
  strncpy(chrPtr, tokenPointer, (nextTokenPointer-tokenPointer));

  if ((nextTokenPointer-tokenPointer+1) < ptrLen) {
    chrPtr[(nextTokenPointer-tokenPointer)] = '\0';
  } 
  else {  
    chrPtr[ptrLen-1] = '\0';
  }
}

int doCommandRECVCOORDS() {

  char gprsBuffer[17];
  char gpsBuffer[SERIAL_RECEIVE_BUFFER_SIZE];
  char outBuffer[60];

  int bufferIndex = 0;
  int tokenIndex = 0;

  char* tokenPointer;
  char* nextTokenPointer;

  char gpsDate[7];
  char gpsTime[11];

  char gpsLatitude[11];
  char gpsLatitudeDirection[2];
  char gpsLongitude[11];
  char gpsLongitudeDirection[2];
  char gpsAltitude[11];
  char gpsAltitudeUnits[2];

  int gpsCheckSum = 0;

  int receivedCoords = 0;
  int receivedCoordsAlt = 0;
  
  timeOut = millis() + timeOutInterval;

  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("Gathering GPS   "));
  lcdSerial.print(F("coordinates.    "));
  
  //lcdSerial.end();

  gpsConnected = 0;
  gpsSerial.begin(GPS_SERIAL_BAUD);
  gpsSerial.flush();
  //gpsSerial.listen();
  gpsConnected = 1;
  
  outBuffer[0] = '\0';

  if (sendGPRSData("RECVCOORDS")) {
  //if (true) {
    
    printGPRSBuffer(gprsBuffer, 1);

    if (strcmp(gprsBuffer, "OK") == 0) {
    //if (true) {
      
      delay(1000);

      while (true) {

        delay(1000);

        for (bufferIndex = 0; bufferIndex < SERIAL_RECEIVE_BUFFER_SIZE; bufferIndex++) {
          gpsBuffer[bufferIndex] = '\0';
        }

        bufferIndex = 0;

        while (gpsSerial.available() <= 0) {
          if (millis() >= timeOut) {
            break;
          }
        }
        while (true) {

          gpsBuffer[bufferIndex] = gpsSerial.read();

          if (!((gpsSerial.available() > 0) && (strstr(gpsBuffer, "\n") == NULL) && (bufferIndex < SERIAL_RECEIVE_BUFFER_SIZE))) {
            break;
          }

          bufferIndex++;
        }

        gpsBuffer[bufferIndex] = '\0';
        //gprsSerial.println(gpsBuffer);
        //lcdSerial.print(gpsBuffer);
        
        tokenIndex = 0;
        tokenPointer = gpsBuffer;
        nextTokenPointer = strchr(tokenPointer, ',');

        if (strstr(gpsBuffer, "$GPRMC") != NULL) {

          while (true) {

            //Serial.println("Hello? $GPRMC");

            if (nextTokenPointer == NULL) {
              nextTokenPointer = tokenPointer + strlen(tokenPointer);
            }

            if (tokenIndex == 1) {
              copyTokenToCharPtr(gpsTime, 11, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 3) {
              copyTokenToCharPtr(gpsLatitude, 11, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 4) {
              copyTokenToCharPtr(gpsLatitudeDirection, 2, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 5) {
              copyTokenToCharPtr(gpsLongitude, 11, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 6) {
              copyTokenToCharPtr(gpsLongitudeDirection, 2, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 9) {
              copyTokenToCharPtr(gpsDate, 7, tokenPointer, nextTokenPointer);
            }

            tokenPointer = strchr(tokenPointer, ',');

            if ((tokenIndex >= 9) || (tokenPointer == NULL)) {     
              break;
            }

            tokenPointer++;
            nextTokenPointer = strchr(tokenPointer, ',');

            tokenIndex++;
          } 

          if (doGPSCheckSum(gpsBuffer)) {
            receivedCoords++;
          } else {
            gpsSerial.flush();
            receivedCoords = 0;
          }

        } else if (strstr(gpsBuffer, "$GPGGA") != NULL) {

          while (true) {

            //Serial.println("Hello? $GPGGA");

            if (nextTokenPointer == NULL) {
              nextTokenPointer = tokenPointer + strlen(tokenPointer);
            }

            /*
            if (tokenIndex == 1) {
             copyTokenToCharPtr(gpsTime, 17, tokenPointer, nextTokenPointer);
             } else
             */

            if (tokenIndex == 2) {
              copyTokenToCharPtr(gpsLatitude, 11, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 3) {
              copyTokenToCharPtr(gpsLatitudeDirection, 2, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 4) {
              copyTokenToCharPtr(gpsLongitude, 11, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 5) {
              copyTokenToCharPtr(gpsLongitudeDirection, 2, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 9) {
              copyTokenToCharPtr(gpsAltitude, 11, tokenPointer, nextTokenPointer);
            } 
            else if (tokenIndex == 10) {
              copyTokenToCharPtr(gpsAltitudeUnits, 2, tokenPointer, nextTokenPointer);
            }

            tokenPointer = strchr(tokenPointer, ',');

            if ((tokenIndex >= 10) || (tokenPointer == NULL)) {
              break;
            }

            tokenPointer++;
            nextTokenPointer = strchr(tokenPointer, ',');

            tokenIndex++;
          }

          if (doGPSCheckSum(gpsBuffer)) {
            receivedCoordsAlt++;
          } else {
            gpsSerial.flush();
            receivedCoordsAlt = 0;
          }
        }

        //if (receivedCoords && ((receivedCoords > 10) || (receivedCoords && receivedCoordsAlt))) {
        if (receivedCoords && receivedCoordsAlt) {
    
          strcpy(outBuffer, gpsDate);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsTime);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsLatitude);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsLatitudeDirection);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsLongitude);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsLongitudeDirection);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsAltitude);
          strcat(outBuffer, ",");
          strcat(outBuffer, gpsAltitudeUnits);
    
          break;
          
        } else if (millis() >= timeOut) {
          break;
        }
      }
    }
  }
  
  gpsSerial.flush();
  gpsSerial.end();
  gpsConnected = 0;
  
  //lcdSerial.begin(LCD_SERIAL_BAUD);
  //lcdSerial.flush();
  //lcdSerial.listen();
  
  //gprsSerial.println(outBuffer);
  //lcdSerial.println(outBuffer);
  
  if (receivedCoords && receivedCoordsAlt) {
    
    if (sendGPRSData(outBuffer)) {
      
      printGPRSBuffer(gprsBuffer, 1);
      
      if (strcmp(gprsBuffer, "OK") == 0) {
    
        lcdSerial.write(254);
        lcdSerial.write(128);
        
        lcdSerial.print(F("Lat: "));  
        for (int i = 0; i < (strlen(gpsLatitude)); i++) {
          lcdSerial.write(gpsLatitude[i]);
        }
        for (int i = 0; i < (strlen(gpsLatitudeDirection)); i++) {
          lcdSerial.write(gpsLatitudeDirection[i]);
        }
        for (int i = 0; i < (11 - strlen(gpsLatitude) - strlen(gpsLatitudeDirection)); i++) {
          lcdSerial.write(' ');
        }
        
        lcdSerial.print(F("Lng: "));  
        for (int i = 0; i < (strlen(gpsLongitude)); i++) {
          lcdSerial.write(gpsLongitude[i]);
        }
        for (int i = 0; i < (strlen(gpsLongitudeDirection)); i++) {
          lcdSerial.write(gpsLongitudeDirection[i]);
        }
        for (int i = 0; i < (11 - strlen(gpsLongitude) - strlen(gpsLongitudeDirection)); i++) {
          lcdSerial.write(' ');
        }
        
        delay(5000);
    
        return 1;
        
      } else if (strcmp(gprsBuffer, "ERROR") == 0) {
    
        lcdSerial.write(254);
        lcdSerial.write(128);
        lcdSerial.print(F("Error sending   "));
        lcdSerial.print(F("coordinates.    "));
        
        delay(5000);
        
        return 0;
        
      } else {
    
        lcdSerial.write(254);
        lcdSerial.write(128);
        lcdSerial.print(F("Unknown GPRS    "));
        lcdSerial.print(F("response.       "));
        
        delay(5000);
        
        return 0;
        
      }
      
    } else {
    
        lcdSerial.write(254);
        lcdSerial.write(128);
        lcdSerial.print(F("No response     "));
        lcdSerial.print(F("from GPRS.      "));
        
        delay(5000);
        
        return 0;
        
    }
    
  } else if (millis() >= timeOut) {

    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("GPS search      "));
    lcdSerial.print(F("timed out.      "));
        
    delay(5000);
    
  } else {

    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("GPS search      "));
    lcdSerial.print(F("failed.         "));
        
    delay(5000);
  }
    
  return 0;
}

int clientCommands() {

  if (checkGPRSConnection()) {

    nextUpdate = millis() + updateInterval;
    
    timeOut = millis() + timeOutInterval;
    doCommandRECVCOORDS();
    //doCommandRECVJPG();
    //doCommandBYE();

    return 1;

  } else {
    return 0;
  }
}

int serverCommands() {

  char srvCommand[17];
  char messageEnc[65];

  while (true) {

    timeOut = millis() + timeOutInterval;
    loadGPRSCharArray(srvCommand, " \n");
  
    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("Doing command:  "));
    
    for (int i = 0; i < (strlen(srvCommand)); i++) {
      lcdSerial.write(srvCommand[i]);
    }
    for (int i = 0; i < (16 - strlen(srvCommand)); i++) {
      lcdSerial.write(' ');
    }

    if (strcmp(srvCommand, "HELLO") == 0) {
      doCommandHELLO();
    } else if (strcmp(srvCommand, "SENDCOORDS") == 0) {
      doCommandSENDCOORDS();
    } else if (strcmp(srvCommand, "CHALLENGE") == 0) {
      loadGPRSCharArray(messageEnc, " \n");
      doCommandCHALLENGE(messageEnc);
    } else if (strcmp(srvCommand, "CHALLENGEFAIL") == 0) {
      doCommandCHALLENGEFAIL();
    } else if (strcmp(srvCommand, "CHALLENGEPASS") == 0) {
      doCommandCHALLENGEPASS();
    } else if (strcmp(srvCommand, "OKCLIENT") == 0) {
      if (!clientCommands()) {
        break;
      }
    } else if (strcmp(srvCommand, "BYE") == 0) {
      break;
    } else {
      break;
    }
  }

  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("Disconnecting   "));
  lcdSerial.print(F("from server.    "));
  
  doCommandBYE();

  return 1;
}


int doGPRSConnect() {

  char gprsBuffer[65];

  char apnString[65];
  char apnCompare[65];
  char apnSignIn[65];
  
  timeOut = millis() + timeOutInterval;

  gprsSerial.flush();
  delay(1000);

  gprsSerial.println(F("AT"));
  delay(5000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "OK\r") != 0) {

    powerUpOrDown();

    delay(10000);

    while (gprsSerial.available()) {
      gprsSerial.read();
    }
    delay(1000);
  }

  //Serial.println(F("Ready to Connect . . ."));

  gprsSerial.println(F("AT+CGATT?"));
  delay(1000);
 
  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "+CGATT: 1\r") != 0) {
  
    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("Attaching to    "));
    lcdSerial.print(F("network . . .   "));

    printGPRSBuffer(gprsBuffer, 2);

    if (strcmp(gprsBuffer, "OK\r") != 0) {
      return 0;
    }

    gprsSerial.println(F("AT+CGATT=1"));
    delay(10000);

    printGPRSBuffer(gprsBuffer, 3);

    if (strcmp(gprsBuffer, "OK\r") != 0) {
      return 0;
    }

  } else {

    printGPRSBuffer(gprsBuffer, 2);

    if (strcmp(gprsBuffer, "OK\r") != 0) {
      return 0;
    }
  }

  gprsSerial.println(F("AT+CIPMUX?"));
  delay(1000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "+CIPMUX: 0\r") != 0) {
  
    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("Setting single  "));
    lcdSerial.print(F("user . . .      "));

    printGPRSBuffer(gprsBuffer, 2);

    if (strcmp(gprsBuffer, "OK\r") != 0) {
      return 0;
    }

    gprsSerial.println(F("AT+CIPMUX=0"));
    delay(10000);

    printGPRSBuffer(gprsBuffer, 3);

    if (strcmp(gprsBuffer, "OK\r") != 0) {
      return 0;
    }

  } else {

    printGPRSBuffer(gprsBuffer, 2);

    if (strcmp(gprsBuffer, "OK\r") != 0) {
      return 0;
    }
  }

  gprsSerial.println(F("AT+CSTT?"));
  delay(10000);

  printGPRSBuffer(gprsBuffer, 3);

  strcpy(apnString, "\"");
  strcat(apnString, apnServer);
  strcat(apnString, "\",\"");
  strcat(apnString, apnUserName);
  strcat(apnString, "\",\"");
  strcat(apnString, apnPassWord);
  strcat(apnString, "\"");

  strcpy(apnCompare, "+CSTT: ");
  strcat(apnCompare, apnString);
  strcat(apnCompare, "\r");

  strcpy(apnSignIn, "AT+CSTT=");
  strcat(apnSignIn, apnString);

  if (strcmp(gprsBuffer, apnCompare) != 0) {
  
    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("Setting         "));
    lcdSerial.print(F("APN . . .       "));

    gprsSerial.println(apnSignIn);
    delay(3000);

    printGPRSBuffer(gprsBuffer, 6);
    
  } else {
    printGPRSBuffer(gprsBuffer, 3);
  }

  if (strcmp(gprsBuffer, "OK\r") == 0) {}

  gprsSerial.println(F("AT+CIICR"));
  delay(3000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "ERROR\r") == 0) {}

  while (gprsSerial.available()) {
    gprsSerial.read();
  }
  delay(1000);

  gprsSerial.println(F("AT+CIFSR"));
  delay(3000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "ERROR\r") == 0) {
    powerUpOrDown();
  }
  
  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("IP Address:     "));
  
  for (int i = 0; i < (strlen(gprsBuffer) - 1); i++) {
    lcdSerial.write(gprsBuffer[i]);
  }
  for (int i = 0; i < (16 - strlen(gprsBuffer)); i++) {
    lcdSerial.write(' ');
  }

  gprsSerial.println(F("AT+CIPSPRT?"));
  delay(10000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "+CIPSPRT: 0\r") != 0) {

    printGPRSBuffer(gprsBuffer, 2);

    gprsSerial.println(F("AT+CIPSPRT=0"));
    delay(1000);

    printGPRSBuffer(gprsBuffer, 3);

    if (strcmp(gprsBuffer, "OK\r") != 0) {}
  } 
  else {
    printGPRSBuffer(gprsBuffer, 2);
  }
  
  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("Connecting to   "));
  lcdSerial.print(F("server . . .    "));

  gprsSerial.print(F("AT+CIPSTART=\"tcp\",\""));
  gprsSerial.print(webAddress);
  gprsSerial.print(F("\",\""));
  gprsSerial.print(portNumber);
  gprsSerial.println(F("\""));  // Start up the connection
  delay(5000);

  printGPRSBuffer(gprsBuffer, 3);

  if (strcmp(gprsBuffer, "OK\r") == 0) {

    while (gprsSerial.available() <= 0) {

      delay(1000);

      if (timeOut <= millis()) {
        break;
      }
    }
  }

  printGPRSBuffer(gprsBuffer, 2);

  if (strcmp(gprsBuffer, "STATE: TCP CLOSED\r") == 0) {

    gprsConnected = 0;
    printGPRSBuffer(gprsBuffer, 2);

    if (strcmp(gprsBuffer, "CONNECT FAIL\r") == 0) {
      return 0;
    }

  } else if (strcmp(gprsBuffer, "ALREADY CONNECT\r") == 0) {

    while (gprsSerial.available()) {
      Serial.write(gprsSerial.read());
    }
    delay(1000);

    gprsSerial.println(F("AT+CIPCLOSE"));
    delay(3000);
    gprsConnected = 0;

    printGPRSBuffer(gprsBuffer, 3);

    if (strcmp(gprsBuffer, "CLOSE OK\r") != 0) {} 
    else {}

    return 0;
  } else if (strcmp(gprsBuffer, "CONNECT OK\r") == 0) {
  
    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("Connected to    "));
    lcdSerial.print(F("server!!!       "));

    gprsConnected = 1;

    delay(10000);

    return 1;

  }
  
  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("Not connected.  "));
  lcdSerial.print(F("                "));

  return 0;
}

int checkGPRSConnection() {

  char gprsBuffer[65];

  gprsSerial.flush();
  delay(1000);

  gprsSerial.println(F("AT+CIPSTATUS"));
  delay(3000);

  printGPRSBuffer(gprsBuffer, 5);

  if (strcmp(gprsBuffer, "STATE: CONNECT OK\r") == 0) {
    gprsConnected = 1;
  } else {
    gprsConnected = 0;
  }

  return gprsConnected;
}

void setup() {

  delay(1000);  // give the Ethernet shield a second to initialize.

  pinMode(9, OUTPUT); // Set GPRS Reset pin to output
  
  nextUpdate = 0;
  
  lcdSerial.begin(LCD_SERIAL_BAUD);
  lcdSerial.flush();
  //lcdSerial.listen();
  
  lcdSerial.write(254);
  lcdSerial.write(128);
  lcdSerial.print(F("Starting        "));
  lcdSerial.print(F("Arduino . . .   "));
  
  /*
  gprsSerial.begin(GPRS_SERIAL_BAUD);
  while (!gprsSerial) {}
  
  while (true) {
   doCommandRECVCOORDS();
  }
  */
}

void loop() {

  if (nextUpdate <= millis()) {

    gprsConnected = 0;
    gprsSerial.begin(GPRS_SERIAL_BAUD);

    while (!gprsSerial) {}
    
    gprsSerial.flush();
    
    timeOut = millis() + timeOutInterval;

    if (doGPRSConnect()) {

      //checkGPRSConnection();

      while (gprsSerial) {

        if (gprsSerial.available() > 0) {
          serverCommands();
        }

        if (millis() >= timeOut) {
          doCommandBYE();
          break;
        }
      }

    } else {
      nextUpdate = millis() + 1000;
    }

    gprsSerial.end();
    gprsConnected = 0;
  
    lcdSerial.write(254);
    lcdSerial.write(128);
    lcdSerial.print(F("Waiting for next"));
    lcdSerial.print(F("update . . .    "));
  }
  
  delay(10000);
}


