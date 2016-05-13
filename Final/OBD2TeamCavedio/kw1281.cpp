
#include "kw1281.h"


void Kw::disconnect (){
  Serial.end();
  connected = false;
  currAddr = 0;
}

void Kw::obdWrite(uint8_t data) {
  delay(5);
  Serial.write(data);                // sendChar() 28.04.2016 20:35
  delayMicroseconds(1300);            // Uart byte Tx duration at 9600 baud ~= 1040 us
  uint8_t dummy = Serial.read();     // Read Echo from the line
  if (dummy != data ) {
    return;   // compare received and transmitted char
  }
}

uint8_t Kw::obdRead() {
  unsigned long timeout = millis() + 1000;
  // Check if serial reply
  while (!Serial.available()) {
    if (millis() >= timeout) {
      disconnect();
      errorTimeout++;
      return 0;
    }
  }
  uint8_t data = Serial.read();
  // Discard if recived OxFF
  while (data == 0xff) {
    data = Serial.read();
  }
  return data;
}

void Kw::send5baud(uint8_t data) {
  // // 1 start bit, 7 data bits, 1 parity, 1 stop bit
  #define bitcount 10
  byte bits[bitcount];
  byte even = 1;
  byte bit;

  // GPIO
  pinMode(pinKLineTX, OUTPUT);
  digitalWrite(pinKLineTX, LOW);
  delay(500);
  
  // Buil the 10 bit connection array
  for (int i = 0; i < bitcount; i++) {
    bit = 0;
    if (i == 0)  bit = 0;
    else if (i == 8) bit = even; // computes parity bit
    else if (i == 9) bit = 1;
    else {
      bit = (byte) ((data & (1 << (i - 1))) != 0);
      even = even ^ bit;
    }
    bits[i] = bit;
  }
  
  // now send bit stream
  for (int i = 0; i < bitcount + 1; i++) {
    if (i != 0) {
      // wait 200 ms (=5 baud), adjusted by latency correction
      delay(200);
      if (i == bitcount) break;
    }
    if (bits[i] == 1) {
      // high
      digitalWrite(pinKLineTX, LOW);
    } else {
      // low
      digitalWrite(pinKLineTX, HIGH);
    }
  }
  digitalWrite(pinKLineTX, LOW);
}

bool Kw::KWP5BaudInit(uint8_t addr) {
  send5baud(addr);
  return true;
}

bool Kw::KWPSendBlock(char *s, int sizeb) {
  for (int i = 0; i < sizeb; i++) {
    uint8_t data = s[i];
    obdWrite(data);
    if (i < sizeb - 1) {
      uint8_t complement = obdRead();
      if (complement != (data ^ 0xFF)) {
        disconnect();
        errorData++;
        return false;
      }
    }
  }
  blockCounter++;
  return true;
}

bool Kw::KWPReceiveBlock(char s[], int maxsize, int &sizeb) {
  bool ackeachbyte = false;
  uint8_t data = 0;
  // Counter of data in the block
  int recvcount = 0;
  
  // If sizeb == 0 the size of the block will sent from the car
  if (sizeb == 0) ackeachbyte = true;
  
  if (sizeb > maxsize) {
    return false;
  }
  
  unsigned long timeout = millis() + 1000;
  
  while ((recvcount == 0) || (recvcount != sizeb)) {
    while (Serial.available()) {
      // Read data from the car
      data = obdRead();
      // Put in the array
      s[recvcount] = data;
      // increase the counter
      recvcount++;

      // If sizeb == 0 the size of the block will sent from the car as first byte 
      if ((sizeb == 0) && (recvcount == 1)) {
        sizeb = data + 1; // Plus one because the lenght sent from the car don't count the block lenght byte
        if (sizeb > maxsize) {
          return false;
        }
      }
      if ((ackeachbyte) && (recvcount == 2)) {
        // The second byte of the block must be the block counter
        if (data != blockCounter) {
          disconnect();
          errorData++;
          return false;
        }
      }
      // Send the complement
      if (((!ackeachbyte) && (recvcount == sizeb)) ||  ((ackeachbyte) && (recvcount < sizeb))) {
        obdWrite(data ^ 0xFF);  // send complement ack
        if (maxsize == 3) {
          break;
        }
      }
      timeout = millis() + 1000;
    }
    if (millis() >= timeout) {
      disconnect();
      errorTimeout++;
      return false;
    }
  } 
  // Increment the block counter
  blockCounter++;
  return true;
}

bool Kw::KWPSendAckBlock() {
  // Send the Ack Block
  char buf[32];
  sprintf(buf, "\x03%c\x09\x03", blockCounter);
  return (KWPSendBlock(buf, 4));
}

bool Kw::connect(uint8_t addr, int baudrate) {
  blockCounter = 0;
  currAddr = 0;

  // Connection at 5 bits per second
  KWP5BaudInit(addr);
  
  // Start the serial
  Serial.begin(9600);
  // Empty the Serial
  while (Serial.available() > 0){
    Serial.read();
  }
  
  // Used only for Arduino2 (GPIO -> SERIAL)
  //PIOD->PIO_PDR = 0x10;
  //Serial.begin(baudrate);
  //while (!Serial);
  
  //Answer: 0x55, 0x01, 0x8A
  char s[3];
  int sizeb = 3;
  if (!KWPReceiveBlock(s, 3, sizeb)) {
    return false;
  }
  if ( (((uint8_t)s[0]) != 0x55) || (((uint8_t)s[1]) != 0x01) || (((uint8_t)s[2]) != 0x8A) ) {
    disconnect();
    errorData++;
    return false;
  }
  currAddr = addr;
  connected = true;
  if (!readConnectBlocks()) return false;
  return true;
}

bool Kw::readConnectBlocks() {
  // read connect blocks
  // and addittional block
  String info;
  while (true) {
    int sizeb = 0;
    char s[64];
    if (!(KWPReceiveBlock(s, 64, sizeb))) return false;
    if (sizeb == 0) return false;
    // Exit from the loop when the car send an ack
    if (s[2] == '\x09') break;
    // Error if in not an ASCII block
    if (s[2] != '\xF6') {
      disconnect();
      errorData++;
      return false;
    }
    // Print the connection block
    //String text = String(s);
    //info += text.substring(3, sizeb - 2);

    // Send the ACK
    if (!KWPSendAckBlock()) return false;
  }
  return true;
}

bool Kw::readSensors(int group) {
  char s[64];
  // Send the request
  sprintf(s, "\x04%c\x29%c\x03", blockCounter, group);
  if (!KWPSendBlock(s, 5)) return false;
  
  int sizeb = 0;
  // Read the reply 
  KWPReceiveBlock(s, 64, sizeb);
  // Error if in not a Sensor 
  if (s[2] != '\xe7') {
    disconnect();
    errorData++;
    return false;
  }
  
  float v = 0;
  // For every group there is 4 sensors
  int count = (sizeb - 4) / 3;
  for (int idx = 0; idx < count; idx++) {
    // Calculate the sensor value
    byte k = s[3 + idx * 3];
    byte a = s[3 + idx * 3 + 1];
    byte b = s[3 + idx * 3 + 2];
    String n;
    String t = "";
    char buf[32];
    switch (k) {
      case 1:  v = 0.2 * a * b; engineSpeed = v;          break; // Engine Speed - RPM
      case 2:  v = a * 0.002 * b;                         break;
      case 3:  v = 0.002 * a * b;                         break; // Throttle Position - Deg
      case 4:  v = abs(b - 127) * 0.01 * a;               break; // ATDC
      case 5:  v = a * (b - 100) * 0.1;  oilTemp=v;       break; // Oil Temperature -°C
      case 6:  v = 0.001 * a * b;                         break; // V
      case 7:  v = 0.01 * a * b;   vehicleSpeed = v;      break; // Vehicle Speed - km/h
      case 8:  v = 0.1 * a * b;                           break; //
      case 9:  v = (b - 127) * 0.02 * a;                  break; // Deg
      case 10: if (b == 0) t = "COLD"; else t = "WARM";   break; // 
      case 11: v = 0.0001 * a * (b - 128) + 1;            break; // 
      case 12: v = 0.001 * a * b;                         break; // Ohm
      case 13: v = (b - 127) * 0.001 * a;                 break; // mm
      case 14: v = 0.005 * a * b;                         break; // bar
      case 15: v = 0.01 * a * b;                          break; // ms
      case 18: v = 0.04 * a * b;                          break; // mbar
      case 19: v = a * b * 0.01;                          break; // l
      case 20: v = a * (b - 128) / 128;                   break; // %%
      case 21: v = 0.001 * a * b;                         break; // V
      case 22: v = 0.001 * a * b;                         break; // ms
      case 23: v = b / 256 * a;                           break; // %%
      case 24: v = 0.001 * a * b;                         break; // A
      case 25: v = (b * 1.421) + (a / 182);               break; // g/s
      case 26: v = float(b - a);                          break; // C
      case 27: v = abs(b - 128) * 0.01 * a;               break; // °
      case 28: v = float(b - a);                          break; // " "
      case 30: v = b / 12 * a;                            break; // Deg k/w
      case 31: v = b / 2560 * a;                          break; // °C
      case 33: v = 100 * b / a;                           break; //%%
      case 34: v = (b - 128) * 0.01 * a;                  break; // kW
      case 35: v =  a * b;     fuelConsumption = v;       break; // fuelConsumption  - l/h
      case 36: v = ((unsigned long)a) * 2560 + ((unsigned long)b) * 10;  break; // km
      case 37: v = b;                                     break; // oil pressure ??
      case 38: v = (b - 128) * 0.001 * a;                 break; // "Deg k/w
      case 39: v = b / 256 * a;                           break; // mg/h
      case 40: v = b * 0.1 + (25.5 * a) - 400;            break; // A
      case 41: v = b + a * 255;                           break; // Ah
      case 42: v = b * 0.1 + (25.5 * a) - 400;            break; // Kw
      case 43: v = b * 0.1 + (25.5 * a);                  break; // V
      case 44: sprintf(buf, "%2d:%2d", a, b); t = String(buf); break;
      case 45: v = 0.1 * a * b / 100;                     break; // " "
      case 46: v = (a * b - 3200) * 0.0027;               break; // Deg k/w
      case 47: v = (b - 128) * a;                         break; // ms
      case 48: v = b + a * 255;                           break; // " "
      case 49: v = (b / 4) * a * 0.1;                     break; // mg/h
      case 50: v = (b - 128) / (0.01 * a);                break; // mbar
      case 51: v = ((b - 128) / 255) * a;                 break; // mg/h
      case 52: v = b * 0.02 * a - a;                      break; // Nm
      case 53: v = (b - 128) * 1.4222 + 0.006 * a;        break; // g/s
      case 54: v = a * 256 + b;                           break; // count
      case 55: v = a * b / 200;                           break; // a
      case 56: v = a * 256 + b;                           break; // WSC
      case 57: v = a * 256 + b + 65536;                   break; // WSC
      case 59: v = (a * 256 + b) / 32768;                 break; // g/s
      case 60: v = (a * 256 + b) * 0.01;                  break; // sec
      case 62: v = 0.256 * a * b;                         break; // S
      case 64: v = float(a + b);                          break; // Ohm
      case 65: v =0.01 * a * (b - 127);                   break; // mm
      case 66: v = (a * b) / 511.12;                      break; // V
      case 67: v = (640 * a) + b * 2.5;                   break; // Deg
      case 68: v = (256 * a + b) / 7.365;                 break; // deg/s
      case 69: v = (256 * a + b) * 0.3254;                break; // bar
      case 70: v = (256 * a + b) * 0.192;                 break; // m/s^2
      default:                                            break;
    } 
  }
  sensorCounter++;
  return true;
}



