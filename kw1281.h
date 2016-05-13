/*
  Arduino Nano OBD reader (OBD protocol KW1281,  Audi A4 B5 etc.)

  wiring:
  D2 --- OBD level shifter input (RX) (e.g. LM339)
  D3 --- OBD level shifter output (TX) (e.g. LM339)
  A5 --- Arduino 20x4 LCD display SCL
  A4 --- Arduino 20x4 LCD display SDA

  NOTE: For the level shifting, I used a 'AutoDia K409 Profi USB adapter', disassembled it,
      and connected the Arduino to the level shifter chip (LM339) - the original FTDI chip TX line
      was removed (so it does not influence the communication)
*/
#ifndef __KW1281_H__
#define __KW1281_H__

#define pinKLineTX 2
#define pinKLineRX 0

#include "Arduino.h"

// https://www.blafusel.de/obd/obd2_kw1281.html

#define ADR_Engine 0x01
#define ADR_Gears  0x02
#define ADR_ABS_Brakes 0x03
#define ADR_Airbag 0x15
#define ADR_Dashboard 0x17
#define ADR_Immobilizer 0x25
#define ADR_Central_locking 0x35

class Kw {
  public:
    bool status             = 1;
    uint8_t currAddr        = 0;
    uint8_t blockCounter    = 0;
    uint8_t errorTimeout    = 0;
    uint8_t errorData       = 0;
    bool connected          = false;
    int sensorCounter       = 0;
    uint8_t currPage        = 2;
    int8_t coolantTemp      = 0;
    int8_t oilTemp          ;
    int8_t intakeAirTemp    = 0;
    int8_t oilPressure      = 0;
    float engineLoad        = 0;
    int   engineSpeed       ;
    float throttleValve     = 0;
    float supplyVoltage     = 0;
    uint8_t vehicleSpeed    ;
    uint8_t fuelConsumption1 ;
    uint8_t fuelConsumption2;
    uint8_t fuelConsumption ;
    uint8_t fuelLevel       = 0;
    unsigned long odometer  = 0;
    // *************************************** METODI ******************************************************************
    void disconnect() ;
    void obdWrite(uint8_t data);
    uint8_t obdRead();
    void send5baud(uint8_t data) ;
    bool KWP5BaudInit(uint8_t addr);
    bool KWPSendBlock(char *s, int sizeb) ;
    bool KWPReceiveBlock(char s[], int maxsize, int &sizeb) ;
    bool KWPSendAckBlock();
    bool connect(uint8_t addr, int baudrate) ;
    bool readConnectBlocks();
    bool readSensors(int group);
    
};

#endif




