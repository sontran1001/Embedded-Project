/*
 * 
 * 
 */

#include "kw1281.h"
/* ------- BLE -------*/
#include <BLE_API.h>
#define TXRX_BUF_LEN                      20            // Buffer Lenght BLE
BLE                                       ble;          // Bluetooth LE class
Timeout                                   timeout;      // Timeout class

static uint8_t rx_buf[TXRX_BUF_LEN];   // Array for Buffer character
static uint8_t rx_buf_num;             // Position on the buffer
static uint8_t rx_state = 0;           // State = 1 RX, State = 0 TX

// The Nordic UART Service
static const uint8_t service1_uuid[]                = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_tx_uuid[]             = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t service1_rx_uuid[]             = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[]           = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};

uint8_t tx_value[TXRX_BUF_LEN] = {0,}; //
uint8_t rx_value[TXRX_BUF_LEN] = {0,}; //

GattCharacteristic  characteristic1(service1_tx_uuid, tx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  characteristic2(service1_rx_uuid, rx_value, 1, TXRX_BUF_LEN, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);

GattCharacteristic *uartChars[] = {&characteristic1, &characteristic2};

GattService         uartService(service1_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic *));

void disconnectionCallBack(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
  ble.startAdvertising();
}

void connectionCallBack(const Gap::ConnectionCallbackParams_t *param)
{
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (const uint8_t *)"1", 1);
}

/* ------- RICEZIONE -------*/
// -----> Non nel nostro caso <------
/*
void writtenHandle(const GattWriteCallbackParams *Handler)
{
  uint8_t buf[TXRX_BUF_LEN];
  uint16_t bytesRead, index;

  Serial.println("onDataWritten : ");
  Serial.println("Dati ricevuti : ");
  if (Handler->handle == characteristic1.getValueAttribute().getHandle()) {
    ble.readCharacteristicValue(characteristic1.getValueAttribute().getHandle(), buf, &bytesRead);
    Serial.print("bytesRead: ");
    Serial.println(bytesRead, HEX);
    for (byte index = 0; index < bytesRead; index++) {
      Serial.write(buf[index]);
    }
    Serial.println("");
  }
}


void m_uart_rx_handle()
{ //update characteristic data
  ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), rx_buf, rx_buf_num);
  memset(rx_buf, 0x00, 20);
  rx_state = 0;
}
*/
/* ------- /RICEZIONE -------*/
/* ------- /BLE -------*/

Kw kwp;

void setup() {
  Serial.begin(9600);
  while (!Serial) ;
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  /* ------- BLE -------*/
  

  ble.init();
  ble.onDisconnection(disconnectionCallBack);
  ble.onConnection(connectionCallBack);
  // Ricezione
  //ble.onDataWritten(writtenHandle);
  //Serial.attach(uart_handle); // set an interrupt for the serial
  
  // setup adv_data and srp_data
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED); // Only BLE
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)"TXRX", sizeof("TXRX") - 1); // Shortened Local Name
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                   (const uint8_t *)uart_base_uuid_rev, sizeof(uart_base_uuid_rev)); // UIDD Service

  // set adv_type
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
  // add service
  ble.addService(uartService);
  // set device name
  ble.setDeviceName((const uint8_t *)"OBD Team Cavedio");
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(160);
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();
/* ------- /BLE -------*/
}

void loop() {
  ble.waitForEvent();
  unsigned char buf[20];
  char buf2[20];
  uint8_t le ,b,c;
  String a ,valore;
  
  if (kwp.currAddr != ADR_Engine) {
    kwp.status=1;
    while (Serial.available() > 0){
      Serial.read();
    } 
    digitalWrite(13,kwp.status);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t*)"ATry to connect", 15);
    kwp.connect(ADR_Engine, 9600);
  } else {
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), (uint8_t*)"ACONNECTED", 10);
    kwp.readSensors(1); //1 -> RPM 
    a = String("RPM: ") + String(kwp.engineSpeed,DEC) ;
    le = a.length()+1;
    a.getBytes(buf,le);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, le);
    a = String("OIL: ") + String(kwp.oilTemp,DEC) ;
    le = a.length()+1;
    a.getBytes(buf,le);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, le);
    kwp.readSensors(15); // SPEED
    a = String("CONS: ") + String(kwp.fuelConsumption, DEC);
    le = a.length()+1;
    a.getBytes(buf,le);
    ble.updateCharacteristicValue(characteristic2.getValueAttribute().getHandle(), buf, le);

    //readSensors(6); // 7 -> Velocità
  }

}
