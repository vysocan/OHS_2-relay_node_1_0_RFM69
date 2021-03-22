/*
 * Open Home Security - Relay node with RS485, Power AC 80~250V
 * Temperature(A7) - under power supply.
 * Temperature(A6) - not populated.
 * Relay 1(D9), 2(D8).
 * 
 * Board v.1.00
 */
// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9)  PWM
//                  +----+

#include <SPI.h>
#include <RFM69.h>
#include <RFM69_ATC.h>
#include <avr/eeprom.h> // Global configuration for in chip EEPROM

// Node settings
#define NODEID          5    // This is our address 
#define VERSION         100  // Version of EEPROM struct
#define PING_DELAY      1200000 // In milliseconds, 20 minutes
#define SENSOR_DELAY    600000  // In milliseconds, 10 minutes
// Constants
#define REG_LEN         21   // Size of one conf. element
#define NODE_NAME_SIZE  16   // As defined in gateway
#define ALARM_ZONES     9    // Fixed number by hardware
// Pins
#define RELAY_1         9
#define RELAY_2         8
// Radio
#define NETWORKID       100  // Do not change, defined in gateway
#define GATEWAYID       1    // Do not change gateway address
#define RADIO_REPEAT    5    // Repeat sending
#define FREQUENCY       RF69_868MHZ // Match this with the version of your gateway (others: RF69_433MHZ, RF69_915MHZ)
#define KEY             "ABCDABCDABCDABCD" // Has to be same 16 characters/bytes on all nodes, not more not less!
//#define ENABLE_ATC      // Comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI        -75

#ifdef ENABLE_ATC
  RFM69_ATC radio;
#else
  RFM69 radio;
#endif


// Global variables
int8_t  res;
uint8_t radioLength;
uint8_t pos;
uint8_t msg[REG_LEN+1];
//uint8_t msg[25]; // We send sensor msg longer than reg. element size
unsigned long sensorMillis;
unsigned long aliveMillis;

// Configuration struct
struct config_t {
  uint16_t version;
  char     reg[REG_LEN * 3]; // Number of elements on this node
} conf; 

// Float conversion 
union u_tag {
  byte  b[4]; 
  float fval;
} u;

/* 
 * Registration 
 */
void sendConf(){
  int8_t result;
  uint16_t count = 0;

  // Wait some time to avoid contention
  delay(NODEID * 1000);   

  Serial.print(F("Conf:"));

  while (count < sizeof(conf.reg)) {
    msg[0] = 'R'; // Registration flag
    memcpy(&msg[1], &conf.reg[count], REG_LEN);    
    result = radio.sendWithRetry(GATEWAYID, msg, REG_LEN + 1, RADIO_REPEAT);
    Serial.print(F(" ")); Serial.print(result);
    count += REG_LEN;
  }  
  
  Serial.println(F("."));
}
// Set defaults on first time
void setDefault(){
  conf.version = VERSION;   // Change VERSION to force EEPROM load
  conf.reg[0+(REG_LEN*0)] = 'S';       // Sensor
  conf.reg[1+(REG_LEN*0)] = 'T';       // Temperature
  conf.reg[2+(REG_LEN*0)] = 0;         // Local address
  conf.reg[3+(REG_LEN*0)] = B00000000; // Default setting
  conf.reg[4+(REG_LEN*0)] = B00011110; // Default setting, group=16, disabled
  memset(&conf.reg[5+(REG_LEN*0)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*1)] = 'I';       // Input
  conf.reg[1+(REG_LEN*1)] = 'D';       // Digital
  conf.reg[2+(REG_LEN*1)] = 1;         // Local address
  conf.reg[3+(REG_LEN*1)] = B00000000; // Default setting
  conf.reg[4+(REG_LEN*1)] = B00011110; // Default setting, group=16, disabled
  memset(&conf.reg[5+(REG_LEN*1)], 0, NODE_NAME_SIZE);
  conf.reg[0+(REG_LEN*2)] = 'I';       // Input
  conf.reg[1+(REG_LEN*2)] = 'D';       // Digital
  conf.reg[2+(REG_LEN*2)] = 2;         // Local address
  conf.reg[3+(REG_LEN*2)] = B00000000; // Default setting
  conf.reg[4+(REG_LEN*2)] = B00011110; // Default setting, group=16, disabled
  memset(&conf.reg[5+(REG_LEN*2)], 0, NODE_NAME_SIZE);
}
/*
 * Send ping command to gateway 
 */
void sendPing(void) {
  msg[0] = 'C'; // Command
  msg[1] = 2;   // PING = 2
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 2);
}
/*
 * Process incoming radio data
 */
void checkRadio(){
  // Look for incomming transmissions
  if (radio.receiveDone()) {
    radioLength = radio.DATALEN; 
    if (radio.ACKRequested()) { 
      delay(5); // wait after receive, we need this delay or gateway will not see ACK!!!
      radio.sendACK();
      Serial.print(F("ACK:"));
    }
    //for (uint8_t ii=0; ii < radioLength; ii++){ Serial.print((char)radio.DATA[ii], HEX); Serial.print("-"); }; Serial.println(F("<"));
    if ((char)radio.DATA[0] == 'C') {
      Serial.print(F("C:")); Serial.println(radio.DATA[1]);
      // Commands from gateway
      switch (radio.DATA[1]) {
        case 1: // Request for registration
          sendConf(); 
          break;
        default: break;
      }
    }
    if ((char)radio.DATA[0] == 'R') { // Registration
      Serial.print(F("R:"));
      // Replace part of conf string with new paramters.
      pos = 0; 
      while (((conf.reg[pos] != radio.DATA[1]) || (conf.reg[pos+1] != radio.DATA[2]) ||
              (conf.reg[pos+2] != radio.DATA[3])) && (pos < sizeof(conf.reg))) {
        pos += REG_LEN; // size of one conf. element
      }
      if (pos < sizeof(conf.reg)) {
        Serial.println(pos/REG_LEN); // Show # of updated element       
        memcpy(&conf.reg[pos], &radio.DATA[1], REG_LEN);
        // Save it to EEPROM
        conf.version = VERSION;
        eeprom_update_block((const void*)&conf, (void*)0, sizeof(conf)); // Save current configuration              
      }
    }
    // Data to relays
    if ((char)radio.DATA[0] == 'I') { // Input
      Serial.print(F("I:"));
      Serial.print((byte)radio.DATA[1]); 
      Serial.print(F("="));
      u.b[0] = radio.DATA[2]; u.b[1] = radio.DATA[3]; u.b[2] = radio.DATA[4]; u.b[3] = radio.DATA[5]; 
      Serial.print((byte)u.fval);
      Serial.println();
      switch((byte)radio.DATA[1]){
        case 1: digitalWrite(RELAY_1, (byte)u.fval); break;
        case 2: digitalWrite(RELAY_2, (byte)u.fval); break;
      }
    }
  }
}
/*
 * Send float value of one element to gateway 
 */
void sendValue(uint8_t element, float value) {
  u.fval = value; 
  msg[0] = conf.reg[(REG_LEN*element)];
  msg[1] = conf.reg[1+(REG_LEN*element)];
  msg[2] = conf.reg[2+(REG_LEN*element)];
  msg[3] = u.b[0]; msg[4] = u.b[1]; msg[5] = u.b[2]; msg[6] = u.b[3];
  // Send to GW 
  radio.sendWithRetry(GATEWAYID, msg, 7);
}
/*
 * Setup
 */
void setup() {
  // Set pins
  pinMode(RELAY_1, OUTPUT);    // set pin 
  pinMode(RELAY_2, OUTPUT);    // set pin 
  // RFM69
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  radio.setHighPower();  // uncomment only for RFM69HW!
  // radio.encrypt(KEY); // uncomment if you use encryption
  #ifdef ENABLE_ATC
    radio.enableAutoPower(ATC_RSSI);
  #endif
 
  Serial.begin(115200); 
  Serial.println(F("Start"));

  eeprom_read_block((void*)&conf, (void*)0, sizeof(conf)); // Read current configuration
  if (conf.version != VERSION) setDefault();

  sendConf();

  sensorMillis  = millis();
  aliveMillis = millis();

  // Let's start
  delay(1000);
}
/*
 * Main loop
 */
void loop() {
  // Process radio data
  checkRadio();

  // Send alive packet every SENSOR_DELAY
  if ((unsigned long)(millis() - sensorMillis) > SENSOR_DELAY){
    sensorMillis = millis();    
    sendValue(0, (((float)analogRead(A7) * 0.003223)-0.5)*100); 
  }

  // Send alive packet every PING_DELAY
  if ((unsigned long)(millis() - aliveMillis) > PING_DELAY){
    aliveMillis = millis();
    sendPing();
  }
}
