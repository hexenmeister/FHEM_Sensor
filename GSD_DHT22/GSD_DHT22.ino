//----------------------------------------------------------------------------------------------------------------------
// TinyTX - An ATtiny84 and RFM12B Wireless Temperature & Humidity Sensor Node
// By Nathan Chantrell. For hardware design see http://nathan.chantrell.net/tinytx
//
// Using the DHT22 temperature and humidity sensor
//
// Licenced under the Creative Commons Attribution-ShareAlike 3.0 Unported (CC BY-SA 3.0) licence:
// http://creativecommons.org/licenses/by-sa/3.0/
//
// Requires Arduino IDE with arduino-tiny core: http://code.google.com/p/arduino-tiny/
//----------------------------------------------------------------------------------------------------------------------

#include <DHT22.h> // https://github.com/nathanchantrell/Arduino-DHT22
#include <JeeLib.h> // https://github.com/jcw/jeelib

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // interrupt handler for JeeLabs Sleepy power saving

#define myNodeID 01       // RF12 node ID in the range 1-30
#define network 210       // RF12 Network group
#define myUID 01          // device id

#define freq RF12_868MHZ  // Frequency of RFM12B module

#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 10       // Number of milliseconds to wait for an ack

#define DHT22_PIN 10      // DHT sensor is connected on D10/ATtiny pin 13
#define DHT22_POWER 9     // DHT Power pin is connected on D9/ATtiny pin 12
#define LED_POWER 0       // Status-LED-Pin 

#define magicNumber 83    // magic number (GSD-Net identifier)

DHT22 myDHT22(DHT22_PIN); // Setup the DHT

//########################################################################################################################
//Data Structure to be sent
//########################################################################################################################
// Payload: magic, uID [, SensorType [, SensorData]* ]*
// <SensorType>: 0-9 -> Reserved/not Used
// <SensorType>: 10-99 -> Default
// <SensorType>: 100-199 -> Userdifined
// <SensorType>: 200-255 -> Internal/Test

 typedef struct {
   byte magic;   // magic number
   byte uID;     // deviceID
   
   int msg_counter;
          
   byte m_powerSupply; // 252 => 202
   int powerSupply;  // Supply voltage
   byte m_Temperature; // 11 => 16
   int temperature;  //Temperature reading
   byte m_Humidity;    // 16 => 24
   int humidity;     // Humidity reading
 } Payload;

 Payload tinytx;


// Wait a few milliseconds for proper ACK
 #ifdef USE_ACK
  static byte waitForAck() {
   MilliTimer ackTimer;
   while (!ackTimer.poll(ACK_TIME)) {
     if (rf12_recvDone() && rf12_crc == 0 &&
        rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID)) {
          if ((rf12_len > 0 && rf12_data[0] == myUID) || (rf12_len == 0)) {
            return 1;
          }
        }
        //set_sleep_mode(SLEEP_MODE_IDLE);
        //sleep_mode();
     }
   return 0;
  }
 #endif

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//-------------------------------------------------------------------------------------------------
 static byte rfwrite(){
  #ifdef USE_ACK
   for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
     rf12_sleep(-1);              // Wake up RF module
      while (!rf12_canSend())
      rf12_recvDone();
      rf12_sendStart(RF12_HDR_ACK, &tinytx, sizeof tinytx); 
      rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
      byte acked = waitForAck();  // Wait for ACK
      rf12_sleep(0);              // Put RF module to sleep
      if (acked) { return 1; }      // Return if ACK received
  
   Sleepy::loseSomeTime(RETRY_PERIOD * 1000);     // If no ack received wait and try again
   }
   return 0;
  #else
     rf12_sleep(-1);              // Wake up RF module
     while (!rf12_canSend())
     rf12_recvDone();
     rf12_sendStart(0, &tinytx, sizeof tinytx); 
     rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
     rf12_sleep(0);              // Put RF module to sleep
     return 1;
  #endif
 }



//--------------------------------------------------------------------------------------------------
// Read current supply voltage
//--------------------------------------------------------------------------------------------------
 long readVcc() {
   bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
   long result;
   // Read 1.1V reference against Vcc
   #if defined(__AVR_ATtiny84__) 
    ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
   #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
   #endif 
   delay(2); // Wait for Vref to settle
   ADCSRA |= _BV(ADSC); // Convert
   while (bit_is_set(ADCSRA,ADSC));
   result = ADCL;
   result |= ADCH<<8;
   result = 1126400L / result; // Back-calculate Vcc in mV
   ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
   return result;
} 

//########################################################################################################################

void setup() {
  // initialize static structure parts
 tinytx.magic = magicNumber;
 tinytx.uID   = myUID;
 tinytx.m_powerSupply = 202; // TODO: constants
 tinytx.m_Temperature = 16;
 tinytx.m_Humidity    = 24;
 
 tinytx.msg_counter = 0;

  rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
  rf12_sleep(0);                          // Put the RFM12 to sleep

  pinMode(DHT22_POWER, OUTPUT); // set power pin for DHT to output
  pinMode(LED_POWER, OUTPUT);
  
  PRR = bit(PRTIM1); // only keep timer 0 going
  
  ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
  
}

void loop() {
  
  digitalWrite(DHT22_POWER, HIGH); // turn DHT sensor on
  digitalWrite(LED_POWER, HIGH); // turn LED sensor on
  
  DHT22_ERROR_t errorCode;
  
  Sleepy::loseSomeTime(2000); // Sensor requires minimum 2s warm-up after power-on.
  
  errorCode = myDHT22.readData(); // read data from sensor

  if (errorCode == DHT_ERROR_NONE) { // data is good
    tinytx.msg_counter++;
    tinytx.temperature = (myDHT22.getTemperatureC()*100); // Get temperature reading and convert to integer, reversed at receiving end
    tinytx.humidity = (myDHT22.getHumidity()*100); // Get humidity reading and convert to integer, reversed at receiving end
    tinytx.powerSupply = readVcc(); // Get supply voltage
    rfwrite(); // Send data via RF 
  }

  digitalWrite(LED_POWER, LOW); // turn LED off
  digitalWrite(DHT22_POWER, LOW); // turn DS18B20 off
  
  Sleepy::loseSomeTime(60000); //JeeLabs power save function: enter low power mode for 60 seconds (valid range 16-65000 ms)
    
}


