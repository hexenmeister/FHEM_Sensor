#include <JeeLib.h> // https://github.com/jcw/jeelib
//#include <RF12.h>
#include <Ports.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <util/delay.h>
//#include <stdlib.h>

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // interrupt handler for JeeLabs Sleepy power saving

#define myNodeID 01       // RF12 node ID in the range 1-30
#define network 210       // RF12 Network group
#define myUID 01          // device id

#define freq RF12_868MHZ  // Frequency of RFM12B module

#define USE_ACK           // Enable ACKs, comment out to disable
#define RETRY_PERIOD 3    // How soon to retry (in seconds) if ACK didn't come in
#define RETRY_LIMIT 5     // Maximum number of times to retry
#define ACK_TIME 22       // Number of milliseconds to wait for an ack


#define magicNumber 83    // magic number (GSD-Net identifier) => SubNodeID 1 Byte
//#define magicNumber 84    // magic number (GSD-Net identifier)  => SubNodeID 2 Byte

//#define SERIAL_DEBUG

// for troubleshooting
#ifdef SERIAL_DEBUG
#include <TinyDebugSerial.h>
TinyDebugSerial mySerial = TinyDebugSerial();
#endif

#define USE_EEPROM
#define USE_BAT_MEASURE

#define USE_SENSOR_BH1750
#define USE_SENSOR_BMP085
#define USE_SENSOR_DHT22

#define USE_LED

#ifdef USE_LED
#define LED_POWER 0       // Status-LED-Pin 
#endif

#ifdef USE_EEPROM
#include <avr/eeprom.h>
#endif

#if defined(USE_SENSOR_BH1750)  || defined(USE_SENSOR_BMP085)
 PortI2C i2c (2);         // SDA to D8 and SCL to D7
#endif

#if defined(USE_SENSOR_BH1750)
 //#include <AS_PortsBH1750.h>
 //AS_BH1750 lightMeter (i2c);
 #include <Test_PortsBH1750.h>
 BH1750 lightMeter (i2c);
#endif

#if defined(USE_SENSOR_BMP085)
 #include <PortsBMP085.h> // Part of JeeLib
 BMP085 baroMeter (i2c, 3); // ultra high resolution
#endif

#ifdef USE_SENSOR_DHT22
#include <DHT22.h> // https://github.com/nathanchantrell/Arduino-DHT22
#define DHT22_PIN 10      // DHT sensor is connected on D10/ATtiny pin 13
#define DHT22_POWER 9     // DHT Power pin is connected on D9/ATtiny pin 12
DHT22 myDHT22(DHT22_PIN); // Setup the DHT
#endif

// Payload
typedef struct {
   byte magic;   // magic number
   #if magicNumber==83
   byte uID;     // deviceID
   #else
   int uID;      // deviceID
   #endif
   int msg_counter;

   #if defined USE_BAT_MEASURE
   byte m_powerSupply; // 202
   int powerSupply;    // Supply voltage
   #endif
   
   #if defined USE_SENSOR_BMP085
   byte m_Pressure;    // 48
   int pressure;       // Pressure reading
   #endif
   
   #if defined USE_SENSOR_DHT22 || defined USE_SENSOR_BMP085
   byte m_Temperature; // 16
   int temperature;    // Temperature reading
   #endif
   
   #if defined USE_SENSOR_DHT22
   byte m_Humidity;    // 24
   int humidity;       // Humidity reading
   #endif
   
   #if defined USE_SENSOR_BH1750
   byte m_Brightness; // 32
   int brightness;    // Brightness reading
   #endif
   
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

#define DATA_RATE_49200  0xC606           // Approx  49200 bps
#define DATA_RATE_38400  0xC608           //  Approx  38400 bps
#define DATA_RATE        DATA_RATE_38400  // use one of the data rates defined above

#ifdef USE_LED
int led = 3;
#endif

void setup() {
   tinytx.magic = magicNumber;
   
  // Ohne Empfänger eig. nicht sinnvoll
  rf12_initialize(myNodeID,freq,network); // Initialize RFM12 with settings defined above 
  //Default: rf12_xfer(0xC606); // approx 49.2 Kbps, i.e. 10000/29/(1+6) Kbps
  //rf12_xfer(0xC608); // Approx  38.4Kbps
  //rf12_xfer(DATA_RATE); // DATA RATE
  rf12_control(DATA_RATE); // DATA RATE
  rf12_sleep(0);                          // Put the RFM12 to sleep

  // --- hier ist ein guter Platz für eigene Initialisierungen
  #ifdef SERIAL_DEBUG
  // Serial-DEBUG (Send only)
   mySerial.begin( 9600 );
   mySerial.print("Serial initialized\r\n");
  #endif
  #ifdef USE_LED
   pinMode(led, OUTPUT);
  #endif
  
   #if defined(USE_SENSOR_BH1750)
   #ifdef SERIAL_DEBUG
   mySerial.print("Initialize BH1750...");
   if(!lightMeter.begin()){
     // Error  
     mySerial.print(" error\r\n");
   }
   #endif
   #ifndef SERIAL_DEBUG
   lightMeter.begin();
   #endif
   #endif
   
   #if defined(USE_SENSOR_BMP085)
   baroMeter.getCalibData();
   #endif
  
  // ---
  // EEPROM Test
  #ifdef USE_EEPROM
   #ifdef SERIAL_DEBUG
   mySerial.print("Read from eeprom at 0 ... ");
   #endif
   uint8_t t = eeprom_read_byte(0);
   #ifdef SERIAL_DEBUG
   mySerial.print((int32_t)t);
   mySerial.print("\r\n");
   #endif
   if(t!=1) {
     #ifdef SERIAL_DEBUG
     mySerial.print("Read to eeprom 1 at 0 ... ");
     #endif
     eeprom_write_byte( 0, 1 );
     #ifdef SERIAL_DEBUG
     mySerial.print("ok\r\n");
     #endif
   }
   #endif
  // ---
  
  #ifdef USE_SENSOR_DHT22
  pinMode(DHT22_POWER, OUTPUT); // set power pin for DHT to output
  #endif
  #ifdef USE_LED
  pinMode(LED_POWER, OUTPUT);
  #endif
  
  PRR = bit(PRTIM1); // only keep timer 0 going
  ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
}

void loop() {
  // put your main code here, to run repeatedly:
  #ifdef SERIAL_DEBUG
  mySerial.print("entering loop\r\n");
  #endif
 
  #ifdef USE_SENSOR_DHT22
  digitalWrite(DHT22_POWER, HIGH); // turn DHT sensor on
  DHT22_ERROR_t errorCode;
  #endif
  
  #ifdef USE_LED
  digitalWrite(LED_POWER, HIGH); // turn LED sensor on
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  //delay(500);               // wait for a second
  Sleepy::loseSomeTime(500);
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  //delay(500); 
  Sleepy::loseSomeTime(500);
  Sleepy::loseSomeTime(1000);
  #else
  Sleepy::loseSomeTime(1000);
  #endif
  
  
 
  #ifdef USE_SENSOR_DHT22
  errorCode = myDHT22.readData(); // read data from sensor

  //if (errorCode == DHT_ERROR_NONE) { // data is good
  //  tinytx.temperature = (myDHT22.getTemperatureC()*100); // Get temperature reading and convert to integer, reversed at receiving end
  //  tinytx.humidity = (myDHT22.getHumidity()*100); // Get humidity reading and convert to integer, reversed at receiving end
  //} else {
  //  tinytx.temperature = 32768; // Error-Marker
  //  tinytx.humidity = 32768; // Error-Marker
  //}
  #endif
  
  #if defined(USE_SENSOR_BH1750)
  #ifdef SERIAL_DEBUG
  mySerial.print("reading light level... ");
  #endif
  // Licht-Werte auslesen und aufbereiten
  //float lux = lightMeter.readLightLevel(); // TODO: Prüfen: Daten ohne Sensor
  int32_t lux = lightMeter.readLightLevel();  // TEST
  //char clux[9];
  //dtostrf(lux, 8, 1, clux);
  #ifdef SERIAL_DEBUG
  mySerial.print("\r\nlux: ");
  //mySerial.print(clux);
  mySerial.print(lux);
  mySerial.print(" Lx\r\n");
  #endif
  #endif
  
  
  #if defined(USE_SENSOR_BMP085)
  #ifdef SERIAL_DEBUG
  mySerial.print("reading baro... temp... ");
  #endif
  // Get raw temperature reading
  baroMeter.startMeas(BMP085::TEMP);
  Sleepy::loseSomeTime(16);
  int32_t traw = baroMeter.getResult(BMP085::TEMP);
  #ifdef SERIAL_DEBUG
  mySerial.print("ok ... ");
  //mySerial.print((int16_t)traw);

  mySerial.print("press ... ");
  #endif
  // Get raw pressure reading
  baroMeter.startMeas(BMP085::PRES);
  #ifdef SERIAL_DEBUG
  mySerial.print("ok ... \n\rget result... ");
  #endif
  Sleepy::loseSomeTime(32);
  int32_t praw = baroMeter.getResult(BMP085::PRES);
  #ifdef SERIAL_DEBUG
  //mySerial.print((int16_t)praw);
  mySerial.print("ok ... \n\r");
  #endif
  
  // Calculate actual temperature and pressure
  int16_t temp;
  int32_t press;
  #ifdef SERIAL_DEBUG
  mySerial.print("calclate ... ");
  #endif
  baroMeter.calculate(temp, press);
  #ifdef SERIAL_DEBUG
  mySerial.print("ok\r\n");
  mySerial.print("Temp: ");
  char buf [12];
  itoa ( temp,buf,10);
  mySerial.print(buf);
  //mySerial.print(temp);  
  mySerial.print(" C\r\n");
  mySerial.print("Press: ");  
  ltoa ( press,buf,10);
  mySerial.print(buf);
  //mySerial.print((int16_t)press);
  mySerial.print(" hPa\r\n");
  #endif
  #endif
  
  #ifdef USE_SENSOR_DHT22
  digitalWrite(DHT22_POWER, LOW); // turn Sensors off
  #endif

  // soll nicht ausgeführt werden, nur 'hineinkompiliert' (Optimizer wirft sonst raus)
  if(tinytx.magic==1) {
    if(rfwrite()) {
      // ? TODO
    }
  }
  
  #ifdef SERIAL_DEBUG
  mySerial.print("go to sleep...\r\n\r\n\r\n");
  #endif
  Sleepy::loseSomeTime(6000); //JeeLabs power save function: enter low power mode for 60 seconds (valid range 16-65000 ms)
}
