// attiny85_dht22_ldr_xrf.ino
//
// This sketch is used on several wireless sensor nodes running on the attiny85 to report temperature/humidity levels to a central node utilsiing Ciseco XRF radios
// In addition to the above a PIR will wake the micro through an interrupt to send an 'alarm' message when it detects motion.
// This code will put the micro to sleep most of the time, only waking to send values every n minutes (configurable via the int interval). Every 10 sensor transmissions the micro will send the voltage level of the battery (vcc)
// The radio is also put to sleep when not used, using sleep mode 2 (http://openmicros.org/index.php/articles/88-ciseco-product-documentation/260-srf-configuration) - this results in only 0.5uA sleep power consumption.
//
// Libraries used here are a modified version of the Ciseso LLAPSerial library (https://github.com/CisecoPlc/LLAPSerial) and the DHT22 library written by robtillaart (http://playground.arduino.cc/Main/DHTLib)
// The LLAPSerial library has been copied and modified slightly to work with the attiny85 - it's classes are simply referenced in this sketch at the bottom of the file.
//
// Interrupt, watchdog and sleep code has been adapted from code written by Nick Gammon. Reference: http://www.gammon.com.au/forum/?id=11488&reply=9#reply9
//
// The attiny85 core used is: https://code.google.com/p/arduino-tiny/
//
// To ensure predicible serial behaviour, the internal oscillator needs to be 'tuned'. This sketch assumes that the oscillator has been tuned with the 'Save_to_EEPROM' TinyTuner sketch. More information on TinyTuner
// can be found here : http://forum.arduino.cc/index.php/topic,8553.0.html
//
// Ben Morrice January 2016
//
//
//                  Pinout
//                 ATTINY85
//                  +-\/-+
// RESET / LDR (D5) |    | VCC
//      XRF TX (D3) |    | BATTERYMON (A1)
//     PIR INT (D4) |    | DHT22 (D1)
//              GND |    | XRF/DHT22 POWER ENABLE (D0)
//                  +----+
//
// -- CHANGE DEVICE ID BEFORE UPLOADING -- //
char* deviceID = "TS";
const int interval = 1; // how often in minutes to sleep before sending sensor measurements
// -- END OF CHANGES REQUIRED -- //

const int sleep_total = (interval*60)/8; // Approximate number 
const int battery_total = 10;
int battery_count = 0;
volatile int sleep_count = 0;
long startMillis = 1;
long currentMillis = 1;
long currentPIREvent = 1;
long lastPIREvent = 1;
int vcc;
char cMessage[13];
int alarmDetected = 0;
char tmp[6];

const byte PIR = 4;
const byte pinPower = 0;
const byte batteryMonitor = A1;

#include "Arduino.h"
#include "dht.h"
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer
#include <EEPROM.h>

#define DHT22_PIN 1

dht DHT;

ISR (PCINT0_vect) 
{
  currentPIREvent = millis();
  if(currentPIREvent - lastPIREvent > 10000 ) { // only send an event if 60 seconds have passed from the previous
    sendMessage("ALARM");
    lastPIREvent = millis();
   }
   alarmDetected = 1;//
detachInterrupt(PIR);
alarmDetected = 1;
sleep_count = sleep_total;
GIMSK &= ~(1<<PCIE); //disable interrupt
delayMillis(20);
sleep_count++;
}  // end of PCINT0_vect
 
// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
   sleep_count ++;
}  // end of WDT_vect

 


void resetWatchdog ()
  {
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset, clear existing interrupt
  WDTCR = bit (WDCE) | bit (WDE) | bit (WDIF);
  // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
  WDTCR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  // pat the dog
  wdt_reset();  
  }  // end of resetWatchdog
  
// small function to delay for a period of time
// using this instead of 'delay'
void delayMillis(int interval) {
  startMillis = millis();
  while(1) {
    currentMillis = millis();
    if(currentMillis - startMillis > interval) {
      break;
    }   
  }
}  
  
  
void setup ()
{
  
  resetWatchdog ();  // do this first in case WDT fires
  
  OSCCAL = EEPROM.read(0);
  delayMillis(1000);
  
  Serial.begin(9600);
  // let the oscilator value kick in before attempting to use serial
  delayMillis(2000);
  pinMode(batteryMonitor, INPUT);

  //-------------enable XRF sleep mode 2---------------------------------------------- 
  //http://openmicros.org/index.php/articles/88-ciseco-product-documentation/260-srf-configuration       
  pinMode(pinPower,OUTPUT);
  digitalWrite(pinPower,HIGH);
  Serial.print("+++");      // enter AT command mode
  delayMillis(1500);        // delay 1.5s
  Serial.println("ATSM2");  // enable sleep mode 2 <0.5uA
  delayMillis(2000);        // delay 2.0s  
  Serial.println("ATDN");   // exit AT command mode*/
  delayMillis(2000);        // delay 2.0s
  sendMessage("STARTED");

  //---------------------------------------------------------------------------------
  
  pinMode (PIR, INPUT);
  digitalWrite (PIR, HIGH);  // internal pull-up
  
  // pin change interrupt (example for D4)
  PCMSK  = bit (PCINT4);  // want pin D4 / pin 3
  GIFR  |= bit (PCIF);    // clear any outstanding interrupts
  GIMSK |= bit (PCIE);    // enable pin change interrupts  
}  // end of setup

void loop ()
{
  
  currentMillis = millis();
  if(currentMillis - lastPIREvent > 60000)
    
  
  if (sleep_count >= sleep_total) {
    delayMillis(2000);
    enableRadio();
    if (battery_count == battery_total) {
      sendMessage("WAKE");
      vcc = 3.3 * ((int) analogRead(batteryMonitor)); // 3.3 is attiny85 voltage from step up regulator
      sendIntWithDP("BATT",vcc,3); //readVcc returns a the Vcc in milliamp. We want volts, so 3 decimal places are needed
      sendMessage("SLEEPING");
      battery_count = 0;
    }
    
    if(alarmDetected == 1) {
      lastPIREvent = millis();
      sendMessage("MOTION");
      alarmDetected = 0;
      GIMSK |= bit (PCIE); //enable interrupt

      attachInterrupt(PIR);
    }
    int photocellReading = analogRead(photocellPin);
    sendMessage("LIG" + String(photocellReading));
    delayMillis(100);
    enableDHT();
    int chk = DHT.read22(DHT22_PIN);
    disableDHT();
    if (chk == DHTLIB_OK) { //DHTLIB_OK is 0
      sendMessage("HUM" + String(dtostrf(DHT.humidity,2,2,tmp)));
      sendMessage("TMP" + String(dtostrf(DHT.temperature,2,2,tmp)));
    }
    battery_count++;
    sleep_count = 0;
  }
  disableRadio();
  goToSleep ();

}  // end of loop


void goToSleep ()
{
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  ADCSRA = 0;            // turn off ADC
  power_all_disable ();  // power off ADC, Timer 0 and 1, serial interface
  noInterrupts ();       // timed sequence coming up
  resetWatchdog ();      // get watchdog ready
  sleep_enable ();       // ready to sleep
  interrupts ();         // interrupts are required now
  sleep_cpu ();          // sleep                
  sleep_disable ();      // precaution
  power_all_enable ();   // power everything back on
}  // end of goToSleep 

void enableRadio() {
  digitalWrite(pinPower,HIGH);
  delayMillis(1000);
}
void disableRadio() {
  digitalWrite(pinPower,LOW);
}

void enableDHT() {
  //digitalWrite(pinEnableDHT,HIGH);
  delayMillis(3000);
}
void disableDHT() {
  //digitalWrite(pinEnableDHT,LOW);
}


//-------------------------------------------------------------------------------------------------------
// function to read supply voltage
//-------------------------------------------------------------------------------------------------------


 long readVcc() {
   bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
   long result;
   // Read 1.1V reference against Vcc
   #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
   delayMillis(10); // Wait for Vref to settle
   ADCSRA |= _BV(ADSC); // Convert
   while (bit_is_set(ADCSRA,ADSC));
   result = ADCL;
   result |= ADCH<<8;
   result = 1126400L / result; // Back-calculate Vcc in mV
   ADCSRA &= ~ bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
   return result;
} 
  
  
//-------------------------------------------------------------------------------------------------------
// LLAPSerial (https://github.com/CisecoPlc/LLAPSerial) Code BEGIN
//-------------------------------------------------------------------------------------------------------

void sendMessage(String sToSend)
{
  cMessage[0] = 'a';
  cMessage[1] = deviceID[0];
  cMessage[2] = deviceID[1];

  for (byte i = 0; i<9; i++) {
    if (i < sToSend.length())
      cMessage[i+3] = sToSend.charAt(i);
     else
      cMessage[i+3] = '-';
    }
    Serial.print(cMessage);
    Serial.flush();
    delayMillis(1000);
    
}

void sendIntWithDP(String sToSend, int value, byte decimalPlaces)
{
 
  char cValue[8];		// long enough for -3276.7 and the trailing zero
  byte cValuePtr=0;
  itoa(value, cValue,10);
  char* cp = &cValue[strlen(cValue)];
  *(cp+1) = 0;	// new terminator
  while (decimalPlaces-- && --cp )
  {
    *(cp+1) = *cp;
  }
  *cp = '.';

  cMessage[0] = 'a';
  cMessage[1] = deviceID[0];
  cMessage[2] = deviceID[1];
  for (byte i = 0; i<9; i++) {
    if (i < sToSend.length())
      cMessage[i+3] = sToSend.charAt(i);
    else if (cValuePtr < 8 && cValue[cValuePtr] !=0)
      cMessage[i+3] = cValue[cValuePtr++];
    else
      cMessage[i+3] = '-';
  }
  Serial.print(cMessage);
  Serial.flush();
  delayMillis(1000);
}

//-------------------------------------------------------------------------------------------------------
//LLAPSerial Code (https://github.com/CisecoPlc/LLAPSerial) END
//-------------------------------------------------------------------------------------------------------
