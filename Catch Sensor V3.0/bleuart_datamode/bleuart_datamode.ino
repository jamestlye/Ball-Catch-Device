/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

//Flex Sensor Setup

// output
const char RELEASE = 0;
const char CATCH = 1;
const char THRESHOLD = 2;

// input
const char THRESH_QUERY = 0;
const char THRESH_UPDATE = 1;

const int flexPin = A0;
//const int minButtonReleaseTime = 50; //50 milliseconds
// const int ledPin =  13;

int currentVal;
bool lastState = LOW;
bool currentState;
bool printedRelease = false;

unsigned long onTime = 0;
unsigned long lastOnTime = 0;
unsigned long currentTime = 0;
unsigned long lastBattCheckTime = millis();

int threshold;
int thresholdOff = 20;
int holdDuration = 1000;

double voltSum = 0;
double voltAvg;

bool validState = true; //valid if the voltage is not rapidly changing
double slope;

int past10Volt[15];
int i;

double past15Avg[15];
unsigned long past15Times[15];

int buzzer = 11;

float battPercent = 0.00;
String battPercentString;

unsigned long sum_x = 0;
double sum_y = 0;
unsigned long sum_x2 = 0;
double sum_xy = 0;
unsigned long clippedTime = 0;


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
//  while (!Serial);  // required for Flora & Micro
  // we use this for Feather, so not required
  // (also if we block and wait for serial we will always need a wired connection)


  delay(500);

  // pinMode(ledPin, OUTPUT);
  pinMode(flexPin, INPUT);

  Serial.begin(115200);

 /* Initialise the module */
 Serial.print(F("Initialising the Bluefruit LE module: "));

 if ( !ble.begin(VERBOSE_MODE) )
 {
   error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
 }
 Serial.println( F("OK!") );

 if ( FACTORYRESET_ENABLE )
 {
   /* Perform a factory reset to make sure everything is in a known state */
   Serial.println(F("Performing a factory reset: "));
   if ( ! ble.factoryReset() ){
     error(F("Couldn't factory reset"));
   }
 }

 if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Catch Sensor" )) ) {
   error(F("Could not set device name?"));
 }

 /* Disable command echo from Bluefruit */
 ble.echo(false);

 Serial.println("Requesting Bluefruit info:");
 /* Print Bluefruit information */
 ble.info();

 ble.verbose(false);  // debug info is a little annoying after this point!

 /* Wait for connection */ //needed to be connected in order to work
 while (! ble.isConnected()) {
     delay(500);
 }

 Serial.println(F("******************************"));

 // LED Activity command is only supported from 0.6.6
 if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
 {
   // Change Mode LED Activity
   Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
   ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
 }

 // Set module to DATA mode
 Serial.println( F("Switching to DATA mode!") );
 ble.setMode(BLUEFRUIT_MODE_DATA);

//CALIBRATIOM START
int volt,average = 0;
tone(buzzer, 1000, 125);
delay(200);
tone(buzzer, 2000, 125);
delay(200);
tone(buzzer, 3000, 125);
delay(200);
tone(buzzer, 4000, 125);
delay(5000);

for (int i = 0; i<5; i++){
    tone(buzzer, 1000, 125);
    delay(200);
    tone(buzzer, 3000, 125);

    delay(2000);

    average += analogRead(flexPin);

    tone(buzzer, 4000, 125);
    delay(200);
    tone(buzzer, 4000, 125);
    
    delay(3000);    
}

threshold = average/5;
//CALIBRATION END
 Serial.println(threshold);



  Serial.println(F("******************************"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/



void loop(void)
{
//  tone(buzzer, 4000,500);

  currentVal = analogRead(flexPin);
  currentTime = millis();

//  Serial.print(currentVal);
//  Serial.print(',');

  voltSum = 0;
  for(i = 0; i < 14; i++){ //they are all array of 15 items
    //shuffling forward
    past10Volt[i] = past10Volt[i+1];
    past15Times[i] = past15Times[i+1];
    past15Avg[i] = past15Avg [i+1];
    voltSum += past10Volt[i];    
  }
  past10Volt[14] = currentVal;
  past15Times[14] = currentTime;
  voltSum += currentVal;
  
  voltAvg = voltSum / 15.0;
  past15Avg[14] = voltAvg;

  Serial.print(threshold);
  Serial.print(',');
  Serial.println(voltAvg);
//  Serial.print(',');
  
//  delay(25);


  currentState = (voltAvg >= threshold - thresholdOff && voltAvg <= threshold + thresholdOff); //determine the state of the flex sensor ON/OFF

  //determine if state is valid (slope)

  //LEAST SQUARE APPROXIMATION (to find line of best fit's slope)(unit: volt/ms) 
  sum_x = sum_y = sum_x2 = sum_xy = 0;
  
  for (int k = 0; k < 15; k++){

    clippedTime = past15Times[k] - past15Times[0];
    
    sum_x += clippedTime;
    sum_y += past15Avg[k];
    sum_x2 += clippedTime*clippedTime;
    sum_xy += past15Avg[k]*clippedTime;
  }

  slope = (15.0 * sum_xy - sum_x * sum_y)/(15.0 * (double)(sum_x2) - (double)(sum_x * sum_x));
//  Serial.print(sum_x2);
//  Serial.print(",");
//  Serial.println(slope);


  validState = (abs(slope) <= 0.2);

  if (!currentState || !validState){
    lastOnTime = currentTime;
  }
  
  if (currentState == true){
    onTime = currentTime - lastOnTime;
    //if ON time is enough
    if (onTime >= holdDuration && !printedRelease){
      //CATCH!!

     char message[5];
     // 1 byte for release/press flag, 4 bytes for ulong timestamp
     message[0] = CATCH;
     
     strncpy(message+1, (char *)&lastOnTime, 4); //sending lastOnTime as timestamp bc it is the original catch time (roughly 1000 ms before)

     Serial.println("Sending press");
     ble.write(message, 5);

      
      
     Serial.println("caught");
     Serial.println(onTime);

      tone(buzzer, 3000, 125);
      delay(200);
      tone(buzzer, 4000, 125);
      
      printedRelease = true;
    }
    }

  if (currentState == false && printedRelease == true){
    //RELEASE!!

   char message[5];
   // 1 byte for release/press flag, 4 bytes for ulong timestamp
   message[0] = RELEASE;
   strncpy(message+1, (char *)&currentTime, 4);
     
   Serial.println("Sending release");
   ble.write(message, 5);

    
   Serial.println("released");
   Serial.println(slope);

    tone(buzzer, 4000, 125);
    delay(200);
    tone(buzzer, 3000, 125);
    
    printedRelease = false; //reset for the next cycle of catch and release
  }
  
  
  lastState = currentState;  


  
  // if the pc sent any data
  if (ble.available()){
    char c = ble.read();

    // if the computer is asking for the current value
    if (c == THRESH_QUERY){
      char message[3];
      message[0] = THRESHOLD;
      strncpy(message+1, (char *)&threshold, 2);
      ble.write(message, 3);
    }

    // if the computer is trying to set the value
    if (c == THRESH_UPDATE){
      char newval[2];

      newval[0] = ble.read();
      newval[1] = ble.read();

      strncpy((char *)&threshold, newval, 2);
    }
  }
  
}
