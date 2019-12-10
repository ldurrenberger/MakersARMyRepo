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
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
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
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


//set servo data
//10kg (120 degree) & 35kg (270 degree) servo:
//specify which output pin you are using
#define SERVOMIN  105 // This is the ‘minimum’ pulse length count (out of 4096)
#define SERVOMAX  510 // This is the ‘maximum’ pulse length count (out of 4096)
#define USMIN  500 // This is the rounded ‘minimum’ microsecond length
#define USMAX  2500 // This is the rounded ‘maximum’ microsecond length
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

char buf[1000];
char msg_open = 0;
int bigServoBegin;
int bigServoEnd;
int currentPos;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  bigServoBegin = SERVOMIN;
  bigServoEnd = SERVOMIN + ((SERVOMAX - SERVOMIN) * 2 / 3);
  currentPos = (bigServoBegin + bigServoEnd)/2;
  
  //servo initialization
  pwm.begin();
  // In theory the internal oscillator is 25MHz but it really isn't
  // that precise. You can 'calibrate' by tweaking this number till
  // you get the frequency you're expecting!
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates

  
  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("Also servo stuff!"));
  Serial.println(F("---------------------------------------"));

  

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

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }
}



/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    Serial.println("No data!");
    return;
  }
  bool toMove = false;
  Serial.println(ble.buffer);
  msg_open = 1;
  if (msg_open == 1) {
    //Serial.println("Received past closed message");
    //TODO: check behavior, potentially put it in a different char array
    char newBuf[1000];
    sprintf(newBuf, "%c%c", buf, ble.readline());
    for(int i = 0; i < 1000; i++){
      buf[i] = newBuf[i];
      if(buf[i] == 0){
        break;
      }
    }
//    Serial.print("Received message: ");
//    Serial.println(buf);
//    Serial.println("Message details:");
    for(int i = 0; i < 10; i++){
      if(buf[i] == 0){
        break;
      }
      else{
//        Serial.print("\t");
//        Serial.println((int)buf[i]);
      }
    }
//    Serial.println();

    int i = 0;
    while (true) {
      if(buf[i] == '>'){
        msg_open = 0;
        toMove = true;
        break;
      }
      if(buf[i] == 0){
        break;
      } 
      i++;
    }
  }
  else {
    //TODO: check behavior, potentially put it in a different char array
    sprintf(buf, "%c", ble.readline());
    bool endedLine = false;

//    while (true) {
//      if (buf[i] == '<') {
//        msg_open = 1;
//        break;
//      }
//      if (buf[i] == 0) {
//        break;
//      }
//    }
    for(int i = 0; i < 1000; i++){
      if(buf[i] == '>'){
        endedLine = true;
        break;
      }
      if(buf[i] == 0){
        break;
      }
    }

    if(!endedLine){
      msg_open = 1;
    }
    else{
      toMove = true;
    }
  }

  //we have the complete buffer, translate this and send it to the servo
  if(toMove){
    int deg;
    // <3>
    sscanf(buf, "%s%d%s", &deg);
    int pulseLength = map(deg, 180, 0, bigServoBegin, bigServoEnd);
    currentPos = pulseLength;

    //clear the buffer
    for(int i = 0; i < 1000; i++){
      buf[i] = 0;
    }
    toMove = false;
  }

  pwm.setPWM(0,0, currentPos);

  
  // Some data was found, its in the buffer
//  Serial.print(F("[Recv] ")); 
//  Serial.println(ble.buffer);
  ble.waitForOK();
}
