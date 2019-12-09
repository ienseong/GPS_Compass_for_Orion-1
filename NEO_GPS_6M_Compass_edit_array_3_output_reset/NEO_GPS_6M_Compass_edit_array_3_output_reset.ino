#include <AltSoftSerial.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "Keyboard.h"

/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 8(rx) and 9(tx) and a HMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

int led = 13;
int i =0;
byte turn;
//boolean turn;

//static const int RXPin = 8, TXPin = 9;
//static const int RXPin = 0, TXPin = 1;// GPS
static const int RXPin = 0, TXPin = 1;// GPS

static const uint32_t GPSBaud = 9600;
volatile static byte flag_data_updated;

static double gps_temp1,gps_temp2, compass_temp;

//static double arr[3] = {0.0f,0.0f, 0.0f};
static double arr[3];
static double headingDegrees;
// int led_gpio = 4;
int led_gpio = 12;
int led_gps = 2;

// Assign a Uniquej ID to the HMC5883 Compass Sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// The TinyGPS++ object
TinyGPSPlus gps;

static const int MAX_SATELLITES = 40;

TinyGPSCustom totalGPGSVMessages(gps, "GPGSV", 1); // $GPGSV sentence, first element
TinyGPSCustom messageNumber(gps, "GPGSV", 2);      // $GPGSV sentence, second element

struct
{
  bool active;

} sats[MAX_SATELLITES];

TinyGPSCustom satNumber[4]; // to be initialized later

// The serial connection to the NEO-6m GPS module
SoftwareSerial ss(RXPin, TXPin);

char inChar;

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);

  delay(500);
}

void setup()
{
  pinMode(led_gpio, OUTPUT);
  Serial.begin(9600);
  ss.begin(GPSBaud);
  displaySensorDetails();
   pinMode(led, OUTPUT);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded from the GPS Module.
  while (ss.available() > 0){
    if (gps.encode(ss.read())){
      UpdateGpsInfo();
      if(arr[0] != gps_temp1 || arr[1] != gps_temp2 || arr[2] != compass_temp) {
        DisplayNewData();
        gps_temp1 = arr[0];
        gps_temp2 = arr[1];
        compass_temp = arr[2];
      }

      turn= (turn+1)%2;
      digitalWrite(led_gps, turn);   // turn the LED on (HIGH is the voltage level)
      digitalWrite(led, turn);   // turn the LED on (HIGH is the voltage level)
    }

    inChar = Serial.read();

    if(inChar=='r'){ //reset
      Serial.println(F("Reset"));
      delay(1000);
      for ( i =0;i<10;i++){
        turn= (turn+1)%2;
        digitalWrite(led_gpio, turn);   // turn the LED on (HIGH is the voltage level)
        delay(100);
      }
      _softRestart();
     }
   }

   if(digitalRead(TXPin)==HIGH){
    digitalWrite(led_gpio, turn);   // turn the LED on (HIGH is the voltage level)
    delay(500);
  }
}

void UpdateGpsInfo()
{
  if (totalGPGSVMessages.isUpdated()){
    int totalMessages = atoi(totalGPGSVMessages.value());
    int currentMessage = atoi(messageNumber.value());
    if (gps.location.isValid()){
      arr[0] = gps.location.lat();
      arr[1] = gps.location.lng();
    }
    else{
    }
  }
  else{
  }

  UpdateCompass();
 }

void UpdateCompass(){
  //Serial.println();
  if(mag.begin()){
    displayCompassInfo();
  }
}

void displayCompassInfo()
{
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  double heading = atan2(event.magnetic.y, event.magnetic.x);
  double declinationAngle = 0.06; // radian
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;

  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;

  // Convert radians to degrees for readability.
  //float headingDegrees = heading * 180/M_PI;
   headingDegrees = heading * 180/M_PI;
   arr[2] = headingDegrees /1.0;
}

void DisplayNewData() {
  for (int i =0;i<3;i++){
    if( i ==2){
      Serial.print((int)arr[i]);
    }
    else {
      Serial.print(arr[i],6);
    }
    Serial.print(" ");
  }
  Serial.println();
}


#define SCB_AIRCR (*(volatile uint32_t *)0xE000ED0C) // Application Interrupt and Reset Control location

void _softRestart()
{
  Serial.end();  //clears the serial monitor  if used
  SCB_AIRCR = 0x05FA0004;  //write value for restart
}
