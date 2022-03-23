/******************************************************************************/
//Includes:
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <RadioHead.h>
// #include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPSPlus.h>
/******************************************************************************/
// Defined constants
TinyGPSPlus gps;
#define FrontGPS_Serial Serial2 // designate a serial communication line that connects to the front GPS, or GPS 1
#define BackGPS_Serial Serial1 //designate a serial communication line that connects to the back GPS, or GPS 2
#define READGPS1 1500 // timer period for reading from gps 1 in milliseconds
#define READGPS2 1000 // timer period for reading from gps 2 in milliseconds
//Global variables
unsigned int read_gps1_countdown; //countdown for reading from gps 1
unsigned int read_gps2_countdown; //countdown for reading from gps 2
// variables to hold the latitude and longitude of the front of the boat
float frontLat;
float frontLon;
// variables to hold the latitude and longitude of the back of the boat
float backLat;
float backLon;
float heading; // represents the
uint32_t timer = millis();
uint32_t timer2 = millis();


float bearing(float lat1,float lon1,float lat2,float lon2)  { // bearing function determines a heading based on two GPS coordinates, used in the determination of boat heading
  Serial.println(lat1,8);
  Serial.println(lon1,8);
  Serial.println(lat2,8);
  Serial.println(lon2,8);
  float teta1 = radians(lat1);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2-lat1);
  float delta2 = radians(lon2-lon1);
  //==================Heading Formula Calculation================//
  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
  float brng = atan2(y,x);
  brng = degrees(brng);// radians to degrees
  brng = ( ((int)brng + 360) % 360 );
  return brng;
}

void setup() { // Adafruit function which is automatically called on startup and runs before the main loop, Here is where we start everything and set startup conditions if needed
  Serial.begin(9600); // start the main Serial, this is the one which outputs to the monitor
  FrontGPS_Serial.begin(9600);
  BackGPS_Serial.begin(9600);
  // read_gps1();  // run a read from GPS 1 once
  // read_gps1_countdown = millis(); // set the value of the countdown for GPS 1
//  read_gps2();  // run a read from GPS 1 once
//  read_gps2_countdown = millis(); // set the value of the countdown for GPS 2
  /*
    We run each task that makes up our cyclic executive loop once in setup to gather an initial timestamp for
    each of their respective countdown variables. The countdown variable is a time stamp in milliseconds grabbed from
    the systems clock.
  */
}

void loop() {
  /*
    the following conditional loop is a soft-real time cyclic executive function. We check
    if the difference between the current time, gathered from millis(), and the last timestamp,
    denoted as something_countdown, is greater than the defined frequency for that task. If another
    task is running when this condition is met we simply wait until computing space is available
    then run. Priority can be set based on how high the task is in the if statement.

    TLDR: This lets us set how often we run each task, based on the period constant, Ex: "READGPS1"
  */
//  if((millis() - read_gps1_countdown) > READGPS1) { // Read the front GPS
//    read_gps1();
//    read_gps1_countdown = millis();
//  }
//  else if((millis() - read_gps2_countdown) > READGPS2) { // Read the back GPS
//    read_gps2();
//    read_gps2_countdown = millis();
//  }
  read_gps1();
  read_gps2();
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
    Serial.print("Front coords: (");
    Serial.print(frontLat,8);
    Serial.print(", ");
    Serial.print(frontLon,8);
    Serial.print(")");
    Serial.println("");
    Serial.print("Back coords: (");
    Serial.print(backLat,8);
    Serial.print(", ");
    Serial.print(backLon,8);
    Serial.print(")");
    Serial.println("");
  }  
  if (millis() - timer2 > 2000) {
    timer2 = millis(); // reset the timer
    heading = bearing(backLat, backLon, frontLat, frontLon);
    Serial.println("");
    Serial.print("The heading is: ");
    Serial.print(heading);
    Serial.print(" Degrees");
    Serial.println("");
  }
}

void read_gps1() {
    while (FrontGPS_Serial.available()) {
        gps.encode(FrontGPS_Serial.read());
        frontLat = gps.location.lat();
        frontLon = gps.location.lng();
    }    
}

void read_gps2() {
    while (BackGPS_Serial.available()){
        gps.encode(BackGPS_Serial.read());
        backLat = gps.location.lat();
        backLon = gps.location.lng();
    }
}
