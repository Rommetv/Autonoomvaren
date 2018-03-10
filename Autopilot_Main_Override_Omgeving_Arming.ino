#include <Wire.h>
#include "compass.h"
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>
#include <Average.h>
//#include "LowPower.h"
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include <Servo.h>

Servo LinkerEsc; 
Servo RechterEsc;

//Motor Control
int maxspeed;
//maxspeed - motgain * 30 > 0
//MOTGAIN 4
int motgain = 5;
int anglerange = 30;
//int rightpin = 10;
//int leftpin = 9;
int led = 13;
int overavg = 10;
int steerarray[10];
int throttlearray[10];
int linksIn = 5;
int rechtsIn = 3;
unsigned long RRechts = 0;
unsigned long RLinks = 0;

//standby en sensor
int Switch = 6;
int armStatus = 0;
const int trigPin = 2;
const int echoPin = 4;
long duration, cm;


//GPS
static int r = 6371;
const float Pi = 3.14159;
int accuracy = 5;
unsigned long lasttime = 0;

SoftEasyTransfer rx;

SoftwareSerial link(7, 8);

struct RECEIVE_DATA_STRUCTURE{
  float latitude;
  float longitude;
};

RECEIVE_DATA_STRUCTURE gpsin;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  //pinMode(rightpin, OUTPUT);
  //pinMode(leftpin, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(linksIn, INPUT);
  pinMode(rechtsIn, INPUT);
  //standby en sensor
  pinMode(Switch, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  compass_init(2);
  compass_debug = 1;
  compass_offset_calibration(3);
  link.begin(38400);
  rx.begin(details(gpsin), &link);
  LinkerEsc.attach(9);
  RechterEsc.attach(10);
  digitalWrite(led, HIGH);
}

void loop() {
  Navigate(52.346528, 4.778659, 1850);
  Navigate(52.345233, 4.781572, 1850);
  Navigate(52.346150, 4.782902, 1850);
  //Control();
}

//void Control() {
//  while (pulseIn(5, HIGH, 100000) > 0){
//    int throttle = pulseIn(4, HIGH, 20000);
//    int steer = pulseIn(5, HIGH, 20000);
//    if (throttle > 1000 && steer > 1000) {
//      //Override(throttle, steer);
//    }
//    delay(50);
//  }
//}

void Navigate(float lat2, float long2, int velocity) {
  maxspeed = velocity;
  while (GetDistanceInM(gpsin.latitude, gpsin.longitude, lat2, long2) > accuracy) {
    if (rx.receiveData()){
      lasttime = millis();
    }
      while (digitalRead(Switch) == HIGH){
        if (armStatus == 0){
        ++armStatus;
        Serial.println("Arm");
        RechterEsc.writeMicroseconds(800);
        delay(30);
        LinkerEsc.writeMicroseconds(800);
        delay(30);
     }
  
    RechterEsc.writeMicroseconds(1520);
    LinkerEsc.writeMicroseconds(1520);
    Serial.println("Stand-By");
   }

    if (armStatus == 0 ){
        Serial.println("BeforeArm");
        RechterEsc.writeMicroseconds(1850);
        LinkerEsc.writeMicroseconds(2260);
    }

  //als al bij arm is geweest dan
   else{
    while (pulseIn(rechtsIn, HIGH) > 1550){
      Override();
      
      delay(50);
    }
      while (pulseIn(linksIn, HIGH) > 1550){
      Override();
      
      delay(50);
    }
    //ultrasonic
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    cm = microsecondsToCentimeters(duration);
    while(cm < 20 &&(pulseIn(rechtsIn, HIGH, 20000) < 1650 || pulseIn(linksIn, HIGH, 20000) < 1650)){
      Serial.println("Achteruit");
        RechterEsc.writeMicroseconds(1200);
        LinkerEsc.writeMicroseconds(1200);
        delay(2000);
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        cm = microsecondsToCentimeters(duration);
        
    }
    
    
    if ((millis() - lasttime) < 2000) {
      int bearing = GetBearing();
      int heading = GetHeading(gpsin.latitude, gpsin.longitude, lat2, long2);
      int leftmot = MotorSpeed(bearing, heading, 1);
      int rightmot = MotorSpeed(bearing, heading, 0);
      if (leftmot < 0) {
        leftmot = 1520;
      }
      if (rightmot < 0) {
        rightmot = 1520;
      }
      RunMotor(leftmot, 1);
      RunMotor(rightmot, 0);
      Serial.print(bearing);
      Serial.print("  ");
      Serial.print(heading);
      Serial.print("  ");
      Serial.print(GetDistanceInM(gpsin.latitude, gpsin.longitude, lat2, long2));
      Serial.print("  ");
      Serial.print(leftmot);
      Serial.print("  ");
      Serial.println(rightmot);
    }
    else {
      RunMotor(1520, 1);
      RunMotor(1520, 0);
      Serial.println("No GPS Signal");
    }
   }
  }
  Serial.println("NavLoop Return");
  RunMotor(1520, 1);
  RunMotor(1520, 0);
}

void Override() {
  RLinks = pulseIn(linksIn, HIGH);
  RRechts = pulseIn(rechtsIn, HIGH);
  RunMotor(RRechts, 0);
  RunMotor(RLinks, 1);
  Serial.print(RLinks);
  Serial.print("  ");
  Serial.println(RRechts);
}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

void RunMotor(int val, int option){
  if (option == 0) {
    //Right Motor
   RechterEsc.writeMicroseconds(val);
  }
  else {
    //Left Motor
   LinkerEsc.writeMicroseconds(val);
  }
}

int GetBearing() {
  compass_heading();
  float headingradians = bearing / (180 / M_PI);
  float declinationAngle = 0.289724656;
  headingradians += declinationAngle;
  bearing = (int)((headingradians * 180 / M_PI)+270) % 360;
  return bearing;
}

int MotorSpeed(int bearing, int heading, int option) {
  heading = heading % 360;
  bearing = bearing % 360;
  if (heading == bearing)
  {
    return (maxspeed);
  }
  int o1 = (int)heading - (int)bearing;
  int o2 = (int)bearing - (int)heading;
  int direction = ((360 - ((int)heading - (int)bearing)) % 360);
  if (direction < 180) {
    if (direction > anglerange)
    {
      //Right Motor
      if (option == 0) {
        return maxspeed;
      }
      //Left Motor
      else {
        return 1520;
      }
    }
    else
    {
      //Right Motor
      if (option == 0) {
        return maxspeed;
      }
      //Left Motor
      else {
        return maxspeed - (direction * motgain);
      }
    }
  }
  else if (direction >= 180)
  {
    if ((360 - direction) > anglerange)
    {
      //Right Motor
      if (option == 0) {
        return 1520;
      }
      //Left Motor
      else {
        return maxspeed;
      }
    }
    else
    {
      //Right Motor
      if (option == 0) {
        return maxspeed - ((360 - direction) * motgain);
      }
      //Left Motor
      else {
        return maxspeed;
      }
    }
  }
  return 0;
}

static float GetHeading(float lat1, float long1, float lat2, float long2)
{
  //Current Latitude, Current Longitude
  //Projected Latitude, Projected Longitude
  //To get reverse heading, add 180
  lat1 = ConvertToRadians(lat1);
  lat2 = ConvertToRadians(lat2);
  long1 = ConvertToRadians(long1);
  long2 = ConvertToRadians(long2);
  float dLon = long2 - long1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  return ConvertToStaticDegrees(atan2(y, x));
}
static float GetDistanceInM(float lat1, float long1, float lat2, float long2)
{
  //Current Latitude, Current Longitude
  //Projected Latitude, Projected Longitude
  lat1 = ConvertToRadians(lat1);
  lat2 = ConvertToRadians(lat2);
  long1 = ConvertToRadians(long1);
  long2 = ConvertToRadians(long2);
  float dLat = lat2 - lat1;
  float dLon = long2 - long1;
  float a = haversin(dLat) + cos(lat1) * cos(lat2) * haversin(dLon);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return r * c * 1000; // Distance in M
}
static float ConvertToDegrees(float rad)
{
  float degrees = rad / (Pi / 180);
  if (degrees <= -180) {
    return degrees + 360;
  }
  else {
    return degrees;
  }
}
static float ConvertToStaticDegrees(float rad)
{
  float degrees = rad / (Pi / 180);
  if (degrees <= 0) {
    return degrees + 360;
  }
  else {
    return degrees;
  }
}
static float ConvertToRadians(float angle)
{
  return (Pi / 180) * angle;
}
static float haversin(float input)
{
  return (sin(input / 2)) * (sin(input / 2));
}















