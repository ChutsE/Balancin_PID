#include <ESP8266WiFi.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

#define MAX 800

#define range 5

#define kp 20
#define ki 10
#define kd 10

int PWM1=16;
int PWM2=0;

float PID,pid_p=0,pid_i=0,pid_d=0;
float prev_theta=0, timeprev=0;

int conv;

float distancia, theta, dt=0;
float R=261; //mm

void setup() {
  Serial.begin(115200);

  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}


void loop() {
  
  dt=(millis()-timeprev)/1000;
  timeprev=millis();
  
  VL53L0X_RangingMeasurementData_t measure;
  
    
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
  distancia=measure.RangeMilliMeter-20;
  theta=(2*atan(distancia/R)-PI/2)*(180/PI);
  
  
  
  
  pid_p=kp*theta;
  pid_i=pid_i+ki*theta*dt;
  pid_d=kd*((theta-prev_theta)/dt);
  prev_theta=theta;
  
  PID=pid_p+pid_i+pid_d;

  Serial.print("P= ");Serial.print(pid_p);Serial.print(" I= ");Serial.print(pid_i);Serial.print(" D= ");Serial.print(pid_d);Serial.print(" PID= ");Serial.println(PID);

  if(PID>MAX){
    PID=MAX;
  }
  if(PID<-MAX){
    PID=-MAX;
  }
  
  
  Serial.print("Distance (mm): "); Serial.print(measure.RangeMilliMeter); Serial.print("  Angulo ( °): "); Serial.println(theta);

    

    if(theta<range){
      conv=map(PID, -MAX , 0, 254, 60);
      analogWrite(PWM1,0);
      analogWrite(PWM2,conv);
    }
    else if(theta>range){
      
      conv=map(PID, 0 , MAX, 60, 200);
      analogWrite(PWM2,0);
      analogWrite(PWM1,conv);
    }
 
 
  delay(50);
}
