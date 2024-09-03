#include <Servo.h>
#include <Arduino.h>
#include <ListLib.h>
#include <HCSR04.h>
#include "Wire.h"
#include <MPU6050_light.h>
List<double> leftlist;

MPU6050 mpu(Wire);
unsigned long timer = 0;

Servo myservo;  // create servo object to control a servo

// defines arduino pins numbers

//ultra right
const int trigPin1 = 22;
const int echoPin1 = 23;
long duration1;
int distance1;

//ultra forworld
const int trigPin2 = 24;
const int echoPin2 = 25;
long duration2;
int distance2;

// ultra left
const int trigPin3 = 28;
const int echoPin3 = 29;
long duration3;
int distance3;

HCSR04 sensor1(trigPin1,echoPin1);
HCSR04 sensor2(trigPin2,echoPin2);
HCSR04 sensor3(trigPin3,echoPin3);

//pins motors
int motor2pin1_in3= 13;
int motor2pin2_in4 = 30;
int ENB=12;

//pin sernvo
int pin_servo=4;
int pos = 90;

String state="ff";
String state_old;
int max_speed=137;
int min_speed=110;
int corner_counter=0;
int round_counter=0;
int ff_counter=0;
int dir=0;//1 for left -1 for right
int expected_angle;
int real_angle;
float front0;
float ref_angle=0;
float front_old;
float left_old;
float right_old;
float left0;
float right0;
float diff;
long timer1=0;
long timer2=0; 

float dis_right(){
  distance1=sensor1.dist();
  return distance1;}

float dis_front(){
  distance2=sensor2.dist();
  return distance2;}

float dis_left(){
  distance3=sensor3.dist();
  return distance3;} 

void straight(){
  if(real_angle>expected_angle){
    diff=real_angle-expected_angle;
    myservo.write(90+diff);
    diff=map(diff,0,30,0,50);
    diff=constrain(diff,10,50);
    delay(diff);}

  else if(real_angle<expected_angle){
    diff=expected_angle-real_angle;
    myservo.write(90-diff);
    diff=map(diff,0,30,0,50);
    diff=constrain(diff,10,50);
    delay(diff);}
}

void stop(){ 
  digitalWrite(motor2pin1_in3, LOW);
  digitalWrite(motor2pin2_in4, LOW);
}

void forword(int s){ 
    analogWrite(ENB, s);
    digitalWrite(motor2pin1_in3, HIGH);
    digitalWrite(motor2pin2_in4, LOW);
  
}

void ff(){
  double angle;  
  float left_new;
  float right_new;
  float front_new;
  
  int x;
 
  while(true){
     
    
  if(millis()-timer1>500){
    Serial.print("expected_angle=");Serial.println(expected_angle);
    Serial.print("real_angle=");Serial.println(real_angle);
    timer1=millis();
    }

  left_new=dis_left();delayMicroseconds(20);
  if(left_new==0){left_new=left_old;}
  right_new=dis_right();delayMicroseconds(20);
  if(right_new==0){right_new=right_old;}
  front_new=dis_front();delayMicroseconds(20);
  if(front_new==0){front_new=front_old;}
////////////////jump in reading
      if(left_new>400||right_new>400||front_new>400){
          Serial.print("------jump occured ff------");
          Serial.print("leftnew=");Serial.println(left_new);
          Serial.print("righttnew=");Serial.println(right_new);
          
          left_new=left_old;
          right_new=right_old;
          front_new=front_old;
          forword(min_speed);}
/////////////////collision
      if(front_new<15&&front_old<15){
        Serial.println("collision");
          stop();   }   ////normal moving
      else {
        forword(max_speed); 
        }
      
//detect finish     
      if(corner_counter==2&& front_new<=front0){
        stop();state="finish";break;}

      if(corner_counter==0){
              if(left_new>70 && front_new<130 &&left_old<=70){corner_counter++;
                  Serial.print("corner=");Serial.print(corner_counter);stop();
                  ///////////
                  state="rl1";if(corner_counter==1)dir=1;delay(150);break;}
              
              else if(right_new>70 && front_new<130 &&right_old<=70){corner_counter++;
                  Serial.print("corner=");Serial.print(corner_counter);stop();
                  ///////////
                  state="rr1";if(corner_counter==1)dir=-1;delay(150);break;}
      }
      else if(dir==1){
                if(left_new>70 && front_new<130 &&left_old<=70){corner_counter++;
                    Serial.print("corner=");Serial.print(corner_counter);stop();
                    ///////////
                    state="rl1";if(corner_counter==1)dir=1;delay(150);break;}
      }
      else {
                if(right_new>70 && front_new<130 &&right_old<=70){corner_counter++;
                    Serial.print("corner=");Serial.print(corner_counter);stop();
                    ///////////
                    state="rr1";if(corner_counter==1)dir=-1;delay(150);break;}
      }
 

  Serial.print("left_new=");Serial.println(left_new);
  Serial.print("left_old=");Serial.println(left_old);
  Serial.print("left0=");Serial.println(left0);
  Serial.print("right0=");Serial.println(right0);
  Serial.print("right_new=");Serial.println(right_new); 
  Serial.print("front=");Serial.println(front_new);
  front_old=front_new;
  left_old=left_new;
  right_old=right_new;
  Serial.println("---------ffee--------");

          mpu.update();
            if((millis()-timer)>10){
              angle=mpu.getAngleZ();
              Serial.print("direction=");Serial.println(angle);
              timer = millis();
              }
            Serial.println("---------ff--------");

          expected_angle=corner_counter*85*dir;
          real_angle=((int)(angle));
        if(real_angle>expected_angle){
            diff=real_angle-expected_angle;
            myservo.write(90+diff);
          }

          else if(real_angle<expected_angle){
            diff=expected_angle-real_angle;
            myservo.write(90-diff);
          }
 

}//end of while
          
    Serial.print("after break front=");
    Serial.println(front_new);
    Serial.print("state=");Serial.println(state);
    
    
      if(state=="rl1"){
        Serial.println("----------going to rl1---------");
        rl1(angle);
      }

      else if(state=="rr1"){
        Serial.println("----------going to rr1---------");
        rr1(angle);
      }
      else if(state=="finish"){Serial.println("congratulations!!!!");}
      ////////
      left_old=left_new;
      right_old=right_new;
      ///////
      
  //////
  }//end of function ff

///////////////////

 void rl1(double a){
  double angle;
  unsigned long timer = 0;
  myservo.write(60);
  delay(20);
  double b=a+70;
  forword(min_speed);

  ////////////////////

  while(true){
    //////
    mpu.update();
    if((millis()-timer)>5){
      angle=mpu.getAngleZ();
      Serial.println("direction=");Serial.println(angle);
      Serial.println("rl1-------------------");
      timer = millis();
      }
    ///////
    if(angle>=b){stop(); real_angle=mpu.getAngleZ();break;}
    }
    Serial.print("aftre break angle=");Serial.println(angle);
    ff();
  }  


  void rr1(double a){
  double angle;
  unsigned long timer = 0;
  myservo.write(120);
  delay(20);
  double b=a-70;
  Serial.print("angle b=");Serial.println(b);
  forword(min_speed);
  ////////////////////
  while(true){
    //////
    mpu.update();
    if((millis()-timer)>5){
      angle=mpu.getAngleZ();
      Serial.println("direction=");Serial.println(angle);
      Serial.println("rr1-------------------");
      timer = millis();
      }
    ///////
    if(angle<=b){stop();real_angle=mpu.getAngleZ();break;}
    }
    Serial.print("aftre break angle=");Serial.println(angle);
    ff();
  }  
  
void setup() {
  
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(2,OUTPUT);

  pinMode(motor2pin1_in3, OUTPUT);
  pinMode(motor2pin2_in4, OUTPUT);

  pinMode(ENB, OUTPUT);
  

  myservo.attach(pin_servo);  // attaches the servo on pin 9 to the servo object
  delay(20);

  Serial.begin(9600); // Starts the serial communication
  stop();
  straight();
  delay(5);
  Serial.print("left0=");Serial.println(left0);//;left0=dis_left();

  left0=dis_left();
  while(left0==0){stop();Serial.print("while left0");left0=dis_left();}
 
  right0= dis_right();
  while(right0==0)right0= dis_right();
  front0=dis_front();

  state="ff";
  state_old="ff";

  front_old=front0;
  right_old=right0;
  left_old=left0;

  delay(10000);
  Serial.println("Iam ready");
/////////////////
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
/////////////
  ff();
}

void loop() {

}
