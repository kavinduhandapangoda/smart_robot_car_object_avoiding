/*
  Thingerbits Smart turtle kit
  Sample code 1
  Object avoiding turtle
  Visit www.Thingerbits.lk for more details about kits and training programmes
*/


#include <Servo.h>   // servo library including

#define trig 2      //  intialinzing trig pin
#define echo 10     //  intialinzing echo pin

Servo servo1;       

int servopin = 11;   // servo pin
 
   int value;
   //-------------------------------------- L293 pinout----------------------------------------------------------------------------------------------------------------- 
   const int motorA1      = 3;  
   const int motorA2      = 4; 
   const int motorAspeed  = 5;
   const int motorB1      = 7; 
   const int motorB2      = 8; 
   const int motorBspeed  =6;

   //----------------------------------------- robot speed variables---------------------------------------------------------------------------------------------------------
   int vSpeed = 90;        
   int turn_speed = 130;
   

void setup() {
   servo1.attach(servopin);
   pinMode(trig, OUTPUT);
   pinMode(echo, INPUT);
  Serial.begin(9600);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

}

void loop() {

     forward();                       // car is running forward

     servo1.write(70);                // checking barriers
     delay(100);
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long t=pulseIn(echo, HIGH);
    long cm = (t/2)/29.1;                 //display distance at serial moniter for devoloping
    Serial.print(cm);

       
    if (cm > 35){                        // checking distances
      forward();                         // if the distance is large than 35 allow car to run forward
      Serial.println ("forward");
        }
        
          
     if (cm < 15){
      backward();                        // if the distance is lower than 35, run backward and stop the car 
      delay(500);
      Serial.println ("block");
      st();                              

         
                                          //Measure right distance
      servo1.write(0);
      delay(1000);
     digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long t1=pulseIn(echo, HIGH);
    long cm1 = (t1/2)/29.1;
    Serial.print(cm1);
    Serial.println("right cm ");

             
                                           //Measure left distance    
    servo1.write(140);
    delay(1000);
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long l=pulseIn(echo, HIGH);
    long cm3 = (l/2)/29.1;
    Serial.print(cm3);
    Serial.println("left cm ");

      
      servo1.write(70);

      if (cm1 > 70 ){                     // checking right distance
         Tright();
         Serial.println("Tright");
         delay(200);
                                           //turning car right
      }
      else if(cm3 > 70){
        Tleft();                         // checking left distance
        Serial.println("Tleft");
        delay(200);
                                         //turning car right
        }
              }
                     }

 //----------------------------------------turtle moving functions--------------------------------------------------------------------------------------------------                  

void backward(){
   digitalWrite (motorA1,LOW);
   digitalWrite (motorA2,HIGH);                       
   digitalWrite (motorB1,LOW);
   digitalWrite (motorB2,HIGH);

  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, vSpeed);

   }

  void forward(){
  digitalWrite (motorA1,HIGH);
   digitalWrite (motorA2,LOW);                       
   digitalWrite (motorB1,HIGH);
   digitalWrite (motorB2,LOW);

  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, vSpeed);
  }

void Tleft(){
   digitalWrite (motorA1,HIGH);
   digitalWrite (motorA2,LOW);                       
   digitalWrite (motorB1,LOW);
   digitalWrite (motorB2,HIGH);

  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, vSpeed);
   }  

  void Tright(){
   digitalWrite (motorA1,LOW);
   digitalWrite (motorA2,HIGH);                       
   digitalWrite (motorB1,HIGH);
   digitalWrite (motorB2,LOW);

  analogWrite (motorAspeed, vSpeed);
  analogWrite (motorBspeed, vSpeed);
  }

  void st(){
  analogWrite (motorAspeed, 0);
  analogWrite (motorBspeed, 0);
    }

