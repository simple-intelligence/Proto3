#include <Servo.h> 
 
Servo myservo0;
Servo myservo1;
Servo myservo2;
Servo myservo3;
 
int pos = 0;
 
void setup() 
{ 
    Serial.begin (9600);
    
    myservo0.attach(3);
    myservo1.attach(4);
    myservo2.attach(5);
    myservo3.attach(6);
    
    pinMode (7, OUTPUT);
    digitalWrite (7, HIGH);
} 
 
 
void loop() 
{
    if (Serial.available())
    {
        pos = Serial.parseInt ();
        Serial.println (pos);
        
        myservo0.writeMicroseconds(pos);
        myservo1.writeMicroseconds(pos);
        myservo2.writeMicroseconds(pos);
        myservo3.writeMicroseconds(pos);
        
        delay(15);
    }
} 
