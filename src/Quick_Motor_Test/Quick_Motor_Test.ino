// Strap shit down when you do this
#include <Servo.h> 
 
Servo myservo0;
Servo myservo1;
Servo myservo2;
Servo myservo3;
 
int pos = 0;
 
void setup() 
{ 
    Serial.begin (9600);
    
    myservo0.attach(2);
    myservo1.attach(3);
    myservo2.attach(4);
    myservo3.attach(5);
    
    pinMode (6, OUTPUT);
    digitalWrite (6, HIGH);
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
