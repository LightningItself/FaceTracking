#include <Servo.h>
Servo servo_x;
Servo servo_y;

char xy;
int spd;

char mode; // Holds the mode (D, A)
int pin_number; // Holds the pin number

int wait_for_transmission = 5; // Delay in ms in order to receive the serial data

void setup() {
    Serial.begin(9600); // Serial Port at 9600 baud
    Serial.setTimeout(100); // Instead of the default 1000ms, in order
    servo_x.attach(9);
    servo_y.attach(10); 
    servo_x.write(90);
    servo_y.write(90);
}

void loop() {
  
    // Check if characters available in the buffer
    if (Serial.available() > 0) {
        xy = Serial.read();
        delay(wait_for_transmission);
        spd = Serial.parseInt(); // Waits for an int to be transmitted
        if(xy=='X')
            servo_x.write(spd);
        else
            servo_y.write(spd);
        
        
          
    }
}