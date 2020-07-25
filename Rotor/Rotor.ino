#include <Servo.h>
int potValue = 0;
String val; // variable to receive data from the serial port
Servo ESC1, ESC2, ESC3, ESC4; //Name the Servo

void setup() {
	Serial.begin(9600);
//	Serial3.begin(9600);
	ESC1.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
	ESC2.attach(10,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
	ESC3.attach(11,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
	ESC4.attach(12,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
}

void loop() {
	if( Serial.available() ) // if data is available to read
	{
		// val = Serial3.read(); // read it and store it in 'val'
		potValue = Serial.parseInt();
		Serial.println(potValue);
	}

//	ESC1.writeMicroseconds(potValue);
	ESC2.writeMicroseconds(potValue);
//	ESC3.writeMicroseconds(potValue);
//	ESC4.writeMicroseconds(potValue);
}
