// defines pins numbers
const int trigPin = 24;
const int echoPin = 25;
const int averageNumber = 20;

// defines variables
long duration;
int distance, distanceLast;
int distanceSmooth = 0;
int index = 0;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 24000);

  // Calculating the distance
  distance = duration * 0.034/2;
  
  if(distance == 0){
    distance = distanceLast;
  }
  distanceLast = distance;
  
  Serial.print("Distance: ");
  Serial.println(distance);
  index ++;
}
