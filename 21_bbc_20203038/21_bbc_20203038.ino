#include <Servo.h>

//Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
int a, b; // unit: mm
// configurable parameters
// Framework setting
#define _DIST_TARGET 327   // (unit: mm)
#define _DIST_MIN 100 
#define _DIST_MAX 300 

// Servo range 
#define _DUTY_MIN 1000 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1200 // servo neutral position (90 degree)
#define _DUTY_MAX 1400 // servo full counterclockwise position (180 degree)

Servo myservo;

void setup() {
  // initialize serial port
  Serial.begin(57600);
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);

  a = 70;
  b = 227;
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if(dist_cali > _DIST_TARGET) myservo.writeMicroseconds(_DUTY_MIN);
  else myservo.writeMicroseconds(_DUTY_MAX);
  delay(20);
}
