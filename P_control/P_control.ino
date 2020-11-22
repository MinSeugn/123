#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9      
#define PIN_SERVO 10 
#define PIN_IR A0 

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100 
#define _DIST_MAX 410

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

// Servo range 
#define _DUTY_MIN 850  
#define _DUTY_NEU 1200  
#define _DUTY_MAX 1650   

// Servo speed control
#define _SERVO_ANGLE 30  
#define _SERVO_SPEED 1000   

// Event periods
#define _INTERVAL_SERVO 20  
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 1.7

// ir_filter
#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.35     // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.
float dist_ema=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

//////////////////////
// global variables //
//////////////////////

// Servo instance   
Servo myservo;          

// Distance sensor
float dist_target; 
float dist_raw;
float dist_cali;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   
bool event_dist, event_servo, event_serial; 
// Servo speed control
int duty_chg_per_interval;  
int duty_target, duty_curr, duty_neutral;
int dist_min, dist_max;    

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 

//ir_cali
const float coE[] = {-0.0000461, 0.0304732, -4.4440694, 332.9559901};


void setup() {
// initialize GPIO pins for LED and attach servo 
pinMode(PIN_LED,OUTPUT);    
myservo.attach(PIN_SERVO);
duty_target = duty_curr = _POS_START;
duty_neutral = _DUTY_NEU;

// initialize global variables
dist_min = _DIST_MIN;
dist_max= _DIST_MAX;
dist_target = _DIST_TARGET;  

// move servo to neutral position
myservo.writeMicroseconds(_DUTY_NEU);

// initialize serial port
Serial.begin(57600); 

// initialize last sampling time
  last_sampling_time_dist = last_sampling_time_serial = last_sampling_time_servo = 0;
  event_dist = event_serial = event_servo = false;

// convert angle speed into duty change per interval.
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }


////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
     event_dist = false;  
  // get a distance reading from the distance sensor
     dist_cali = filtered_ir_distance(); 

  // PID control logic
    error_curr = dist_target - dist_cali; 
    pterm =  error_curr;
    control = _KP * pterm;

  // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral + control; 

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]

  }
  
  if(event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval 
     if(duty_target > duty_curr) {
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target) duty_curr = duty_target;
      }
      else {
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
  }

    // update servo position
  myservo.writeMicroseconds(duty_curr);

  }
  
  if(event_serial) {
    event_serial = false; 
    Serial.print("dist_ir:");
    Serial.print(dist_cali);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val; 
  float volt = float(analogRead(PIN_IR)); 
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; 
  return val; 
}
float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  dist_ema = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*dist_ema;
  dist_cali = coE[0] * pow(dist_ema, 3) + coE[1] * pow(dist_ema, 2) + coE[2] * dist_ema + coE[3];
  
  return dist_cali;
}
