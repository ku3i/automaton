#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//#include <VL53L0X.h>
#include "Adafruit_VL53L0X.h"
#include <ESP32Servo.h>

// for the audio DFplayer
#include "DFRobotDFPlayerMini.h"


#define FPSerial Serial1

#define WAIT_MS 10
#define TARGET_LOW 1425

Adafruit_MPU6050 mpu;
//VL53L0X sensor;

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();
Servo esc0;

DFRobotDFPlayerMini audio;

const uint8_t LEDYWL = D10;
const uint8_t BUTTON = D8;
const uint8_t POTI_A = A0;
const uint8_t POTI_B = A1;
const uint8_t PWMPIN = D3;

int button_integ = 0;
int target_lp = TARGET_LOW;

bool armed = false;






unsigned idle_counter = 0;

/* proximity sensing */
const float dist_max_mm = 1200.0f;
float dist_mm = 1200.f;
float proximity_detected = 0.f;
float proximity_into = 500.f;
float proximity_left = 700.f;
float proximity_decay = 0.95f;
float persistence = -1.f;
float rotation = 0.f;
float e = 0.f;

bool state_sensor_active = false;
bool state_motor_start = false;
bool state_idle = false;

/* audio events: */
enum Sound_t : uint8_t {
  s_done          = 0,
  s_sensor_active = 1,
  s_sensor_clear  = 2,
  s_motor_start   = 3,
  s_motor_fast    = 4,
  s_stop_idle     = 6,
} playsound = s_done;



void printDetail(uint8_t type, int value);


/* clamps values to Interval [lo, hi] */
template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi )
{
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

bool button_pressed(uint8_t N = 3)
{
  const bool buttonstate = !digitalRead(BUTTON);

  if (buttonstate)
    button_integ += 1;
  else 
    button_integ = 0;

  button_integ = clamp(button_integ, 0, 2*N);

  return button_integ>=N;
}

bool button_released() { 
  const bool buttonstate = !digitalRead(BUTTON);

  if (!buttonstate) 
    button_integ = 0;

  return buttonstate == 0;
}

void setup()
{
  /* LED and Button */
  pinMode(LEDYWL, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  
  /* Serial Ports */
  Serial.begin(115200);
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/D7, /*tx =*/D6);

  delay(100);

  /* Audio */
  if (!audio.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println("Cannot initialize DFPlayer.");
    while(1) delay(0);
  }
  Serial.println("DFPlayer initialized.");
  delay(100);
  audio.setTimeOut(1000);
  audio.volume(30);  // set volume value [0...30]
  audio.playFolder(2, 3);


  /* MPU */
  if (!mpu.begin()) {
    Serial.println("Cannot initialize MPU6050.");
    while (1) delay(10);
  }
  Serial.println("MPU6050 initialized.");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(100);

  /* Distance Sensor */
  if (!sensor.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
  delay(100);
  sensor.startRangeContinuous();
  //sensor.init();
  //sensor.setTimeout(500);

  /* Motor */
  pinMode(PWMPIN, OUTPUT);
  esc0.attach(PWMPIN, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  Serial.println("Motor initialized.");

  delay(1000);
}


float db(float x, float t) {
  return fabs(x) > t ? x : 0;
}

void reset_idle() {
  idle_counter = 0;
  state_idle = false;
}

void loop() 
{
  static unsigned long timer = millis();

  /* timing */
  digitalWrite(LEDYWL, !digitalRead(LEDYWL));
  
  while(millis() - timer < WAIT_MS) delay(1);
  timer = millis();
  
  /* read distance sensor */
  
  if (sensor.isRangeComplete())
  {
    //int s = sensor.readRangeSingleMillimeters();
    int s = sensor.readRange();
    dist_mm = clamp(s - 50.f, 0.f, dist_max_mm); //in mm
  }
  
 
  if (dist_mm < proximity_into) proximity_detected = 1.0f;
  if (dist_mm > proximity_left) proximity_detected *= proximity_decay;  

  /* get new motion sensor readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* read and clamp potentiometers */
  unsigned pot_a = analogRead(POTI_A);
  unsigned pot_b = analogRead(POTI_B);
  float val_a = clamp(pot_a, 0u, 4095u) / 4095.f; //[0,1] 0.5 is super
  float val_b = clamp(pot_b, 0u, 4095u) / 4095.f; //[0,1] 0.5 is best

  /* get button state */
  bool state = digitalRead(BUTTON);
    
  /* calculate the gyroscopic input */ 
  rotation = db(g.gyro.x, 10*val_b) == 0.0f ? 1.0 : 0.9 * rotation; /*turning thresh*/
  float e_y = clamp(100 * db(-g.gyro.z, 0.1/*motion thresh*/), 0.f, 1.0f);
  e = rotation * e_y;

  
  /* check for arming the ESC */
  if (button_pressed())
  {
    if (!armed)
    {
      Serial.printf("motor unlocked");
      esc0.writeMicroseconds(1000);
      delay(1000);
      esc0.writeMicroseconds(500);
      delay(500);
      armed = true;
      persistence = -1.0;
      while (not button_released());
    } else 
    {
      Serial.printf("motor stopped");
      esc0.writeMicroseconds(500);
      armed = false;
      while (not button_released());
    }
  }

  /* create control outputs */
  float control = 0;

  if (proximity_detected > 0.5)
  {
    control = proximity_detected;
    persistence += 0.02;
  } 
  else {
    persistence -= 0.01;
    control = e * clamp(persistence, 0.f, 1.0f); //keep going
  }

  persistence = clamp(persistence, -1.0f, 50.0f);

  //if (idle_counter > 200)
  //  control = e * 0.2;  does not work
  
  int target_us = val_a*control*200 + TARGET_LOW;

  /* low-pass filter the output */
  float k = 0.96; 
  target_lp = k*target_lp + (1.f-k)*target_us;

  /* set motor output */
  if (armed) {
    esc0.writeMicroseconds(target_lp);
  }

  
  // SENSOR ACTIVE
  if (proximity_detected == 1.0f and !state_sensor_active) {
    state_sensor_active = true;
    playsound = Sound_t::s_sensor_active;
    reset_idle();
  }
    
  // SENSOR CLEAR  
  if (proximity_detected < 0.99f and state_sensor_active) {
    state_sensor_active = false;
    playsound = Sound_t::s_sensor_clear;
    reset_idle();
  }

  // MOTOR START
  if (persistence > 0.f and !state_motor_start) {
    state_motor_start = true;
    //playsound = Sound_t::s_motor_start;
    reset_idle();
  }

  // STOP / IDLE
  /*if (control < 0.01f and state_motor_start) {
    state_motor_start = false;
    reset_idle();
  }*/

  if (++idle_counter > 100 and not state_idle) {
    playsound = Sound_t::s_stop_idle;
    state_idle = true;
  }
  

  /* play audio events */
  switch(playsound) {
  case s_sensor_active: audio.playFolder(2, 1); /*audio.disableLoop();*/ playsound = s_done; break;
  case s_sensor_clear : audio.playFolder(2, 2); /*audio.disableLoop();*/ playsound = s_done; break;
  case s_motor_start  : /*audio.playFolder(2, 3);*/ /*audio.disableLoop();*/ playsound = s_done; break;
  case s_motor_fast   : /*audio.playFolder(2, 4); playsound = s_done; */ break;
  case s_stop_idle    : audio.playFolder(1, 3); /*audio.enableLoop();*/ playsound = s_done; break;
  
  /* no action if done */
  case s_done:
  default: 
      break;
  }

  /* check player status */
  if (audio.available())
      printDetail(audio.readType(), audio.read());

 /* print to serial console */
  Serial.printf("B:%u, p={%4.2f, %4.2f} c=%4.2f esc=%d s=%4.0f prox=%4.2f rot=%4.2f [%u:%u:%u] per=%4.2f\n"
               , state, val_a, val_b, control, target_lp, dist_mm, proximity_detected, rotation, state_sensor_active, state_motor_start, state_idle, persistence); 
}


void printDetail(uint8_t type, int value) 
{
  switch (type) {
    case TimeOut:              Serial.println("Time Out!");      break;
    case WrongStack:           Serial.println("Stack Wrong!");   break;
    case DFPlayerCardInserted: Serial.println("Card Inserted!"); break;
    case DFPlayerCardRemoved:  Serial.println("Card Removed!");  break;
    case DFPlayerCardOnline:   Serial.println("Card Online!");   break;
    case DFPlayerUSBInserted:  Serial.println("USB Inserted!");  break;
    case DFPlayerUSBRemoved:   Serial.println("USB Removed!");   break;
    case DFPlayerPlayFinished: Serial.println("Play Finished!"); break;
    case DFPlayerError:        
      Serial.print("DFPlayerError:"); 
      switch (value) {
        case Busy:             Serial.println("Card not found");          break;
        case Sleeping:         Serial.println("Sleeping");                break;
        case SerialWrongStack: Serial.println("Get Wrong Stack");         break;
        case CheckSumNotMatch: Serial.println("Check Sum Not Match");     break;
        case FileIndexOut:     Serial.println("File Index Out of Bound"); break;
        case FileMismatch:     Serial.println("Cannot Find File");        break; 
        case Advertise:        Serial.println("In Advertise");            break;
        default: break;
      }
      break;
    default: break;
  }
}

  