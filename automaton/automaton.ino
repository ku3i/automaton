#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>


#define WAIT_MS 20
#define TARGET_LOW 1425

Adafruit_MPU6050 mpu;
VL53L0X sensor;

Servo esc0;

const uint8_t LEDYWL = D10;
const uint8_t BUTTON = D8;
const uint8_t POTI_A = A0;
const uint8_t POTI_B = A1;
const uint8_t PWMPIN = D3;

const unsigned max_cycles = 4000/WAIT_MS; // num cycles per 2 seconds

int button_integ = 0;
int target_lp = TARGET_LOW;

bool armed = false;

unsigned cycles = 0;
unsigned wait_cycles = max_cycles;
unsigned waited = 0;

unsigned tick = 0;
float e=0;

float e_x =0;

/* proximity sensing */
const float dist_max_mm = 2000.0f;
float dist_mm = 0;
float proximity_detected = 0.f;
float proximity_into = 400;
float proximity_left = 500;
float proximity_decay = 0.95f;


/* clamps values to Interval [lo, hi] */
template<class T>
constexpr const T& clamp( const T& v, const T& lo, const T& hi )
{
    return (v < lo) ? lo : (hi < v) ? hi : v;
}

bool button_pressed(uint8_t N = 3)
{
  const bool buttonstate = !digitalRead(BUTTON);

  button_integ += buttonstate ? 1 : -1;
  button_integ = clamp(button_integ, 0, 2*N);

  return button_integ>=N;
}


void setup() {
  pinMode(PWMPIN, OUTPUT);
  pinMode(LEDYWL, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:  Serial.println("+-2G");  break;
  case MPU6050_RANGE_4_G:  Serial.println("+-4G");  break;
  case MPU6050_RANGE_8_G:  Serial.println("+-8G");  break;
  case MPU6050_RANGE_16_G: Serial.println("+-16G"); break;
  }

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:  Serial.println("+- 250 deg/s");  break;
  case MPU6050_RANGE_500_DEG:  Serial.println("+- 500 deg/s");  break;
  case MPU6050_RANGE_1000_DEG: Serial.println("+- 1000 deg/s"); break;
  case MPU6050_RANGE_2000_DEG: Serial.println("+- 2000 deg/s"); break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
  case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
  case MPU6050_BAND_94_HZ:  Serial.println("94 Hz");  break;
  case MPU6050_BAND_44_HZ:  Serial.println("44 Hz");  break;
  case MPU6050_BAND_21_HZ:  Serial.println("21 Hz");  break;
  case MPU6050_BAND_10_HZ:  Serial.println("10 Hz");  break;
  case MPU6050_BAND_5_HZ:   Serial.println("5 Hz");   break;
  }


  /* init distance sensor */
  sensor.init();
  sensor.setTimeout(500);

  Serial.println("");
  delay(100);


  esc0.attach(PWMPIN,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
  delay(1000);
  //esc0.writeMicroseconds(ARMING_MS); // arm esc

}


float db(float x, float t) {
  return fabs(x) > t ? x : 0;
}

void loop() 
{
  if (cycles % wait_cycles == 0)
    tick = 1;

  if (1==tick) {
    ++waited;
    if (waited >= wait_cycles/4){
      tick = 0;
      waited = 0;
    }
  }

  // replace by timer.
  digitalWrite(LEDYWL, HIGH);
  delay(WAIT_MS/2);
  digitalWrite(LEDYWL, LOW);
  delay(WAIT_MS/2);

  /* read distance sensor */
  int s = sensor.readRangeSingleMillimeters();

  float sinp_mm = clamp(s - 50.f, 0.f, dist_max_mm); //in mm
  float h = 0.25;
  dist_mm = sinp_mm * (1.0 - h) + dist_mm * h;

  if (dist_mm < proximity_into) proximity_detected = 1.0f;
  if (dist_mm > proximity_left) proximity_detected *= proximity_decay;  

  /* get new motion sensor readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //float A = 10 * clamp((float)(a.acceleration.y / 9.81f), 0.0f, 0.1f); // [0,1]
  //float G = clamp((float)(g.gyro.z * 10000), 0.0f, 1.0f); // [-1,1]

  /* read and clamp potentiometers */
  unsigned pot_a = analogRead(POTI_A);
  unsigned pot_b = analogRead(POTI_B);
  float val_a = clamp(pot_a, 0u, 4095u) / 4095.f; //[0,1]
  float val_b = clamp(pot_b, 0u, 4095u) / 4095.f; //[0,1]

  /* get button state */
  bool state = digitalRead(BUTTON);
    
  /* calculate the gyroscopic input */ 
  e_x = db(g.gyro.x, 10*val_b) == 0.0f ? 1.0 : 0.9 * e_x; /*turning thresh*/
  //float e_y = clamp(100 * db(-g.gyro.z, 0.1), 0.f, 1.0f);
  float e_y = clamp(100 * db(-g.gyro.z, 0.1/*motion thresh*/), 0.f, 1.0f);
  e = e_x * e_y;

  /* create wait cycles from potentiometer */
  wait_cycles = max_cycles * val_b;

  /* check for arming the ESC */
  if (!armed and button_pressed(3))
  {
    Serial.printf("Arming unlocked");
    esc0.writeMicroseconds(1000);
    delay(500);
    armed = true;
  }
  
  if (armed and button_pressed(5))
  {
    esc0.writeMicroseconds(500);
    delay(500);
  }

  /* create control outputs */
  float control = val_a * e * proximity_detected;
  int target_us = control*200 + TARGET_LOW;

  /* low-pass filter the output */
  float k = 0.94; 
  target_lp = k*target_lp + (1.f-k)*target_us;

  /* set motor output */
  if (armed) {
    esc0.writeMicroseconds(target_lp);
  }

  /* print to serial console */
  Serial.printf("B:%u, p={%4.2f, %4.2f} c=%4.2f esc=%d t=%u s=%4.0f prox=%4.2f\n"
               , state, val_a, val_b, control, target_lp, tick, dist_mm, proximity_detected );

  ++cycles;
}



  