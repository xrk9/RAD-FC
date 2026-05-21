// Flight Controller v3 by Rahul Kumar
// Project RAD-1


/* 
 v4  -  Derived from v3


 Summary : 
 v4 is path of v3
 6 months ago only god and I knew how this code worked, now only god knows

 Fixes (v3 patch):
 -Loop timing condition fixed as earlier it was infinite
 -Gyro axes fix
 -Clamping motor output to 1000,2000
 -Anti-windup fix
 -Reduced bring down drone if signal not receiving from 15s to 6 sec

*/


// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// Libraries
#include <Wire.h>             // for MPU communication
#include <RF24.h>             // for NRF24L01 communication
#include <Servo.h>            // for esc communication
#include <math.h>             // for trignometry and some other functions ( like sqrt )
#include <stdint.h>           // for standard integers
#include <Adafruit_BMP280.h>  // for BMP 280 communication


// Addresses for I2C
#define MPU_ADDR 0x68  // MPU 6050 Address
#define BMP_ADDR 0x76  // BMP 280 Address //try 0x77 for older bmp 280


// Arudino Pins
#define led_g 3  // green led pin
#define led_r 4  // red led pin
#define ce 7     // CE pin of NRF
#define csn 8    // CSN pin of NRF


// Motor Pins
#define fr 5   // front right
#define fl 6   // front left
#define br 9   // back right
#define bl 10  // back left

// Hover throttle
#define HOVER_THROTTLE 1500

// Loop Times
#define LOOP_0    4000   // Inner gyro/ESC loop:   1_000_000 / 250Hz = 4_000 µs
#define LOOP_1   20000   // Accel/Kalman/RC loop:  1_000_000 /  50Hz = 20_000 µs
#define LOOP_BARO 40000  // Baro Kalman loop:      1_000_000 /  25Hz = 40_000 µs

// ESC output clamp limits
#define ESC_MIN 1050
#define ESC_MAX 1950  // to avoid unexpected behavious at edge values

// Constant Values - tune via PID testing ------------------------------------------------------------------------------------------------------------------------------------------------------------
#define KP_T 50.0
#define KP_P 40.0
#define KP_R 40.0
#define KP_Y 70.0

#define KI_T 0.0
#define KI_P 0.0
#define KI_R 0.0

#define KD_T 0.0
#define KD_P 0.0
#define KD_R 0.0


// Adafruit Barometer Object
Adafruit_BMP280 bmp;

// Servo Object
Servo escFL, escFR, escBL, escBR;


// Loop timers -> to keep track of loop timing
unsigned long loop_0 = 0, loop_1 = 0, loop_baro = 0;

// Last Loop Timer (micros)
unsigned long last_0 = 0, last_1 = 0, last_baro = 0;


// Struct Declaration
struct KalmanA;             // First Kalman Filter
struct KalmanB;             // Second Kalman filter
struct Data_Package;        // Data package
struct Target;              // Target values
struct Last;                // last IMU values
struct Axis;                // Pitch, Roll n Throttle
struct Prev;                // Previous


// NRF24L01 radio
unsigned long lastReceiveTimeRadio = 0;  // last radio received time (millis)
int16_t ringBuzzerFor = 0;               // How long to ring buzzer (ms)


// Raw IMU (MPU 6050) sensor storage (int16_t)
int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;  // accel
int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;  // gyro

// Gyro and Accel Offsets (raw)
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
int16_t ax_offset = 137, ay_offset = -181, az_offset = -1346;  // update if needed

// Downward Inertial acceleration base
float baselineAccZInertial = 0.0f;  // Inertial Z-accel when vVelocity = 0, not always 1g


// Barometer
float baselinePressure = 0.0f;  // hPa
float lastAltitude = 0.0f;


// Bring Down bool
bool bringDown = false;


struct Axis{
  float err;
  float dTerm;
  float mem;
};

struct Prev{
  Axis pitch;
  Axis roll;
  Axis throttle;
};
Prev prev;

struct Last{
  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
};
Last last;


struct Target{
  float pitchAngle;   // pitch angle (rad)
  float pitchRate;    // pitch rate (rad/s)
  float pitchAcc;     // pitch accel (rad/s²)

  float rollAngle;    // roll angle (rad)
  float rollRate;     // roll rate (rad/s)
  float rollAcc;      // roll accel (rad/s²)

  float yawRate;      // yaw rate (rad/s)
  float yawAcc;       // yaw accel (rad/s²)

  float vVelocity;    // vertical velocity (m/s)
  float Acc;          // vertical acceleration (m/s²)
};
Target target;


struct Data_Package {
  byte j1PotX;        // left joystick x axis
  byte j1PotY;        // left joystick y axis
  byte j2PotX;        // right joystick x axis
  byte j2PotY;        // right joystick y axis
  byte pot1;          // left potentiometer
  byte pot2;          // right potentiometer
  byte tSwitch1;      // main switch (arm/disarm)
  byte tSwitch2;      // side switch
  byte j1B;           // left joystick button
};
Data_Package data;


struct KalmanB {
  float Q_velocity;
  float Q_bias;
  float R_baro;
  float velocity;
  float bias;
  float P[2][2];
};
KalmanB kalVel;

struct KalmanA {
  float Q_angle;    // process noise variance for the angle
  float Q_bias;     // process noise variance for the gyro bias
  float R_measure;  // measurement noise variance (from accelerometer)
  float angle;      // [rad] The angle estimated by the Kalman filter
  float bias;       // [rad/s] The gyro bias estimated by the Kalman filter
  float P[2][2];    // Error covariance matrix
};
KalmanA kalRoll, kalPitch;



// NRF24L01 Receiver Object and address
RF24 radio(ce, csn);
const byte address[5] = {'R', 'A', 'D', '-', '0'};


// Sign function
inline int sign(float x) {
  return (x > 0.0f) - (x < 0.0f);
}

// Clamp integer between lo and hi
inline int clampInt(int val, int lo, int hi) {
  if (val < lo) return lo;
  if (val > hi) return hi;
  return val;
}

// reset radio control values
void resetData() {
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.pot1 = 0;
  data.pot2 = 0;
}

// set up radio to start listening 
void startListening(){
  
  radio.begin();

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();

  radio.openReadingPipe(0, address);
  radio.startListening();
  radio.writeAckPayload(0, &ringBuzzerFor, sizeof(ringBuzzerFor));
  resetData();

}

// reads the radio signal and saves it in data;
// sends ack packet as buzzer buzz time, buzz for ringBuzzerFor ms;
// if connection lost for more than 1 second -> reset data;
// if connection lost for more than 15 seconds -> land down the drone;
void read_receiver() {

  int16_t buzzTimee = ringBuzzerFor;
  unsigned long lastTimeRadio = lastReceiveTimeRadio;

  // ack
  radio.writeAckPayload(0, &buzzTimee, sizeof(buzzTimee));
  ringBuzzerFor = 0;   // clear after loading

  // store data
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    lastReceiveTimeRadio = millis();
  }

  unsigned long currentTimeRadio = millis();

  // failsafe and bring down
  if (currentTimeRadio - lastTimeRadio > 1000) {
    resetData();

    if (currentTimeRadio - lastTimeRadio > 6000) {
      bringDown = true;
    }
  }
}


// begin BMP
bool beginBMP(){

  if(!bmp.begin(BMP_ADDR)){
    return false;
  }

  // Lower noise configuration
  bmp.setSampling(

    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,    // Temp oversampling
    Adafruit_BMP280::SAMPLING_X16,   // Pressure oversampling
    Adafruit_BMP280::FILTER_X16,     // Strong internal filter
    Adafruit_BMP280::STANDBY_MS_1    // ~1000 Hz internal loop

  );
  
  return true;
}


// MPU utility
void writeMPU(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();

}

// Set up registers and make MPU ready to read data
bool beginMPU(){

  Wire.begin();
  Wire.setWireTimeout(3000, true); // 3ms timeout reset bus if stuck
  Wire.setClock(400000); // fast I2C
  delay(200);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) return false;

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  if (available < 1) return false;

  if (Wire.read() != 0x68) return false;

  // Wake up
  writeMPU(0x6B, 0x00); // PWR_MGMT_1: clear sleep
  delay(10);

  // DLPF = 3 => ~42 Hz LPF
  writeMPU(0x1A, 0x03);

  // SMPLRT_DIV = 0 -> sample rate = gyro output rate / (1 + 0)
  writeMPU(0x19, 0x00);

  // Gyro config ±500 dps
  writeMPU(0x1B, 0x08);

  // Accel config ±8g
  writeMPU(0x1C, 0x10);

  delay(100);

  return true;

}

// Read all raw data from MPU (accel + temp + gyro = 14 bytes)
bool readMPUraw() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H

  if (Wire.endTransmission(false) != 0) return false;

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (available < 14) return false;

  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_ax = (int16_t)((hi << 8) | lo) - ax_offset;
  hi = Wire.read(); lo = Wire.read(); raw_ay = (int16_t)((hi << 8) | lo) - ay_offset;
  hi = Wire.read(); lo = Wire.read(); raw_az = (int16_t)((hi << 8) | lo) - az_offset;
  // temp high/low (skip)
  (void)Wire.read(); (void)Wire.read();
  hi = Wire.read(); lo = Wire.read(); raw_gx = (int16_t)((hi << 8) | lo) - gx_offset;
  hi = Wire.read(); lo = Wire.read(); raw_gy = (int16_t)((hi << 8) | lo) - gy_offset;
  hi = Wire.read(); lo = Wire.read(); raw_gz = (int16_t)((hi << 8) | lo) - gz_offset;

  return true;
}

// Read only gyro (6 bytes, faster for inner loop)
bool readGyroRaw(){

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H

  if (Wire.endTransmission(false) != 0) return false;

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
  if (available < 6) return false;

  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_gx = (int16_t)((hi << 8) | lo) - gx_offset;
  hi = Wire.read(); lo = Wire.read(); raw_gy = (int16_t)((hi << 8) | lo) - gy_offset;
  hi = Wire.read(); lo = Wire.read(); raw_gz = (int16_t)((hi << 8) | lo) - gz_offset;

  return true;
}

// Read only accel (6 bytes)
bool readAccelRaw(){

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H

  if (Wire.endTransmission(false) != 0) return false;

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
  if (available < 6) return false;

  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_ax = (int16_t)((hi << 8) | lo) - ax_offset;
  hi = Wire.read(); lo = Wire.read(); raw_ay = (int16_t)((hi << 8) | lo) - ay_offset;
  hi = Wire.read(); lo = Wire.read(); raw_az = (int16_t)((hi << 8) | lo) - az_offset;

  return true;
}


// Calculates gyro offsets, baseline inertial Z-accel, and baseline baro pressure
void calibrate() {

  const int calSamples = 2000;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  long sumAX = 0, sumAY = 0, sumAZ = 0;
  int validSamples = 0;
  delay(500);   // Let sensors settle

  while (validSamples < calSamples) {

    if (readMPUraw()) {
      sumGX += raw_gx;
      sumGY += raw_gy;
      sumGZ += raw_gz;

      sumAX += raw_ax;
      sumAY += raw_ay;
      sumAZ += raw_az;

      validSamples++;
    }
    delay(1);

  }

  // Average gyro offsets
  gx_offset = (int16_t)round((float)sumGX / validSamples);
  gy_offset = (int16_t)round((float)sumGY / validSamples);
  gz_offset = (int16_t)round((float)sumGZ / validSamples);

  // Average accel in g (±8g -> 1/4096)
  float ax = (float)sumAX / validSamples / 4096;
  float ay = (float)sumAY / validSamples / 4096;
  float az = (float)sumAZ / validSamples / 4096;

  // Calculate approximate resting angles
  float PitchAngle = atan2f(-ax, sqrtf(ay*ay + az*az));
  float RollAngle  = atan2f( ay, sqrtf(ax*ax + az*az));

  // Reference inertial Z acceleration at rest (removes mounting tilt from baseline)
  baselineAccZInertial = -sinf(PitchAngle)*ax
                        + cosf(PitchAngle)*sinf(RollAngle)*ay
                        + cosf(PitchAngle)*cosf(RollAngle)*az;

  // Reference pressure for relative altitude
  baselinePressure = bmp.readPressure() / 100.0f;  // hPa

}


// PT1 (first-order low-pass) filter
// prev: last filtered value, input: new raw value, dt: time step, fc: cutoff frequency (Hz)
float pt1(float prev, float input, float dt, float fc){

  float RC    = 0.1591549f / fc;  // 1 / (2*PI*fc)
  float alpha = dt / (RC + dt);
  return prev + alpha * (input - prev);

}



// Initialize struct values to zero
void Initialize(){

  last.ax = 0.0f;
  last.ay = 0.0f;
  last.az = 0.0f;

  last.gx = 0.0f;
  last.gy = 0.0f;
  last.gz = 0.0f;

  prev.pitch.dTerm = 0.0f;
  prev.pitch.err   = 0.0f;
  prev.pitch.mem   = 0.0f;

  prev.roll.dTerm = 0.0f;
  prev.roll.err   = 0.0f;
  prev.roll.mem   = 0.0f;

  prev.throttle.dTerm = 0.0f;
  prev.throttle.err   = 0.0f;
  prev.throttle.mem   = 0.0f;

}


// Initialise angle Kalman filter
void kalmanInitA(KalmanA &k) {

  k.Q_angle   = 0.001f;
  k.Q_bias    = 0.003f;
  k.R_measure = 0.08f;
  k.angle = 0.0f;
  k.bias  = 0.0f;
  k.P[0][0] = 0.0f; k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f; k.P[1][1] = 0.0f;

}

// Initialise velocity Kalman filter
void kalmanInitB(KalmanB &k) {

  k.Q_velocity = 0.10f;
  k.Q_bias     = 0.005f;
  k.R_baro     = 20.0f;
  k.velocity   = 0.0f;
  k.bias       = 0.0f;
  k.P[0][0] = 1.0f; k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f; k.P[1][1] = 1.0f;

}


// Kalman A - prediction step (gyro-driven)
float kalmanPredictA(KalmanA &k, float gyro, float dt) {

  float rate = gyro - k.bias;
  k.angle += dt * rate;

  k.P[0][0] += dt * (dt*k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Q_angle);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += k.Q_bias * dt;

  return k.angle;

}

// Kalman A - correction step (accel-driven)
void kalmanCorrectA(KalmanA &k, float accelAngle) {

  float y = accelAngle - k.angle;
  float S = k.P[0][0] + k.R_measure;
  if (S < 1e-6f) return;

  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  float P00 = k.P[0][0];
  float P01 = k.P[0][1];

  k.angle += K0 * y;
  k.bias  += K1 * y;

  k.P[0][0] -= K0 * P00;
  k.P[0][1] -= K0 * P01;
  k.P[1][0] -= K1 * P00;
  k.P[1][1] -= K1 * P01;

}


// Kalman B - prediction step (accel-driven vertical velocity)
float kalmanPredictB(KalmanB &k, float acc, float dt) {

  k.velocity += (acc - k.bias) * dt;

  k.P[0][0] += -dt*(k.P[0][1] + k.P[1][0]) + dt*dt*k.P[1][1] + k.Q_velocity;
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += k.Q_bias;

  return k.velocity;

}

// Kalman B - correction step (baro-driven)
void kalmanCorrectB(KalmanB &k, float v_baro) {

  float y = v_baro - k.velocity;
  float S = k.P[0][0] + k.R_baro;
  if (S < 1e-6f) return;

  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  float P00 = k.P[0][0];
  float P01 = k.P[0][1];

  k.velocity += K0 * y;
  k.bias     += K1 * y;

  k.P[0][0] -= K0 * P00;
  k.P[0][1] -= K0 * P01;
  k.P[1][0] -= K1 * P00;
  k.P[1][1] -= K1 * P01;

}



// Update Target angles, angular velocities, and accelerations from RC input
void getTarget(){
  
  // right pot: [1, 6] scale
  float Pot2 = ((float)data.pot2 * 0.0196078431372549f) + 1.0f;

  // pitch
  if (data.j2PotX == 127) target.pitchAngle = 0.0f;
  else target.pitchAngle = ((float)data.j2PotX * 0.000684442843919f) - 0.0872664625997f;  // [-5°,5°] in rad
  target.pitchAngle *= Pot2;  // [-30°,30°] in rad at max

  // roll
  if (data.j2PotY == 127) target.rollAngle = 0.0f;
  else target.rollAngle = ((float)data.j2PotY * 0.000684442843919f) - 0.0872664625997f;   // [-5°,5°] in rad
  target.rollAngle *= Pot2;   // [-30°,30°] in rad at max

  // angular rate limits (rad/s)
  target.pitchRate = (float)Pot2 * 0.209439510239f + 0.314159265359f;  // [30,90] deg/s in rad
  target.rollRate  = (float)Pot2 * 0.209439510239f + 0.314159265359f;

  // angular accel limits (rad/s²)
  target.pitchAcc = (float)Pot2 * 0.209439510239f + 0.837758040957f;   // [60,120] deg/s² in rad
  target.rollAcc  = (float)Pot2 * 0.209439510239f + 0.837758040957f;

  // left pot: [1, 10] scale
  float Pot1 = ((float)data.pot1 * 0.0352941176470588f) + 1.0f;

  // yaw rate (rad/s)
  if (data.j1PotY == 127) target.yawRate = 0.0f;
  else target.yawRate = ((float)data.j1PotY * 0.00273777137568f) - 0.349065850399f;  // [-20,20] deg/s in rad
  target.yawRate *= (float)(Pot1 * 0.0107843137255f + 1.0f);  // up to [-75,75] deg/s

  // vertical velocity (m/s)
  if (data.j1PotX == 127) target.vVelocity = 0.0f;
  else target.vVelocity = ((float)data.j1PotX * 0.0023529411764706f) - 0.3f;  // [-0.3,0.3] m/s
  target.vVelocity *= (float)Pot1 * 0.5f;  // up to [-1.5,1.5] m/s

  // accel limits
  target.yawAcc = (float)Pot1 * 0.116355283466f + 0.581776417331f;   // [40,100] deg/s²
  target.Acc    = (float)Pot1 * 0.4f;  // [0.4,4.0] m/s²

  // Bring Down failsafe
  if (bringDown){
    float last_altitude = lastAltitude;
    if      (last_altitude < 1.5f) target.vVelocity = -0.15f;
    else if (last_altitude < 5.0f) target.vVelocity = -0.3f;
    else                           target.vVelocity = -1.5f;
    target.Acc = 0.5f;
  }

}


/* ------------ S-curve rate controller working ------------------

  theta_measure, omega_measure   -> measured by sensors
  theta_target, omega_target     -> wanted theta and omega
  omega_max, alpha_max           -> max angular rate and accel allowed

  Error (e)     = theta_target - theta_measure
  omega_cap     = sqrt(2 * alpha_max * |e|)          -> braking omega so it stops at target
  omega_out     = sign(e) * min(omega_max, omega_cap) -> desired rate right now

  d_omega       = omega_out - omega_last             -> how much we want to change omega
  d_omega_max   = alpha_max * dt                     -> max change per step
  clamp(d_omega, +-d_omega_max)                      -> rate-limit the change
  omega_last    = clamp(omega_last + d_omega, +-omega_max)

 --------------------------------------------------------------- */

// S-curve rate controller: returns a rate-limited angular rate command toward target angle
float control_loop(float e, float alpha_max, float omega_max, float omega_last, float dt){

  float omega_cap = sqrtf(2.0f * alpha_max * fabsf(e));
  float omega_out = sign(e) * fminf(omega_cap, omega_max);

  float d_omega     = omega_out - omega_last;
  float max_d_omega = alpha_max * dt;

  if      (d_omega >  max_d_omega) d_omega =  max_d_omega;
  else if (d_omega < -max_d_omega) d_omega = -max_d_omega;

  float omega = omega_last + d_omega;
  if      (omega >  omega_max) omega =  omega_max;
  else if (omega < -omega_max) omega = -omega_max;

  return omega;

}


// Calibrate ESCs then arm them with a visual motor spin check
void armESC(){

  escFL.attach(fl);
  escFR.attach(fr);
  escBL.attach(bl);
  escBR.attach(br);

  // ESC calibration sequence
  escFL.writeMicroseconds(2000);
  escFR.writeMicroseconds(2000);
  escBL.writeMicroseconds(2000);
  escBR.writeMicroseconds(2000);
  delay(3000);

  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escBL.writeMicroseconds(1000);
  escBR.writeMicroseconds(1000);
  delay(3000);

  delay(1000);  // hold before arming

  // Brief spin for visual motor check
  escFL.writeMicroseconds(1025);
  escFR.writeMicroseconds(1025);
  escBL.writeMicroseconds(1025);
  escBR.writeMicroseconds(1025);
  delay(1000);

  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escBL.writeMicroseconds(1000);
  escBR.writeMicroseconds(1000);

}

// --------------------------------------------------------------------------------
      // SETUP
// --------------------------------------------------------------------------------
void setup(){

  // LEDs
  pinMode(led_g, OUTPUT);
  digitalWrite(led_g, LOW);
  pinMode(led_r, OUTPUT);
  digitalWrite(led_r, HIGH);

  // Start radio (NRF24L01)
  for (int i = 0; i < 10; i++){
    if (radio.isChipConnected()){
      startListening();
      break;
    }
  }

  // MPU and BMP init with retries
  bool mpu_ok = false;
  bool bmp_ok = false;

  for (int i = 0; i < 10; i++){
    if (!mpu_ok) mpu_ok = beginMPU();
    if (!bmp_ok) bmp_ok = beginBMP();
    if (mpu_ok && bmp_ok) break;
    delay(50);
  }

  // Error halt with blinking red LED
  if (!mpu_ok || !bmp_ok || !radio.isChipConnected()){
    while(1){
      delay(500);
      digitalWrite(led_r, LOW);
      delay(500);
      digitalWrite(led_r, HIGH);
    }
  }

  // Initialise state
  Initialize();

  // Initialise Kalman filters
  kalmanInitA(kalPitch);
  kalmanInitA(kalRoll);
  kalmanInitB(kalVel);

  // Wait for arm switch
  while(true){
    read_receiver();
    if (data.tSwitch1) break;
    delay(500);
  }

  // Calibrate sensors
  calibrate();

  // Arm ESCs with visual check
  armESC();
  digitalWrite(led_r, LOW);
  digitalWrite(led_g, HIGH);  // Ready for takeoff

  ringBuzzerFor = 1500;
  read_receiver();
  resetData();
  target.vVelocity = 0.0f;
  delay(1300);  // 1300ms + buffer ≈ 1500ms buzzer ring

  // Set initial sensor state
  lastAltitude = bmp.readAltitude(baselinePressure);
  last_baro = micros();
  readMPUraw();
  last_0 = last_1 = micros();

  // Accel: ±8g -> 4096 LSB/g -> scale = 1/4096 = 0.000244140625
  last.ax = (float)raw_ax * 0.000244140625f;
  last.ay = (float)raw_ay * 0.000244140625f;
  last.az = (float)raw_az * 0.000244140625f;

  // Gyro: ±500 dps -> 65.5 LSB/(°/s) -> to rad/s: 1/65.5 * PI/180 = 0.0002664624812205
  last.gx = (float)raw_gx * 0.0002664624812205f;
  last.gy = (float)raw_gy * 0.0002664624812205f;
  last.gz = (float)raw_gz * 0.0002664624812205f;

  // Initialise loop timers
  loop_0 = loop_1 = loop_baro = micros();

}


// -----------------------------------------------------------------------
// LOOP
// -----------------------------------------------------------------------
void loop(){

  unsigned long now = micros();


  // -----------------------------------------------------------------------
  // 50 Hz loop — RC read, accel Kalman correction, vertical speed prediction
  // -----------------------------------------------------------------------
  if ((now - loop_1) >= LOOP_1) {
    loop_1 += LOOP_1;

    read_receiver();

    // Kill check — disarm and wait for re-arm
    if (!data.tSwitch1){

      escFL.writeMicroseconds(1000);
      escFR.writeMicroseconds(1000);
      escBL.writeMicroseconds(1000);
      escBR.writeMicroseconds(1000);
      digitalWrite(led_r, HIGH);

      while(true){
        read_receiver();
        if (data.tSwitch1) break;
      }

    }

    getTarget();

    readAccelRaw();

    unsigned long currTime = micros();
    float dt = (currTime - last_1) * 1e-6f; 
    last_1 = currTime;

    // Accel: ±8g -> 1/4096 = 0.000244140625
    float ax = (float)raw_ax * 0.000244140625f;
    float ay = (float)raw_ay * 0.000244140625f;
    float az = (float)raw_az * 0.000244140625f;

    if (dt <= 0.06f && dt > 0.0f){

      float filax = pt1(last.ax, ax, dt, 2.0f);
      float filay = pt1(last.ay, ay, dt, 2.0f);
      float filaz = pt1(last.az, az, dt, 2.0f);

      last.ax = filax;
      last.ay = filay;
      last.az = filaz;

      // Compute accel angles
      float pitchAng = atan2f( filay, sqrtf(filax*filax + filaz*filaz));
      float rollAng  = atan2f(-filax, sqrtf(filay*filay + filaz*filaz));

      // Inertial vertical acceleration (remove gravity and mounting tilt)
      float accZInertial = -sinf(pitchAng)*filax
                          + cosf(pitchAng)*sinf(rollAng)*filay
                          + cosf(pitchAng)*cosf(rollAng)*filaz;
      accZInertial = (accZInertial - baselineAccZInertial) * 9.81f;

      // Correct angle Kalman filters with accel measurement
      kalmanCorrectA(kalRoll,  rollAng);
      kalmanCorrectA(kalPitch, pitchAng);

      // Predict vertical velocity with inertial accel
      kalmanPredictB(kalVel, accZInertial, dt);

    } else {

      // dt out of range: soft-update with fixed coefficient
      last.ax += 0.2f * (ax - last.ax);
      last.ay += 0.2f * (ay - last.ay);
      last.az += 0.2f * (az - last.az);

    }

    now = micros();

  }


  // -----------------------------------------------------------------------
  // 25 Hz loop — Baro Kalman correction
  // -----------------------------------------------------------------------
  if ((now - loop_baro) >= LOOP_BARO) {
    loop_baro += LOOP_BARO;

    float curr = bmp.readAltitude(baselinePressure);

    unsigned long currentTime = micros();
    float dt = (currentTime - last_baro) * 1e-6f;  // FIX: was double
    last_baro = currentTime;

    if (dt <= 0.12f && dt > 0.0f){

      float last_alt  = lastAltitude;
      float curr_fil  = pt1(last_alt, curr, dt, 0.5f);
      lastAltitude    = curr_fil;

      float vel_raw   = (curr_fil - last_alt) / dt;
      float vel_fil   = pt1(kalVel.velocity, vel_raw, dt, 2.0f);

      // Correct vertical velocity Kalman with baro-derived velocity
      kalmanCorrectB(kalVel, vel_fil);

    } else {

      lastAltitude += 0.11f * (curr - lastAltitude);

    }

    now = micros();

  }


  // -----------------------------------------------------------------------
  // 250 Hz inner loop — Gyro Kalman prediction + PID + ESC output
  // -----------------------------------------------------------------------
  if ((now - loop_0) >= LOOP_0) {
    loop_0 += LOOP_0;

    readGyroRaw();

    unsigned long currTime = micros();
    float dt = (currTime - last_0) * 1e-6f;  // FIX: was double
    last_0 = currTime;

    // Gyro: ±500 dps -> rad/s: 1/65.5 * PI/180 = 0.0002664624812205
    float gx = (float)raw_gx * 0.0002664624812205f;
    float gy = (float)raw_gy * 0.0002664624812205f;
    float gz = (float)raw_gz * 0.0002664624812205f;

    if (dt <= 0.012f && dt > 0.0f){

      // PT1 filter gyro at 30 Hz
      float fil_gx = pt1(last.gx, gx, dt, 30.0f);
      float fil_gy = pt1(last.gy, gy, dt, 30.0f);
      float fil_gz = pt1(last.gz, gz, dt, 30.0f);

      last.gx = fil_gx;
      last.gy = fil_gy;
      last.gz = fil_gz;

      // Predict angles with gyro (gx drives roll, gy drives pitch)
      kalmanPredictA(kalRoll,  fil_gx, dt);
      kalmanPredictA(kalPitch, fil_gy, dt);


      // ---- Flight Controller ----

      // D-term filter coefficient
      float alpha_inv = 1.0f / (1.0f + 94.0f * dt);


      // ---- PITCH ----

      // S-curve rate command
      float pitch_error = target.pitchAngle - kalPitch.angle;
      float pitch_omega = control_loop(pitch_error, target.pitchAcc, target.pitchRate, fil_gy, dt);
      float p_err = pitch_omega - fil_gy;

      // P term
      float p_pitch = KP_P * p_err;

      // I term (anti-windup applied below)
      float p_step = p_err * KI_P * dt;
      float p_i_potential = prev.pitch.mem + p_step;

      // D term with PT1 filter (~15 Hz cutoff)
      float tmp_p  = prev.pitch.dTerm + KD_P * 94.0f * (p_err - prev.pitch.err);
      float d_pitch = tmp_p * alpha_inv;
      prev.pitch.dTerm = d_pitch;
      prev.pitch.err   = p_err;

      // Pitch command
      float pitch = p_pitch + p_i_potential - d_pitch;


      // ---- ROLL ----

      // S-curve rate command
      float roll_error = target.rollAngle - kalRoll.angle;
      float roll_omega = control_loop(roll_error, target.rollAcc, target.rollRate, fil_gx, dt);
      float r_err = roll_omega - fil_gx;

      // P term
      float p_roll = KP_R * r_err;

      // I term (anti-windup applied below)
      float r_step = r_err * KI_R * dt;
      float r_i_potential = prev.roll.mem + r_step;

      // D term with PT1 filter
      float tmp_r  = prev.roll.dTerm + KD_R * 94.0f * (r_err - prev.roll.err);
      float d_roll = tmp_r * alpha_inv;
      prev.roll.dTerm = d_roll;
      prev.roll.err   = r_err;

      // Roll command
      float roll = p_roll + r_i_potential - d_roll;


      // ---- YAW ----

      float target_yaw  = target.yawRate;
      float yaw_error   = target_yaw - fil_gz;

      // Rate-limit yaw error step
      float max_yaw_err = target.yawAcc * dt;
      if      (yaw_error >  max_yaw_err) yaw_error =  max_yaw_err;
      else if (yaw_error < -max_yaw_err) yaw_error = -max_yaw_err;

      float yaw_cmd = fil_gz + yaw_error;
      if (fabsf(yaw_cmd) > fabsf(target_yaw)) yaw_cmd = target_yaw;

      // Yaw command
      float yaw = KP_Y * (yaw_cmd - fil_gz);


      // ---- VERTICAL VELOCITY (THROTTLE) ----

      float target_vz  = target.vVelocity;
      float vz_now     = kalVel.velocity;
      float vz_error   = target_vz - vz_now;

      float max_vz_err = target.Acc * dt;
      if      (vz_error >  max_vz_err) vz_error =  max_vz_err;
      else if (vz_error < -max_vz_err) vz_error = -max_vz_err;

      float vz = vz_now + vz_error;
      if (fabsf(vz) > fabsf(target_vz)) vz = target_vz;

      float v_err = vz - vz_now;

      // P term
      float p_throttle = KP_T * v_err;

      // I term (anti-windup applied below)
      float v_step = v_err * KI_T * dt;
      float v_i_potential = prev.throttle.mem + v_step;

      // D term with PT1 filter
      float tmp_v  = prev.throttle.dTerm + KD_T * 94.0f * (v_err - prev.throttle.err);
      float d_vel  = tmp_v * alpha_inv;
      prev.throttle.dTerm = d_vel;
      prev.throttle.err   = v_err;

      // Throttle command
      float throttle = p_throttle + v_i_potential - d_vel;


      // ---- Motor mixing (X-frame) ----
      int mFR = (int)(HOVER_THROTTLE + throttle - roll - pitch - yaw);
      int mBR = (int)(HOVER_THROTTLE + throttle - roll + pitch + yaw);
      int mBL = (int)(HOVER_THROTTLE + throttle + roll + pitch - yaw);
      int mFL = (int)(HOVER_THROTTLE + throttle + roll - pitch + yaw);

      int maxMotor = max(max(mFR, mBR), max(mBL, mFL));
      int minMotor = min(min(mFR, mBR), min(mBL, mFL));

      bool highSat = (maxMotor > 1970);
      bool lowSat  = (minMotor < 1030);


      // ---- Per-axis Dynamic Anti-Windup ----

      // Pitch I-term
      bool pPSat = (p_step > 0.0f);
      bool pNSat = (p_step < 0.0f);
      if (!((highSat && pPSat) || (lowSat && pNSat))) {
        prev.pitch.mem += p_step;
      }

      // Roll I-term
      bool rPSat = (r_step > 0.0f);
      bool rNSat = (r_step < 0.0f);
      if (!((highSat && rPSat) || (lowSat && rNSat))) {
        prev.roll.mem += r_step;
      }

      // Throttle I-term
      bool vPSat = (v_step > 0.0f);
      bool vNSat = (v_step < 0.0f);
      if (!((highSat && vPSat) || (lowSat && vNSat))) {
        prev.throttle.mem += v_step;
      }


      // ---- Write to ESCs -----------
      escFR.writeMicroseconds(clampInt(mFR, ESC_MIN, ESC_MAX));
      escBR.writeMicroseconds(clampInt(mBR, ESC_MIN, ESC_MAX));
      escBL.writeMicroseconds(clampInt(mBL, ESC_MIN, ESC_MAX));
      escFL.writeMicroseconds(clampInt(mFL, ESC_MIN, ESC_MAX));


    } else {

      // dt out of range: soft-update with fixed coefficient
      last.gx += 0.43f * (gx - last.gx);
      last.gy += 0.43f * (gy - last.gy);
      last.gz += 0.43f * (gz - last.gz);

    }
  }
}
