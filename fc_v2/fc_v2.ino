// Flight Controller v2 by Rahul Kumar

// Libraries
#include <Wire.h>
#include <RF24.h>
#include <Servo.h>
#include <math.h>
#include <stdint.h>
#include <Adafruit_BMP280.h>

// Addresses for I2C
#define MPU_ADDR 0x68
#define BMP_ADDR 0x76 // try 0x77 for older bmp 280

// Arudino Pins
#define led_g 3
#define led_r 4
#define ce 7
#define csn 8

// Motor Pins
#define fr 5   // front right
#define fl 6   // front left
#define br 9   // back right
#define bl 10  // back left

// Loops Time
#define LOOP_0 2000 // ESC and gyro // 1_000_000 / 500 = 2_000US
#define LOOP_1 10000 // Acc angle calc and RC input receive // 1_000_000 / 100 = 10_000
#define LOOP_Baro 40000 // 1_000_000 / 25 = 40_000

// Adafruit Barometer Object
Adafruit_BMP280 bmp;

// Loop Last Timer (micros)
unsigned long last_0 = 0, last_1 = 0, last_baro = 0; // last time for loop_0, loop_1 and loop_baro respectively at 500, 100, 25Hz respectively

// Servo Object
Servo escFL, escFR, escBL, escBR;

// Struct Declaration
struct KalmanA, KalmanB, Data_Package, Target;

// NRF24L01 radio (micros)
unsigned long lastReceiveTimeRadio = 0, currentTimeRadio = 0;
int16_t ringBuzzerFor = 0;  // How long to ring buzzer

// Raw IMU (MPU  6050)  sensor storage (int16_t)
int16_t raw_ax = 0, raw_ay = 0, raw_az = 0; // accel
int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0; // gyro

// sensor values converted floats
float ax = 0.0f, ay = 0.0f, az = 0.0f; // accel in g
float gx = 0.0f, gy = 0.0f, gz = 0.0f; // gyro in deg/s

// Gyro  and Acc Offsets (raw)
int16_t gx_offset = -151, gy_offset = -3, gz_offset = -39;
int16_t ax_offset = 137, ay_offset = -181, az_offset = -1346; // update later if needed

// Downward Inertial acceleration and it's Baseline Acceleration
float accZInertial = 0.0f; 
float baselineAccZInertial = 0.0f; // The Inertial Acceleration value when vertical velocity = 0m/s, it's not always 1g in our case

// Final Angle and Vertical Velocity
float rollAngle = 0.0f, pitchAngle = 0.0f;
float verticalSpeed = 0.0f;

// Barometer
float baselinePressure = 0.0f; //HPa
float lastAltitude = 0.0f;
float alpha = 0.25; // low pass filter

// Bring Down bool
bool bringDown = false;

// Flight Controller
float throttle = 0.0f, pitch = 0.0f, roll = 0.0f, yaw = 0.0f; // esc microsecond values


struct Target{
  float pitchAngle;   // pitch angle 
  float pitchRate;    // pitch rate
  float pitchAcc;     // pitch accel

  float rollAngle;    // roll angle
  float rollRate;     // roll rate
  float rollAcc;      // roll accel

  float yawRate;      // yaw rate
  float yawAcc;       // yaw accel

  float vVelocity;    // vertical velocity
  float Acc;          // vertical acceleration
};
Target target;


struct Data_Package {
  byte j1PotX;        // left joystick x axis
  byte j1PotY;        // left joystick y axis
  byte j2PotX;        // right joystick x axis
  byte j2PotY;        // right joystick y axis
  byte pot1;          // left potentiometer
  byte pot2;          // right potentiometer
  byte tSwitch1;      // main switch
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
  float Q_angle;    // noise variance for the angle
  float Q_bias;     // noise variance for the gyro bias
  float R_measure;  // measurement noise variance (from accelerometer)
  float angle;      // [deg] The angle estimated by the Kalman filter
  float bias;       // [deg/s] The gyro bias estimated by the Kalman filter
  float P[2][2];    // Error covariance matrix
};
KalmanA kalRoll, kalPitch;


// NRF24L01 Receiver Object and address
RF24 radio(ce,csn);
const byte address[5] = {'R', 'A', 'D', '-', '0'};

// Sign inline function
inline int sign(float x) {
  return (x>0.0f) - (x<0.0f);
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
  radio.startListening(); //  Set the module as receiver
  radio.writeAckPayload(0, &ringBuzzerFor, sizeof(ringBuzzerFor));
  resetData();
}

// reads the radio signal and saves it in data;
// sends ack packet as buzzer buzz time, buzz for ringBuzzerFor ms;
// if connection lost for more than 1 second -> reset data;
// if connection lost for more than 15 seconds -> land down the drone;
void read_receiver() {

  // ack
  radio.writeAckPayload(0, &ringBuzzerFor, sizeof(ringBuzzerFor));
  ringBuzzerFor = 0;   // clear after loading

  // store data
  if (radio.available()) {

    radio.read(&data, sizeof(Data_Package));
    lastReceiveTimeRadio = millis();
  }

  currentTimeRadio = millis();

  // failsafe and bring down
  if (currentTimeRadio - lastReceiveTimeRadio > 1000) {
    resetData();

    if (currentTimeRadio - lastReceiveTimeRadio > 15000) {
      bringDown = true;
    }
  }
}


// begin BMP
void beginBMP(){
  if(!bmp.begin(BMP_ADDR)){
    // Serial.print("BMP280 not found!");
    return;
  }

  // Lower noise configuration
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X1,   // Temp oversampling
    Adafruit_BMP280::SAMPLING_X4,   // Pressure oversampling
    Adafruit_BMP280::FILTER_X8,     // Strong internal filter
    Adafruit_BMP280::STANDBY_MS_1   // ~1000 Hz loop
  );
  
}


// MPU utility
void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Set up registors and make MPU ready to read data
void beginMPU(){
  Wire.begin();
  Wire.setClock(400000); // faster I2C
  // Serial.begin(115200);
  delay(100);

  // Wake up
  writeMPU(0x6B, 0x00); // PWR_MGMT_1: clear sleep
  delay(10);

  // DLPF = 4 => ~20 Hz 
  writeMPU(0x1A, 0x04);

  // SMPLRT_DIV = 0 -> sample rate = gyro output rate / (1 + SMPLRT_DIV)
  writeMPU(0x19, 0x00);

  // Gyro config ±2000 dps (FS_SEL = 3 -> bits 4:3 = 11 -> 0x18)
  writeMPU(0x1B, 0x18);

  // Accel config ±4g (AFS_SEL = 1 -> bits 4:3 = 01 -> 0x08)
  writeMPU(0x1C, 0x08);

  delay(100);

}

// Read the raw data from MPU;
bool readMPUraw() {
  // Start reading at ACCEL_XOUT_H (0x3B) and request 14 bytes
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0){
    return false;
  } // non-zero = error

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
  if (available < 14){
    return false;
  }

  // Read as unsigned and combine to signed int16_t safely
  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_ax = (int16_t)((hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); raw_ay = (int16_t)((hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); raw_az = (int16_t)((hi << 8) | lo);
  // temp high/low (skip)
  (void)Wire.read(); (void)Wire.read();
  hi = Wire.read(); lo = Wire.read(); raw_gx = (int16_t)((hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); raw_gy = (int16_t)((hi << 8) | lo);
  hi = Wire.read(); lo = Wire.read(); raw_gz = (int16_t)((hi << 8) | lo);

  return true;
}


// calculates the gyro offsets and set baseline pressure from baro
void calibrateGyro() {
  const int calSamples = 2000;
  long sumX = 0, sumY = 0, sumZ = 0;
  int validSamples = 0;
  delay(500);   // Let sensors settle

  // Collect samples
  while (validSamples < calSamples) {
    if (readMPUraw()) {          // Only use valid readings
      sumX += raw_gx;
      sumY += raw_gy;
      sumZ += raw_gz;
      validSamples++;
    }
    delay(1);
  }

  // Average offsets
  gx_offset = (float)sumX / validSamples;
  gy_offset = (float)sumY / validSamples;
  gz_offset = (float)sumZ / validSamples;

  // reference for relative altitude
  baselinePressure = bmp.readPressure() / 100;  // HPa
}


// Initialte Kalman filter using our Q_anlge, Q_bias, R_measure
void kalmanInitA(KalmanA &k) {
  k.Q_angle = 0.002f;
  k.Q_bias = 0.001f;
  k.R_measure = 0.03f;
  k.angle = 0.0f;
  k.bias = 0.0f;
  k.P[0][0] = 0.1f; k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f; k.P[1][1] = 0.01f;
}

// Initialte Kalman filter using our Q_velocity, Q_bias, R_baro
void kalmanInitB(KalmanB &k) {
  k.Q_velocity = 0.05f;  // process noise velocity
  k.Q_bias = 0.0001f;    // process noise bias
  k.R_baro = 0.7f;       // baro measurement noise
  k.velocity = 0.0f;
  k.bias = 0.0f;
  k.P[0][0] = 1.0f; k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f; k.P[1][1] = 1.0f;  // initial bias uncertainty
}

// returns kalman updated angle (deg)
float kalmanUpdateA(KalmanA &k, float newRate, float newAngle, float dt) {
  // Predict
  float rate = newRate - k.bias;
  k.angle += dt * rate;

  // Update estimation error covariance - Project the error covariance ahead
  k.P[0][0] += dt * (dt*k.P[1][1] - k.P[0][1] - k.P[1][0] + k.Q_angle);
  k.P[0][1] -= dt * k.P[1][1];
  k.P[1][0] -= dt * k.P[1][1];
  k.P[1][1] += k.Q_bias * dt;

  // Compute Kalman gain
  float S = k.P[0][0] + k.R_measure; // Estimate error
  float K0 = k.P[0][0] / S;
  float K1 = k.P[1][0] / S;

  // Update the angle and the bias - Update estimate with measurement zk (newAngle)
  float y = newAngle - k.angle; // Innovation
  k.angle += K0 * y;
  k.bias  += K1 * y;

  // Update the error covariance
  float P00_temp = k.P[0][0];
  float P01_temp = k.P[0][1];

  k.P[0][0] -= K0 * P00_temp;
  k.P[0][1] -= K0 * P01_temp;
  k.P[1][0] -= K1 * P00_temp;
  k.P[1][1] -= K1 * P01_temp;

  return k.angle;
}

// returns kalman updated vertical velocity
float kalmanUpdateB(KalmanB &k, float acc_meas, float v_baro, float dt) {
    // ----- PREDICT -----
    float v_pred = k.velocity + (acc_meas - k.bias) * dt;
    float b_pred = k.bias;

    // Covariance prediction
    float P00_p = k.P[0][0] + dt*(k.P[1][0] + k.P[0][1]) + dt*dt*k.P[1][1] + k.Q_velocity;
    float P01_p = k.P[0][1] + dt*k.P[1][1];
    float P10_p = k.P[1][0] + dt*k.P[1][1];
    float P11_p = k.P[1][1] + k.Q_bias;

    // ----- MEASUREMENT UPDATE -----
    float y = v_baro - v_pred;          // residual
    float S = P00_p + k.R_baro;         // innovation covariance
    if (S < 1e-9f) S = 1e-9f;           // safety

    float K0 = P00_p / S;
    float K1 = P10_p / S;

    // State update
    k.velocity = v_pred + K0 * y;
    k.bias     = b_pred + K1 * y;

    // Covariance update (Joseph form)
    float P00_new = (1 - K0)*P00_p*(1 - K0) + K0*k.R_baro*K0;
    float P01_new = (1 - K0)*P01_p*(1 - K1) + K0*k.R_baro*K1; // optional correction
    float P10_new = P10_p*(1 - K0) - K1*k.R_baro*K0;           // optional correction
    float P11_new = (1 - K1)*P11_p*(1 - K1) + K1*k.R_baro*K1;  // optional correction

    k.P[0][0] = P00_new;
    k.P[0][1] = P01_new;
    k.P[1][0] = P10_new;
    k.P[1][1] = P11_new;

    return k.velocity;
}

// Initiate Kalman Filters for both axes and vertical velocity
void initalKalmanFilter(){
  // --- Init Kalman filters ---
  kalmanInitA(kalRoll);
  kalmanInitA(kalPitch);
  kalmanInitB(kalVel);
}


// Update Target angles, angular velocities, and angular accerlaration
void getTarget(){
  Pot2 = ((float)data.pot2 / 255.0f ) * 5.0f + 1.0f;  // [1,6]
  target.pitchAngle = ((float)data.j2PotX / 254.0f ) * 10.0f - 5.0f;  // [-5,5]
  target.pitchAngle *= Pot2;  // [-30,30] <- [5,30]

  target.rollAngle = ((float)data.j2PotY / 254.0f ) * 10.0f - 5.0f;  // [-5,5]
  target.rollAngle *= Pot2;  // [-30,30] <- [5,30]

  target.pitchRate = (float)Pot2*12.0f + 18.0f;  // [30, 90]
  target.rollRate = (float)Pot2*12.0f + 18.0f;  // [30,90]

  Pot1 = ((float)data.pot1 / 255.0f ); // [0,1]
  target.yawRate = ((float)data.j1PotY / 254.0f ) * 40.0f - 20.0f;  // [-20,20]
  target.yawRate *= (float)(Pot1 * 2.75f + 1.0f);  // [-75,75] <- [20,75]

  target.vVelocity = ((float)data.j1PotX / 254.0f ) * 0.6 - 0.3;  // [-0.3,0.3] m/s
  target.vVelocity *= (float)( Pot1 * 9 + 1 ) ;  // [-3,3] <- [0.3, 3] m/s
  
  target.pitchAcc = (float)Pot2 * 12.0f + 48.0f;  // [60, 120]
  target.rollAcc = (float)Pot2 * 12.0f + 48.0f;  // [60,120]
  target.yawAcc = (float)Pot1 * 60.0f + 40.0f;  // [40, 100]
  target.Acc = (float)Pot1 * 4.0f + 0.4f; // [0.4, 4.4] m/s2

  if(bring_down){
    target.vVelocity = -1 * filteredAltitude / 3.0f ;
    target.Acc = 2.5f;
  }

}


// 3 parts of get tele, gyro separate, accel separate, and baro separate

// radio failure check 
void errChk(){
  if(!radio.isChipConnected() || radio.failureDetected || radio.getDataRate() != RF24_250KBPS){
    
    radio.failureDetected = 0;

    radio.begin();

    radio.setChannel(76);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);

    radio.setAutoAck(true);
    radio.enableAckPayload();
    radio.enableDynamicPayloads();

    radio.openReadingPipe(0, address);
    radio.startListening(); //  Set the module as receiver
    resetData();
  }
}

// [target.angle, kalman_angle, updated_rates, target.rates, target.acc] -> NULL
void provide_w_max(float des, float cur, float at, float need, float acc){
  angleErr = des - cur;
  w_max = sign(angleErr) * fminf(sqrt(2*acc*fabsf(angleErr)) , need);
  rateErr = w_max - at;
}

// provides the milis valuees for esc
void get_milis(){
  
  // for pitch 
  provide_w_max(target.pitchAngle, PitchAngle, pitchRate, target.pitchRate, target.pitchAcc);
  if (fabsf(rateErr) > 4.0f || fabsf(angleErr) > 3.0f){
    pitch += Kp * ( target.pitchAcc / 120 ) * rateErr;
  } else pitch = 0.0f;

  // for roll
  provide_w_max(target.rollAngle, RollAngle, rollRate, target.rollRate, target.rollAcc);
  if (fabsf(rateErr) > 4.0f || fabsf(angleErr) > 3.0f){
    roll += Kr * ( target.rollAcc / 120 ) * rateErr;
  } else roll = 0.0f;

  // for yaw
  rateErr = target.yawRate - yawRate;
  if (fabsf(rateErr) > 5.0f ){
    yaw += Ky * ( target.yawAcc / 100 ) * rateErr;
  }else yaw = 0.0f;

  // for throttle
  rateErr = target.velocity - xv/1000;
  if (fabs(rateErr) > 5){
    throttle += Kt * ( target.Acc / 4.4 ) * rateErr;
  }


}

// clamp valuese in closed range [1035,1965]
int clamp(float v) {
  if (v < 1035.0f) return 1035;
  if (v > 1965.0f) return 1965;
  return (int)roundf(v);
}

// calibrate esc's first, and ready them to arm
void armESC(){
  escFL.attach(fl);
  escFR.attach(fr);
  escBL.attach(bl);
  escBR.attach(br);

  // calibration
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

  // hold for 3 seconds before arming
  delay(3000);

  // arm esc
  escFL.writeMicroseconds(1035);
  escFR.writeMicroseconds(1035);
  escBL.writeMicroseconds(1035);
  escBR.writeMicroseconds(1035);
}
















