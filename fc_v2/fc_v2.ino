// Flight Controller v2 by Rahul Kumar
// Project RAD-1

// Libraries
#include <Wire.h>             // for MPU communication
#include <RF24.h>             // for NRF24L01 communication
#include <Servo.h>            // for esc communication
#include <math.h>             // for trignometry and some other functions ( like sqrt )
#include <stdint.h>           // for standard integers
#include <Adafruit_BMP280.h>  // for BMP 280 communication


// Addresses for I2C
#define MPU_ADDR 0x68 // MPU 6050 Address
#define BMP_ADDR 0x76 // BMP 280 Address //try 0x77 for older bmp 280


// Arudino Pins
#define led_g 3   // green led pin
#define led_r 4   // red led pin
#define ce 7      // CE pin of NRF
#define csn 8     // CSN pin of NRF


// Motor Pins
#define fr 5   // front right
#define fl 6   // front left
#define br 9   // back right
#define bl 10  // back left


// Loops Time
#define LOOP_0 2000     // ESC and gyro timer // 1_000_000 / 500 = 2_000US
#define LOOP_1 10000    // Angle Kalman (Accel Reading) and RC input receive timer // 1_000_000 / 100 = 10_000
#define LOOP_BARO 40000 // ZInertial velocity Kalamn (Baro Reading) // 1_000_000 / 25 = 40_000


// Constant Valuues - tune via PD testing
#define KP_T 1.5
#define KP_P 4
#define KP_R 4
#define KP_Y 2.5

#define KD_T 0.3
#define KD_P 4
#define KD_R 4
#define KD_Y 0.02


// Adafruit Barometer Object
Adafruit_BMP280 bmp;

// Servo Object
Servo escFL, escFR, escBL, escBR;


// Loop timers -> to keep track of loop timing
unsigned long loop_0 = 0, loop_1 = 0, loop_baro = 0;

// Last Loop Timer (micros)
unsigned long last_0 = 0, last_1 = 0, last_baro = 0; // last time for loop_0, loop_1 and loop_baro respectively at 500, 100, 25Hz respectively

// dt time
double dt_accel = 0.0d, dt_gyro = 0.0d;


// Struct Declaration
struct KalmanA;
struct KalmanB;
struct Data_Package;
struct Target;


// NRF24L01 radio (micros)
unsigned long lastReceiveTimeRadio = 0; // last radio received time
int16_t ringBuzzerFor = 0;              // How long to ring buzzer


// Raw IMU (MPU  6050)  sensor storage (int16_t)
int16_t raw_ax = 0, raw_ay = 0, raw_az = 0; // accel
int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0; // gyro

// Lat values of IMU (MPU 6050)
float last_ax = 0.0f, last_ay = 0.0f, last_az = 0.0f;
float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;

// Gyro  and Acc Offsets (raw)
int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;
int16_t ax_offset = 137, ay_offset = -181, az_offset = -1346; // update later if needed

// Downward Inertial acceleration base
float baselineAccZInertial = 0.0f;   // The Inertial Acceleration value when vertical velocity = 0m/s, it's not always 1g in our case


// Angle and Vertical Velocity
float rollAngle = 0.0f, pitchAngle = 0.0f;
float verticalSpeed = 0.0f;


// Barometer
float baselinePressure = 0.0f; //HPa
float lastAltitude = 0.0f;

// Bring Down bool
bool bringDown = false;



struct Target{
  float pitchAngle;   // pitch angle 
  float pitchRate;    // pitch rate
  float pitchAcc;     // pitch accel

  float rollAngle;    // roll angle
  float rollRate;     // roll rate
  float rollAcc;      // roll accel

  float yawRate;      // yaw rate
  float yawAcc;       // yaw Acc

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


// Sign function
inline int sign(float x) {
  return (x > 0.0f) - (x < 0.0f);
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

    if (currentTimeRadio - lastTimeRadio > 15000) {

      bringDown = true;

    }

  }

}


// begin BMP
bool beginBMP(){
  if(!bmp.begin(BMP_ADDR)){

    // Serial.print("BMP280 not found!");
    return false;

  }

  // Lower noise configuration
  bmp.setSampling(

    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X1,   // Temp oversampling
    Adafruit_BMP280::SAMPLING_X4,   // Pressure oversampling
    Adafruit_BMP280::FILTER_X8,     // Strong internal filter
    Adafruit_BMP280::STANDBY_MS_1   // ~1000 Hz loop

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

// Set up registors and make MPU ready to read data
bool beginMPU(){

  Wire.begin();
  Wire.setClock(400000); // faster I2C
  // Serial.begin(115200);
  delay(200);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0){

    return false;

  } // non-zero = error

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
  if (available < 1){

    return false;

  }

  if(Wire.read() != 0x68){

    return false;

  }


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

  return true;

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

// Read gyro
bool readGyroRaw(){

  // Start reading at GYRO_XOUT_H (0x43) and request 6 bytes
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // GYRO_XOUT_H

  if (Wire.endTransmission(false) != 0){

    return false;

  } // non-zero = error

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
  
  if (available < 6){

    return false;

  }

  // Read as unsigned and combine to signed int16_t safely
  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_gx = (int16_t)((hi << 8) | lo) - gx_offset;
  hi = Wire.read(); lo = Wire.read(); raw_gy = (int16_t)((hi << 8) | lo) - gy_offset;
  hi = Wire.read(); lo = Wire.read(); raw_gz = (int16_t)((hi << 8) | lo) - gz_offset;

  return true;
}

bool readAccelRaw(){

  // Start reading at ACCEL_XOUT_H (0x3B) and request 6 bytes
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H

  if (Wire.endTransmission(false) != 0){

    return false;

  } // non-zero = error

  uint8_t available = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6, (uint8_t)true);
  
  if (available < 6){

    return false;

  }

  // Read as unsigned and combine to signed int16_t safely
  uint8_t hi, lo;
  hi = Wire.read(); lo = Wire.read(); raw_ax = (int16_t)((hi << 8) | lo) - ax_offset;
  hi = Wire.read(); lo = Wire.read(); raw_ay = (int16_t)((hi << 8) | lo) - ay_offset;
  hi = Wire.read(); lo = Wire.read(); raw_az = (int16_t)((hi << 8) | lo) - az_offset;

  return true;
}


// calculates the gyro offsets, set inertial acceleration and baseline pressure from baro
void calibrate() {

  // Gyro Calibration
  const int calSamples = 2000;
  long sumGX = 0, sumGY = 0, sumGZ = 0;  // Store gyrro values
  long sumAX = 0, sumAY = 0, sumAZ = 0;  // Store accel values
  int validSamples = 0;
  delay(500);   // Let sensors settle

  // Collect samples
  while (validSamples < calSamples) {

    if (readMPUraw()) {   
             // Only use valid readings
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

  // Average offsets
  gx_offset = (float)sumGX / validSamples;
  gy_offset = (float)sumGY / validSamples;
  gz_offset = (float)sumGZ / validSamples;

  
  float ax = (float)sumAX / validSamples / 8192;
  float ay = (float)sumAY / validSamples / 8192;
  float az = (float)sumAZ / validSamples / 8192;

  // Calculate Aproximate angles
  float PitchAngle = atan2f(-ax, sqrtf(ay*ay + az*az)) ;
  float RollAngle = atan2f(ay, sqrtf(ay*ay + az*az)) ; 

  // reference accZInertial, as said -> not 1g in our case
  baselineAccZInertial = -sin(PitchAngle)*ax+cos(PitchAngle)*sin(RollAngle)* ay+cos(PitchAngle)*cos(RollAngle)*az; 

  // reference for relative altitude
  baselinePressure = bmp.readPressure() / 100;  // HPa

}


// PT1 filter;
// [float prev, float curr, float dt, float loop_rate] -> [float filterred_value]
float pt1(float prev, float input, float dt, float fc){

    float RC = 0.1591549f / fc ;        // 1/2*PI*fc
    float alpha = dt / (RC + dt);
    return prev + alpha * (input - prev);

}


// Initialte Kalman filter using our Q_anlge, Q_bias, R_measure
void kalmanInitA(KalmanA &k) {

  k.Q_angle = 6.0923483957e-7f;
  k.Q_bias = 3.0461741979e-7f;
  k.R_measure = 9.1385225936e-6f;
  k.angle = 0.0f;
  k.bias = 0.0f;
  k.P[0][0] = 3.0461741979e-5f; k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f; k.P[1][1] = 3.0461741979e-6f;

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


// Update Target angles, angular velocities, and angular accerlaration
void getTarget(){

  // right pot
  float Pot2 = ((float)data.pot2 * 0.0196078431372549f ) + 1.0f;  // [1,6]

  // pitch
  if(data.j2PotX == 127 ) target.pitchAngle = 0.0f;
  else target.pitchAngle = ((float)data.j2PotX * 0.000684442843919f ) - 0.0872664625997f;  // [-5,5] deg <- min
  target.pitchAngle *= Pot2;                                 // [-30,30] deg <- max

  // roll
  if(data.j2PotY == 127) target.rollAngle = 0.0f;
  else target.rollAngle = ((float)data.j2PotY * 0.000684442843919f ) - 0.0872664625997f;  // [-5,5] deg <- min
  target.rollAngle *= Pot2;                                 // [-30,30] deg <- max

  // omega
  target.pitchRate = (float) Pot2 * 0.209439510239f + 0.314159265359f;   // [30, 90] deg/sec
  target.rollRate  = (float) Pot2 * 0.209439510239f + 0.314159265359f;   // [30, 90] deg/sec

  // alpha
  target.pitchAcc = (float)Pot2 * 0.209439510239f + 0.837758040957f;  // [60, 120] deg/sec2
  target.rollAcc  = (float)Pot2 * 0.209439510239f + 0.837758040957f;  // [60,120] deg/sec2

  // left pot
  float Pot1 = ((float)data.pot1 * 0.0352941176470588f ) + 1; // [1,10]

  // yaw omega
  if(data.j1PotY == 127 ) target.yawRate = 0.0f;
  else target.yawRate = ((float)data.j1PotY * 0.00273777137568f ) - 0.349065850399f;  // [-20,20] deg/sec <- min
  target.yawRate *= (float)(Pot1 * 0.0107843137255f + 1.0f);             // [-75,75] deg/sec <- max

  // vertical velocity
  if(data.j1PotX == 127 ) target.vVelocity = 0.0f;
  else target.vVelocity = ((float)data.j1PotX * 0.0023529411764706f ) - 0.3;  // [-0.3,0.3] m/s <- min
  target.vVelocity *= (float) Pot1;                         // [-3,3 ]    m/s <- max

  // accelerations
  target.yawAcc = (float)Pot1 * 0.116355283466f + 0.581776417331f;  // [40, 100]
  target.Acc      = (float)Pot1 * 0.4; // [0.4, 4.0] m/s2

  // Bring Down failsafe
  float last_altitude = lastAltitude;

  if(bringDown){

    if(last_altitude < 1) target.vVelocity = -0.1;
    else if (last_altitude < 5) target.vVelocity = -0.3;
    else target.vVelocity = -0.2f * last_altitude;
    target.Acc = 0.4f;
    
  }

}


/* ------------ Working ------------------

theta_measuure, omega_measure   -> measured by sensors
theta_target, omega_target      -> wanted theta and omega
omega_max, theta_max            -> Max values of omega and alpha allowed

Error (e) = theta_target - theta_measure          -> angle error
omega_cap = sqrt(2 * alpha_max * |e|)             -> Max value of omega so that it stops at target
omega_out = sign(e) * min(omega_max, omega_cap)   -> what omega we want right now

// We want omega to follow s-curve, that is no jerk
// We will increment omega little by little
// omega_last is the omega we had lastly
d_omega = omega_out - omega_last            -> Difference in desired and current omega
d_omega_max = alpha_max * dt                -> What max omega change we want
wrap(d_omega, +-d_omega_max)                -> Wrap d_omega between max desired d_omega
omega_last = omega_last + d_omega           -> Update our omega
cover(omega_last, +- omega_max)             -> Conver it between max values

Simple Math 😇

 --------------------------------------- */


// Control Loop
float control_loop(float e, float alpha_max, float omega_max, float omega_last, float dt){

  float omega_cap = sqrt(2.0f * alpha_max * fabs(e) );
  float omega_out = sign(e) * fminf( omega_cap, omega_max );

  float d_omega = omega_out - omega_last;
  float max_d_omega = alpha_max * dt;

  if(d_omega > max_d_omega) d_omega = max_d_omega;
  else if (d_omega < -max_d_omega) d_omega = -max_d_omega;

  float omega = omega_last + d_omega;
  if(omega > omega_max) omega = omega_max;
  else if (omega < -omega_max) omega = -omega_max;

  return omega;

}


// clamp valuese in closed range [1035,1965]
int clamp(float v) {

  if (v < 1025.0f) return 1025;
  if (v > 1975.0f) return 1975;
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

  // hold for 1 seconds before arming
  delay(1000);

  // Spin at a little above 0 speed for visual check
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


void setup(){

  // Serial.begin(9600);

  // LEDs
  pinMode(led_g, OUTPUT);   // Green
  digitalWrite(led_g, LOW);

  pinMode(led_r, OUTPUT);   // Red
  digitalWrite(led_r, HIGH);

  // starting radio (nrf2401l)
  for(int i = 0; i < 10; i++){

    if(radio.isChipConnected()){

      startListening();
      break;

    }

  }

  // mpu and bmp check
  bool mpu_ok = false;
  bool bmp_ok = false;

  // 10 retires for each imu and bmp sensor if initialization fails
  for(int i = 0; i < 10; i++){

    if(!mpu_ok) mpu_ok = beginMPU();
    if(!bmp_ok) bmp_ok = beginBMP();

    if(mpu_ok && bmp_ok ) break;
    delay(50);

  }

  // if still not initialized, raise error
  if(!mpu_ok || !bmp_ok || !radio.isChipConnected()){

    // Serial.print("Sensor Bad");
    
    while(1){

      delay(500);
      digitalWrite(led_r, LOW);
      delay(500);
      digitalWrite(led_r, HIGH);

    }

  }


  // --- Initiate Kalman filters ---
  kalmanInitA(kalPitch);
  kalmanInitA(kalRoll);
  kalmanInitB(kalVel);


  // Arm check
  while(true){

    read_receiver();
    if(data.tSwitch1) break;  // armed
    delay(500);

  }

  // Calibrate
  calibrate();

  // arm esc and visual check for motors
  armESC();
  digitalWrite(led_g, HIGH);  // ready for takeoff
  digitalWrite(led_r, LOW);

  ringBuzzerFor = 1500;
  read_receiver();
  resetData();    // reset data after listening
  target.vVelocity = 0.0f;
  delay(1300);          // 1300 + buffer = 1500 delay for buzzer ring

  // Set initial variables
  lastAltitude = bmp.readAltitude(baselinePressure);
  last_baro = micros();
  readMPUraw();
  last_0 = last_1 = micros();
  
  // Accel : 4g = 8192 LSB/g
  // 1/ 8192 = 0.0001220703125f
  last_ax = (float) raw_ax * 0.0001220703125f;
  last_ay = (float) raw_ay * 0.0001220703125f;
  last_az = (float) raw_az * 0.0001220703125f;

  // Gyro: 2000dps = 16.4 LSB/(deg/s)
  // 1/16.4 * pi/180 = 0.001064225153689f
  last_gx = (float) raw_gx * 0.001064225153689f;
  last_gy = (float) raw_gy * 0.001064225153689f;
  last_gz = (float) raw_gz * 0.001064225153689f;

  // Initiate loop timers
  loop_0 = loop_1 = loop_baro = micros();

}


void loop(){

  unsigned long now = micros();

  // 100Hz loop - Radio Read -> get Target, MPU angle Read -> angle kalman, vertical speed prediction
  if((now - loop_1) >= 0){
    loop_1 += LOOP_1;

    read_receiver();

    // Kill check
    if(!data.tSwitch1){

      escFL.writeMicroseconds(1000);
      escFR.writeMicroseconds(1000);
      escBL.writeMicroseconds(1000);
      escBR.writeMicroseconds(1000);
      digitalWrite(led_r,HIGH);

      while(true){

        read_receiver();
        if(data.tSwitch1) break;

      }

    }

    getTarget();

    readAccelRaw();

    unsigned long currTime = micros();
    double dt = (currTime - last_1) * 1e-6f;
    last_1 = currTime;

    // Convert raw to physical units
    // Accel : 4g = 8192 LSB/g
    // 1/8192 = 0.0001220703125f
    float ax = (float) raw_ax * 0.0001220703125f;
    float ay = (float) raw_ay * 0.0001220703125f;
    float az = (float) raw_az * 0.0001220703125f;

    if(dt <= 0.03 && dt > 0){

      float filax = pt1(last_ax, ax, dt, 25);
      float filay = pt1(last_ay, ay, dt, 25);
      float filaz = pt1(last_az, az, dt, 25);

      last_ax = filax;
      last_ay = filay;
      last_az = filaz;

      // Compute accel angles (rad)
      float rollAng = atan2f(filay, sqrtf(filay*filay + filaz*filaz));
      float pitchAng = atan2f(-filax, sqrtf(filay*filay + filaz*filaz));

      float accZInertial = -sin ( pitchAng ) * filax + cos( pitchAng ) * sin ( rollAng ) * filay + cos ( pitchAng ) * cos ( rollAng ) * filaz;
      accZInertial = (accZInertial - baselineAccZInertial) * 9.81;

      // Update Kalman filters (deg)
      rollAngle = kalmanUpdateA(kalRoll, last_gx, rollAng, dt_gyro);
      pitchAngle = kalmanUpdateA(kalPitch, last_gy, pitchAng, dt_gyro);

      verticalSpeed += (accZInertial - kalVel.bias) * dt;
      dt_accel = dt;

    }else{

      last_ax += 0.557 * (ax - last_ax);
      last_ay += 0.557 * (ay - last_ay);
      last_az += 0.557 * (az - last_az);

    }

    now = micros();

  }

  // 25Hz loop - Baro Kalman
  if((now-loop_baro) >= 0){

    loop_baro += LOOP_BARO;

    // Read velocity from barometer
    float curr = bmp.readAltitude(baselinePressure);

    unsigned long currentTime = micros();
    double dt = ( currentTime - last_baro ) * 1e-6f;
    last_baro = currentTime;

    if(dt <= 0.1 && dt > 0){

      float last_alt = lastAltitude;
      float curr_fil = pt1(last_alt, curr, dt, 3.0f );
      lastAltitude = curr_fil;

      float vel_raw = (curr_fil - last_alt) / dt;
      float vel_fil = pt1(verticalSpeed, vel_raw, dt, 3.0f);

      float pitchAng = pitchAngle;
      float rollAng = rollAngle;

      float ax = last_ax;
      float ay = last_ay;
      float az = last_az;

      float accZInertial = -sin (pitchAng ) *ax+cos(pitchAng ) * sin(rollAng ) * ay + cos(pitchAng ) * cos(rollAng ) * az;
      accZInertial = (accZInertial - baselineAccZInertial) * 9.81;

      verticalSpeed =  kalmanUpdateB(kalVel, accZInertial, vel_fil, dt_accel);

    }else{

      lastAltitude += 0.43 * (curr - lastAltitude);

    }
    
    now = micros();

  }
  
  // Inner 500Hz loop
  if((now - loop_0) >= 0){

    loop_0 += LOOP_0;

    readGyroRaw();  // Read raw

    unsigned long currTime = micros();
    double dt = (currTime - last_0) * 1e-6f;
    last_0 = currTime;
    
    // Convert raw to physical values
    // Gyro: 2000dps = 16.4 LSB/(deg/s)
    // 1/16.4 * pi/180= 0.001064225153689f
    float gx = (float) raw_gx * 0.001064225153689f;
    float gy = (float) raw_gy * 0.001064225153689f;
    float gz = (float) raw_gz * 0.001064225153689f;

    gx = gx - kalRoll.bias;
    gy = gy - kalPitch.bias;

    if(dt <= 0.006 && dt > 0){
      
      float fil_gx = pt1(last_gx, gx, dt, 100);
      float fil_gy = pt1(last_gy, gy, dt, 100);
      float fil_gz = pt1(last_gz, gz, dt, 100);

      last_gx = fil_gx;
      last_gy = fil_gy;
      last_gz = fil_gz;

      rollAngle += fil_gx * dt;
      pitchAngle += fil_gy * dt;

      dt_gyro = dt;

      // Flight Controller

      // Pitch
      float pitch_omega;

      float pitch_error = target.pitchAngle - pitchAngle;
      if( fabsf(pitch_error) > 1){

        pitch_omega = control_loop(pitch_error, target.pitchAcc, target.pitchRate, fil_gx, dt);
      
      } else pitch_omega = 0.0f;

      float pitch = (pitch_omega - fil_gx) * KP_P - KD_P * fil_gx;
      

      // Roll
      float roll_omega;

      float roll_error = target.rollAngle - rollAngle;
      if( fabsf(roll_error) > 1){

        roll_omega = control_loop(roll_error, target.rollAcc, target.rollRate, fil_gy, dt);
      
      } else roll_omega = 0.0f;

      float roll = (roll_omega - fil_gy) * KP_R - KD_R * fil_gy;


      // Yaw
      float yaw_cmd;

      float target_yaw = target.yawRate;
      float yaw_error = target_yaw - fil_gz;

      if(fabsf(yaw_error) > 5){

        float max_error = target.yawAcc * dt;
        if(yaw_error > max_error) yaw_error = max_error;
        if(yaw_error < -max_error) yaw_error = -max_error;

        yaw_cmd = fil_gz + yaw_error;
        if(yaw_cmd > target_yaw) yaw_cmd = target_yaw;
        else if ( yaw_cmd < -target_yaw) yaw_cmd = -target_yaw;
      
      } else yaw_cmd = 0.0f;
      
      float yaw = KP_Y * (yaw_cmd - fil_gz) - KD_Y * (fil_gz);


      // Vertical velocity
      float vz;

      float target_vz = target.vVelocity;
      float now = verticalSpeed;
      float vz_error = target_vz - now;

      if(fabsf(vz_error) > 0.15) {

        float max_vz_error = target.Acc * dt;
        if(vz_error > max_vz_error) vz_error = max_vz_error;
        if(vz_error < -max_vz_error) vz_error = -max_vz_error;

        vz = now + vz_error;
        if(vz > target_vz) vz = target_vz;
        if(vz < -target_vz) vz = -target_vz;
      
      } else vz = 0.0f;
     
      float throttle = KP_T * (vz - now) - KD_T * (now);

      
      // feeding values to escs'
      escFR.writeMicroseconds(clamp(throttle - roll - pitch - yaw));
      escBR.writeMicroseconds(clamp(throttle - roll + pitch + yaw));
      escBL.writeMicroseconds(clamp(throttle + roll + pitch - yaw));
      escFL.writeMicroseconds(clamp(throttle + roll - pitch + yaw));


    }else{

      last_gx += 0.557 * (gx - last_gx);
      last_gy += 0.557 * (gy - last_gy);
      last_gz += 0.557 * (gz - last_gz);

    }
    
  }

}




// -Over-