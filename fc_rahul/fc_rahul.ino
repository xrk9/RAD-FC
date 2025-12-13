// Flight controller by Rahul Kumar

// libraries
#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <Servo.h>
#include <math.h>
#include <stdint.h>
#include <Adafruit_BMP280.h>

// barometer
Adafruit_BMP280 bmp;

// addresses
#define MPU_ADDR 0x68
#define BMP_ADDR 0x76 // change to 0x77 if fails

// pins
#define led_g 3
#define led_r 4
#define ce 7
#define csn 8

// motor pins
#define fr 5   // front right
#define fl 6   // front left
#define br 9   // back right
#define bl 10  // back left

// Servo
Servo escFL, escFR, escBL, escBR;

// struct
struct Kalman;
struct Data_Package;
struct Target;
struct Input;

// NRF24L01
unsigned long lastReceiveTimeRadio = 0;
unsigned long currentTimeRadio = 0;
int16_t ringBuzzerFor = 0;

// Kalman filter
float Q_angle = 0.002f;  // -------- update if needed -----------
float Q_bias = 0.001f;
float R_measure = 0.03f;

// Raw sensor storage (int16_t) and converted floats
int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;

// sensor values
float ax = 0.0f, ay = 0.0f, az = 0.0f; // in g
float gx = 0.0f, gy = 0.0f, gz = 0.0f; // in deg/s

// gyro and acc offsets (raw)
float gx_offset = -151.0f, gy_offset = -3.0f, gz_offset = -39.0f;
float ax_offset = 137.0f, ay_offset = -181.0f, az_offset = -1346.0f; // update -----------------------------

// Angles
float RollAngle = 0.0f, PitchAngle = 0.0f;
float accelRollAngle = 0.0f, accelPitchAngle = 0.0f;

// Other (Kalman)
unsigned long lastMicros = 0;
unsigned long nowMicros = 0;
float dt = 0.0f;

// vertical volocity by mpu
float AccZInertial = 0.0f;
float VelocityVertical = 0.0f;

// barometer
float baselinePressure = 0;
float altitude = 0;
float filteredAltitude = 0;
unsigned long lastTime = 0;
float verticalSpeed = 0;
float lastAltitude = 0;
bool bring_down = false;

// flight controller
float throttle = 0.0f, pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
float w_max = 0.0f;
float angleErr = 0.0f , rateErr = 0.0f;
float pitchRate = 0.0f, rollRate = 0.0f, yawRate = 0.0f;  // measured
float alpha = 0.3f;
float Pot1 = 0.0f, Pot2= 0.0f;
float Kp = 0.0015f, Kr = 0.0015f, Ky = 0.0007f, Kt = 0.4f;






// Tuning (start here)
float Q_v   = 0.03f;   // process noise for velocity state (m^2/s^4) - small
float Q_b   = 0.0005f; // process noise for bias (m^2/s^2) - how fast bias can change
float R_baro = 0.5f;   // baro velocity measurement variance (m^2/s^2). tune upward if baro noisy
float R_mpu  = 0.8f;   // mpu velocity measurement variance (m^2/s^2). tune

// Kalman state
float xv = 0.0f; // estimated velocity (m/s)
float xb = 0.0f; // estimated MPU bias (m/s)
float P00 = 1.0f, P01 = 0.0f, P10 = 0.0f, P11 = 1.0f; // covariance 2x2

unsigned long lastMicros_new = 0;










struct Target{
  float pitchAngle;
  float rollAngle;

  float pitchRate;
  float rollRate;
  float yawRate;
  float velocity;

  float pitchAcc;
  float rollAcc;
  float yawAcc;
  float Acc;

};
Target target;

struct Input{
  int throttle;
  int pitch;
  int roll;
  int yaw;
};
Input input;
// --- xxx ---

// --- Receiver ---
RF24 radio(ce, csn);
const byte address[5] = {'R', 'A', 'D', '-', '0'};

struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte j2PotY;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte j1B;
};
Data_Package data;

// provides the sign
inline int sign(float x) {
  return (x>0.0f) - (x<0.0f);
}

// reset the radio data
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
  resetData();
}

// reads the radio signal and saves it in data;
// sends ack packet as buzzer buzz time, buzz for ringBuzzerFor ms;
// if connection lost for more than 1 second -> reset data;
// if connection lost for more than 15 seconds -> land down the drone;
// red light if nrf fails
void read_receiver(){
  if (radio.available()) {

    radio.read(&data, sizeof(Data_Package));

    if(ringBuzzerFor>0){
      radio.writeAckPayload(0,&ringBuzzerFor, sizeof(ringBuzzerFor));
      ringBuzzerFor = 0;
    }

    lastReceiveTimeRadio = millis();

  }
  currentTimeRadio = millis();
  // if no signal for 1 second, reset Data, that is connection lost.
  if ( currentTimeRadio - lastReceiveTimeRadio > 1000 ) {
    resetData();
    if (  currentTimeRadio - lastReceiveTimeRadio > 15000 ){
      bring_down = true;
    }
  }
}
// --- xxx ---

// --- Get Angle ---
struct Kalman {
  float Q_angle;    // process noise variance for the angle
  float Q_bias;     // process noise variance for the gyro bias
  float R_measure;  // measurement noise variance (from accelerometer)
  float angle;      // [deg] The angle estimated by the Kalman filter
  float bias;       // [deg/s] The gyro bias estimated by the Kalman filter
  float P[2][2];    // Error covariance matrix
};
Kalman kalRoll, kalPitch;

// Initialte Kalman filter using our Q_anlge, Q_bias, R_measure
void kalmanInit(Kalman &k, float Q_angle, float Q_bias, float R_measure) {
  k.Q_angle = Q_angle;
  k.Q_bias = Q_bias;
  k.R_measure = R_measure;
  k.angle = 0.0f;
  k.bias = 0.0f;
  k.P[0][0] = 0.0f; k.P[0][1] = 0.0f;
  k.P[1][0] = 0.0f; k.P[1][1] = 0.0f;
}

// begin BMP;
// sets the base line pressure;
void beginBMP(){
  if(!bmp.begin(BMP_ADDR)){
    // Serial.print("BMP280 not found!");
    return;
  }

  // Lower noise configuration
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X16,   // Temp oversampling
    Adafruit_BMP280::SAMPLING_X16,   // Pressure oversampling
    Adafruit_BMP280::FILTER_X16,     // Strong internal filter
    Adafruit_BMP280::STANDBY_MS_63   // ~1000 Hz loop
  );
  
  delay(3000);
  ringBuzzerFor = 100;
  read_receiver();

  baselinePressure = bmp.readPressure() / 100;  // reference for relative altitude
  lastTime = millis();

}

// returns kalman updated angle (deg)
float kalmanUpdate(Kalman &k, float newRate, float newAngle, float dt) {
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

// MPU utility
void writeMPU(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
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

// Set up registors and make MPU ready to read data
void beginMPU(){
  Wire.begin();
  Wire.setClock(400000); // optional: faster I2C (ok for short wires)
  // Serial.begin(115200);
  delay(100);

  // Wake up
  writeMPU(0x6B, 0x00); // PWR_MGMT_1: clear sleep
  delay(10);

  // DLPF = 4 => ~20 Hz (good for videography)
  writeMPU(0x1A, 0x04);

  // SMPLRT_DIV = 0 -> sample rate = gyro output rate / (1 + SMPLRT_DIV)
  writeMPU(0x19, 0x00);

  // Gyro config ±2000 dps (FS_SEL = 3 -> bits 4:3 = 11 -> 0x18)
  writeMPU(0x1B, 0x18);

  // Accel config ±4g (AFS_SEL = 1 -> bits 4:3 = 01 -> 0x08)
  writeMPU(0x1C, 0x08);

  delay(100);

}

// calculates the gyro offsets
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
}

// calc the raw acceleration and gyroscope values;
// Updates Pitch and Roll angles;
// update rates of pitch and roll;
void getAngle(){
  nowMicros = micros();
  dt = (nowMicros - lastMicros) / 1000000.0f;
  if (dt <= 0.000001f) dt = 0.000001f; // safety lower bound
  lastMicros = nowMicros;

  // Read sensors, return early if I2C read failed
  if (!readMPUraw()) {
    // I2C read failed; skip this cycle
    return;
  }

  // Convert raw to physical units
  // Accel: ±4g => 8192 LSB/g
  ax = (float)(raw_ax - ax_offset)/8192;
  ay = (float)(raw_ay - ay_offset)/8192;
  az = (float)(raw_az - az_offset)/8192;

  // Gyro: ±2000 dps => 16.4 LSB/(deg/s)
  gx = (float)(raw_gx - gx_offset) / 16.4;
  gy = (float)(raw_gy - gy_offset) / 16.4;
  gz = (float)(raw_gz - gz_offset) / 16.4;

  // Compute accel angles (deg)
  // Roll: rotation around X axis, Pitch: rotation around Y axis
  // Note: choice of signs depends on sensor orientation on your frame
  accelRollAngle  = atan2f(ay, sqrtf(ay*ay + az*az)) * 180.0f / PI; // -ax used to maintain coordinate convention
  accelPitchAngle = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;

  // Update Kalman filters using gyro rates (deg/s) and accel angles (deg)
  RollAngle  = kalmanUpdate(kalRoll, gx, accelRollAngle, dt);
  PitchAngle = kalmanUpdate(kalPitch, gy, accelPitchAngle, dt);

  rollRate = alpha*pitchRate + (1.0f-alpha)*gx;
  pitchRate = alpha*rollRate + (1.0f-alpha)*gy;
  yawRate = alpha*yawRate + (1.0f-alpha)*gz;

  AccZInertial=-sin(PitchAngle*(PI/180))*ax+cos(PitchAngle*(PI/180))*sin(RollAngle*(PI/180))* ay+cos(PitchAngle*(PI/180))*cos(RollAngle*(PI/180))*az;   
  AccZInertial=(AccZInertial-1)*9.81*100;
  VelocityVertical = VelocityVertical + dt * AccZInertial;

  Serial.print("  || VVelocity: ");
  Serial.println(xv);

}

// Initiate Kalman Filter for both axes
void iniitalKalmanFilter(){
  // --- Init Kalman filters ---
  // These Q/R values are good starting points for smooth videography.
  kalmanInit(kalRoll, Q_angle, Q_bias, R_measure);
  kalmanInit(kalPitch, Q_angle, Q_bias, R_measure);
}









// Kalman update: acc_meas (m/s^2), v_baro (m/s), v_mpu (m/s), dt (s)
// new
void kalman_update(float acc_meas, float v_baro, float v_mpu, float dt) {
  // ----- PREDICT -----
  // State transition matrix F = [1  -dt; 0 1]  (see derivation in explanation)
  // Control (acceleration) enters as + a*dt to v
  // Predicted state:
  float v_pred = xv + (acc_meas - xb) * dt;  // note: -xb because bias subtracts from acc->vel
  float b_pred = xb; // random walk

  // Jacobian F applied to P: P = F*P*F^T + Q
  // Here F = [[1, -dt],[0,1]]
  // Compute P_pred manually:
  float P00_p = P00 + dt * (P10 - P01) + dt*dt * P11 + Q_v;
  float P01_p = P01 - dt * P11;
  float P10_p = P10 - dt * P11;
  float P11_p = P11 + Q_b;

  // ----- MEASUREMENT UPDATE (two measurements stacked) -----
  // H = [[1,0],[1,1]]
  // z = [v_baro, v_mpu]^T
  // S = H*P_pred*H^T + R (2x2)
  // Compute S:
  // H*P*H^T = [ P00_p, P00_p + P01_p; P00_p + P10_p, P00_p + P01_p + P10_p + P11_p ]
  float S00 = P00_p + R_baro;
  float S01 = P00_p + P01_p;            // + 0 for R cross (R is diagonal)
  float S10 = P00_p + P10_p;
  float S11 = P00_p + P01_p + P10_p + P11_p + R_mpu;

  // Invert S (2x2)
  float iS00, iS01, iS10, iS11;
  if (!invert2x2(S00, S01, S10, S11, iS00, iS01, iS10, iS11)) {
    // fallback: skip update if singular
    xv = v_pred; xb = b_pred;
    P00 = P00_p; P01 = P01_p; P10 = P10_p; P11 = P11_p;
    return;
  }

  // Compute K = P_pred * H^T * inv(S)
  // P_pred * H^T = [ [P00_p, P00_p + P01_p], [P10_p, P10_p + P11_p] ]
  // multiply that 2x2 by inv(S) to get K (2x2)
  float A00 = P00_p, A01 = P00_p + P01_p;
  float A10 = P10_p, A11 = P10_p + P11_p;

  // K = A * iS
  float K00 = A00 * iS00 + A01 * iS10;
  float K01 = A00 * iS01 + A01 * iS11;
  float K10 = A10 * iS00 + A11 * iS10;
  float K11 = A10 * iS01 + A11 * iS11;

  // Measurement residual y = z - H*x_pred
  float z0 = v_baro;
  float z1 = v_mpu;
  // H*x_pred = [v_pred; v_pred + b_pred]
  float y0 = z0 - v_pred;
  float y1 = z1 - (v_pred + b_pred);

  // Update state: x = x_pred + K * y
  xv = v_pred + K00 * y0 + K01 * y1;
  xb = b_pred + K10 * y0 + K11 * y1;

  // Update P: P = (I - K*H) * P_pred
  // Compute K*H:
  // K*H = [ [K00+K01, K01], [K10+K11, K11] ]? let's compute directly above:
  float KH00 = K00 + K01; // row0 * col0 of H (1,1)?? check: H = [[1,0],[1,1]] so
  // Actually compute (K*H) elementwise:
  // (K*H)[0,0] = K00*1 + K01*1 = K00 + K01
  // (K*H)[0,1] = K00*0 + K01*1 = K01
  // (K*H)[1,0] = K10 + K11
  // (K*H)[1,1] = K11
  float KH01 = K01;
  float KH10 = K10 + K11;
  float KH11 = K11;

  // I - K*H:
  float M00 = 1.0f - KH00;
  float M01 = 0.0f - KH01;
  float M10 = 0.0f - KH10;
  float M11 = 1.0f - KH11;

  // P_new = (I-KH) * P_pred
  float newP00 = M00 * P00_p + M01 * P10_p;
  float newP01 = M00 * P01_p + M01 * P11_p;
  float newP10 = M10 * P00_p + M11 * P10_p;
  float newP11 = M10 * P01_p + M11 * P11_p;

  P00 = newP00; P01 = newP01; P10 = newP10; P11 = newP11;
}
// --- helper: invert 2x2 matrix
bool invert2x2(float a, float b, float c, float d, float &ia, float &ib, float &ic, float &id) {
  float det = a*d - b*c;
  if (fabs(det) < 1e-12f) return false;
  float inv = 1.0f / det;
  ia =  d * inv;
  ib = -b * inv;
  ic = -c * inv;
  id =  a * inv;
  return true;
}

float lastBaroAlt = 0.0f;
float lastBaroTime = 0.0f;
float v_baro = 0.0f;

float v_mpu = 0.0f;      // integrated MPU velocity (m/s) - keep it simple
float mpu_integ_decay = 0.999f; // slight decay to reduce long-term drift








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
  escFL.writeMicroseconds(1000);
  escFR.writeMicroseconds(1000);
  escBL.writeMicroseconds(1000);
  escBR.writeMicroseconds(1000);
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

  target.velocity = ((float)data.j1PotX / 254.0f ) * 0.6 - 0.3;  // [-0.3,0.3] m/s
  target.velocity *= (float)( Pot1 * 9 + 1 ) ;  // [-3,3] <- [0.3, 3] m/s
  
  target.pitchAcc = (float)Pot2 * 12.0f + 48.0f;  // [60, 120]
  target.rollAcc = (float)Pot2 * 12.0f + 48.0f;  // [60,120]
  target.yawAcc = (float)Pot1 * 60.0f + 40.0f;  // [40, 100]
  target.Acc = (float)Pot1 * 4.0f + 0.4f; // [0.4, 4.4] m/s2

  if(bring_down){
    target.velocity = -1 * filteredAltitude / 3.0f ;
    target.Acc = 2.5f;
  }

}

// [target.angle, kalman_angle, updated_rates, target.rates, target.acc] -> NULL
void provide_w_max(float des, float cur, float at, float need, float acc){
  angleErr = des - cur;
  w_max = sign(angleErr) * fminf(sqrt(2*acc*fabsf(angleErr)) , need);
  rateErr = w_max - at;
}

// read BMP280;
// gives vertical speed and filtered altitide
void readBMP(){
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt < 0.09) return; // prevent running faster than 10 Hz
  lastTime = currentTime;

  // Read current altitude (relative)
  altitude = bmp.readAltitude(baselinePressure) * 100;

  // Low-pass filter
  // filteredAltitude = filteredAltitude * 0.4 + altitude * 0.6;
  filteredAltitude += 0.1 * (altitude - filteredAltitude);

  // Vertical speed (m/s)
  verticalSpeed = (filteredAltitude - lastAltitude) / dt;

  lastAltitude = filteredAltitude;

}

// provides the milis valuees
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

// clamp valuese in closed range [1000,2000]
int clamp(float v) {
  if (v < 1000.0f) return 1000;
  if (v > 2000.0f) return 2000;
  return (int)roundf(v);
}

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

void setup() { // ------------------------------------------

  Serial.begin(9600);
  lastMicros_new = micros();

  pinMode(led_g, OUTPUT);
  digitalWrite(led_g, LOW);
  pinMode(led_r, OUTPUT);
  digitalWrite(led_r, HIGH);


  startListening();
  beginMPU();
  iniitalKalmanFilter();
  
  /*
  do{
    read_receiver();
    delay(500);
    Serial.print("kugiug");
  } while ( !data.tSwitch1);
  */

  beginBMP();
  Serial.println("Begin Calibration");
  calibrateGyro();


  if(!bmp.begin(BMP_ADDR) || !readMPUraw() ){
    Serial.print("liuyugyiug");

    digitalWrite(led_g,HIGH);
    digitalWrite(led_r,HIGH);
    delay(1000);
    while(1){
      delay(2000);
    }
  }

  armESC();
  digitalWrite(led_g, HIGH);

  ringBuzzerFor = 1000;
  read_receiver();

  digitalWrite(led_r, LOW);

  // initialize each pitch roll and yaw
  pitch = roll = yaw = throttle = 0.0f;

}

void loop() {

  errChk();
  read_receiver();
  getTarget();
  readBMP();





  unsigned long now = micros();
  float dt = (now - lastMicros_new) * 1e-6f;
  if (dt <= 0) { dt = 0.004f; }
  lastMicros_new = now;

  float acc_z_world = AccZInertial;
  float bmpAlt = filteredAltitude;

  // --- compute v_baro by differentiating altitude (low-pass)
  if (lastBaroTime == 0.0f) {
    lastBaroAlt = bmpAlt;
    lastBaroTime = now * 1e-6f;
    v_baro = 0.0f;
  } else {
    float baro_dt = (now * 1e-6f) - lastBaroTime;
    if (baro_dt <= 0) baro_dt = dt;
    float raw_v_baro = (bmpAlt - lastBaroAlt) / baro_dt;
    // low-pass to reduce noise (alpha ~ 0.4 -> stronger smoothing)
    const float alpha = 0.4f;
    v_baro = alpha * raw_v_baro + (1.0f - alpha) * v_baro;
    lastBaroAlt = bmpAlt;
    lastBaroTime = now * 1e-6f;
  }

  // --- compute v_mpu by integrating acc_z_world
  // acc_z_world should already be in m/s^2 with gravity removed
  v_mpu += acc_z_world * dt;
  // small decay to limit low-frequency drift (tunable or remove if you use bias state)
  v_mpu *= mpu_integ_decay;

  // --- call kalman update
  kalman_update(acc_z_world, v_baro, v_mpu, dt);









  getAngle();
  get_milis();
  
  /*
  if(!data.tSwitch1){
    escFL.writeMicroseconds(1000);
    escFR.writeMicroseconds(1000);
    escBL.writeMicroseconds(1000);
    escBR.writeMicroseconds(1000);
    digitalWrite(led_r, HIGH);
    ringBuzzerFor = 300;
    while(!data.tSwitch1){
      read_receiver();

    }
    digitalWrite(led_r,LOW);
    return;
  }
  */
  
  Serial.print("  Throttle: ");
  Serial.print(throttle);
  Serial.print("  Roll: ");
  Serial.print(roll);
  Serial.print("  Pitch: ");
  Serial.print(pitch);
  Serial.print("  Yaw: ");
  Serial.print(yaw);
}
// --- xxx ---