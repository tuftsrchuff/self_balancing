//Include necessary files
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <JY901.h>

// ================================================================
// ===                      MPU6050 Variables                   ===
// ================================================================
//All variables for MPU reading
#define OUTPUT_READABLE_EULER
#define INTERRUPT_PIN 15
bool blinkState = false;
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ================================================================
// ===                      WT901 Variables                     ===
// ================================================================
//for uart comm
byte open_com[] = {0xFF, 0xF0, 0xF0, 0xF0, 0xF0};// Special unlock/enable thing (not documented anywhere!)
byte unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
byte save[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
byte change_200Hz[] = {0xFF, 0xAA, 0x03, 0x0b, 0x00}; // 200 hz 
byte calib_gy_and_acc[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
byte calib_mag[] = {0xFF, 0xAA, 0x01, 0x02, 0x00};
byte calib_z_0[] = {0xFF, 0xAA, 0x01, 0x03, 0x00};
byte exit_calib[] = {0xFF, 0xAA, 0x01, 0x00, 0x00};
byte set_vert[] = {0xFF, 0xAA, 0x23, 0x01, 0x00};
byte set_horiz[] = {0xFF, 0xAA, 0x23, 0x00, 0x00};
byte set_9ax[] = {0xFF, 0xAA, 0x24, 0x00, 0x00};
byte set_ang_only_out[] = {0xFF, 0xAA, 0x02, 0x08, 0x00};
byte set_baud[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};

double angle_z = 0.0; double angle_x = 0.0; double angle_y = 0.0; 

// ================================================================
// ===                      DC Motor Variables                  ===
// ================================================================
int motorPin1 = 27; 
int motorPin2 = 26; 
int enablePin = 14; 

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 250;

// ================================================================
// ===                      PID Variables                       ===
// ================================================================
float balance = 90;
float errI = 0;
float kp = 25;
float kd = 0;
float ki = 0;
float saturation = 250;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  Wire.begin();
  // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial1.begin(9600, SERIAL_8N1, 17, 16); delay(10); 

  setup_wt901(); 
  setupMPU6050();
  setupDC();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  float eulerMPU = getEulerMPU(-10.52, -0.98, 0.23);
  float eulerWT901 = getEulerWT901(4.5648, 2.2467, -159.3347);
  float avgAngle = (eulerMPU + eulerWT901) / 2;

  Serial.print("Euler MPU: "); Serial.println(eulerMPU);
  Serial.print("Euler WT901: "); Serial.println(eulerWT901);
  Serial.print("Average angle: "); Serial.println(avgAngle);

  pidControl(avgAngle);
}

// ================================================================
// ===                      MPU6050 Functions                   ===
// ================================================================

void dmpDataReady() {
    mpuInterrupt = true;
}

void setupMPU6050(){
  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

            // wait for ready
        Serial.println(F("\nSend any character to begin DMP programming and demo: "));
        while (Serial.available() && Serial.read()); // empty buffer
        while (!Serial.available());                 // wait for data
        while (Serial.available() && Serial.read()); // empty buffer again

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

float getEulerMPU(int offsetX, int offsetY, int offsetZ){
  // if programming failed, don't try to do anything
  // if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      // Serial.print("MPU Euler Angle\t");
      // Serial.println((euler[0] * 180/M_PI) - offsetX);
      // Serial.print("\t");
      // Serial.print((euler[1] * 180/M_PI) - offsetY);
      // Serial.print("\t");
      // Serial.println((euler[2] * 180/M_PI) - offsetZ);
      return (euler[2] * 180/M_PI) - offsetZ;
  }
}

// ================================================================
// ===                      WT901 Functions                     ===
// ================================================================

float getEulerWT901(int offsetX, int offsetY, int offsetZ){
  read_wt901();

  angle_x = (((volatile double)JY901.stcAngle.Angle[0] / 32768 * 180));
  angle_y = (((volatile double)JY901.stcAngle.Angle[1] / 32768 * 180));
  angle_z = (((volatile double)JY901.stcAngle.Angle[2] / 32768 * 180));

  // Serial.print("Euler WT901\t");
  // Serial.println(-1*(angle_x - offsetX), 4); 
  return (float) -1*(angle_x - offsetX);
  // Serial.print("\t"); 
  // Serial.print(angle_y - offsetY, 4); Serial.print("\t"); 
  // Serial.println(angle_z - offsetZ, 4); 

}

void setup_wt901(){
  set_200_hz();
  set_9axis();

  // uncomment/ comment out the orientation you are using 
  // set_vert_orien(); 
  set_horiz_orien();
  
  calib_gyro_and_accel();
  exit_calibration();
}

void read_wt901(){

  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }

}

void set_200_hz()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // Set to 200 HZ
  Serial1.write(change_200Hz, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}
void set_vert_orien()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // Set to 200 HZ
  Serial1.write(set_vert, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}
void set_horiz_orien()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // Set to 200 HZ
  Serial1.write(set_horiz, 5);
  delay(1000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}
void calib_gyro_and_accel()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // calibrate gyro and accel mode
  Serial1.write(calib_gy_and_acc, 5);
  delay(6000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}

void calib_magnetometer()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // calibrate the mag
  Serial1.write(calib_mag, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}

void set_height_0()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // Set the z height to 0
  Serial1.write(calib_z_0, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}

void exit_calibration()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // exit calibration
  Serial1.write(exit_calib, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}

void set_9axis()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // exit calibration
  Serial1.write(set_9ax, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}

void set_get_only_ang_out()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // exit calibration
  Serial1.write(set_ang_only_out, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}
void set_baud_rate()
{
  Serial1.write(open_com, 5);
  delay(2);
  // Unlock configuration
  Serial1.write(unlock, 5);
  delay(2);
  // exit calibration
  Serial1.write(set_baud, 5);
  delay(5000); // ensure set
  // Save configuration
  Serial1.write(save, 5);
  delay(100);
}

// ================================================================
// ===                      DC Motor Functions                  ===
// ================================================================
void setupDC(){
    // sets the pins as outputs:
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enablePin, pwmChannel);
}

void moveMotor(int output){
  //Output val of max motor movement
  if(output < 0){
    //Forward
    dutyCycle = -output;
    Serial.print("Moving forward with duty cycle "); Serial.println(dutyCycle);
    ledcWrite(pwmChannel, dutyCycle);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    //Backwards
    dutyCycle = output;
    Serial.print("Moving backward with duty cycle "); Serial.println(dutyCycle);
    ledcWrite(pwmChannel, dutyCycle);
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
}

// ================================================================
// ===                      PID Control                         ===
// ================================================================
void pidControl(float angle){
  float error = angle - balance;
  errI += error;
  
  //TO DO - Implement derivative - just angular velocity?
  float errD = 0;
  float pid = (kp * error) + (kd * errD) + (ki * errI);
  if(pid > saturation){
    pid = saturation;
  } else if(pid < -saturation){
    pid = -saturation;
  }
  
  int intPID = (int) pid;
  Serial.print("PID Output: "); Serial.println(intPID);
  moveMotor(intPID);
}

