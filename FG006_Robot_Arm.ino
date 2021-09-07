#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// X Pins
#define ELBOW_STEP_PIN       54
#define ELBOW_DIR_PIN        55 // CW when HIGH
#define ELBOW_ENABLE_PIN     38

// Y Pins
#define WRIST_P_STEP_PIN     60
#define WRIST_P_DIR_PIN      61 // CW when HIGH
#define WRIST_P_ENABLE_PIN   56

// Z Pins
#define WRIST_R_STEP_PIN     46
#define WRIST_R_DIR_PIN      48 // CW when HIGH
#define WRIST_R_ENABLE_PIN   62

// E0 Pings
#define SHOULDER_STEP_PIN    26
#define SHOULDER_DIR_PIN     28 // CCW when HIGH
#define SHOULDER_ENABLE_PIN  24

// E1 Pins
#define BASE_STEP_PIN        36
#define BASE_DIR_PIN         34 // CW when HIGH
#define BASE_ENABLE_PIN      30

#define SDPOWER              -1
#define SDSS                 53
#define LED_PIN              13

//NEMA 17
#define NEMA_MICROSTEP       16
#define NEMA_STEPS_PER_REV   200
#define NEMA_DELAY           1000 //microseconds

//Mini Stepper
#define MINI_MICROSTEP 1
#define MINI_STEPS_PER_REV   64
#define MINI_DELAY           750 // microseconds

//Gearing ratios
#define RATIO_SHOULDER       38
#define RATIO_ELBOW          15 // measure this later
#define RATIO_WRIST_PITCH    38
#define RATIO_WRIST_ROTATE   38
#define RATIO_BASE           1


#define MPU_SHOULDER         25
#define MPU_ELBOW            23
#define MPU_WRIST_P          17
#define MPU_WRIST_R          16

#define ANGLE_TOLERANCE       2 //degrees

Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_accel;

// Steps per angle
const float stepAngleBase =        1.8; // default because no gearing
const double stepAngleShoulder =   121600 / 360;
const float stepAngleElbow =       121600 / 360;
const float stepAngleWristPitch =  121600 / 360;
const float stepAngleWristRotate = 2432 / 360;

double positions[] = {0,0,0,0,0}; // base, shoulder, elbow, wrist pitch, wrist rotate


void setup() {
  
  // _________ MPU6050 Initialization _________
  
  Serial.begin(115200);
//  while (!Serial)
//    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("FG006 Startup . . . ");

  pinMode(MPU_WRIST_R, OUTPUT);
  pinMode(MPU_WRIST_P, OUTPUT);
  pinMode(MPU_ELBOW, OUTPUT);
  pinMode(MPU_SHOULDER, OUTPUT);

  //Disable MPU6050s at startup
  digitalWrite(MPU_WRIST_R, HIGH);
  digitalWrite(MPU_WRIST_P, HIGH);
  digitalWrite(MPU_ELBOW, HIGH);
  digitalWrite(MPU_SHOULDER, HIGH);

  // _________ Stepper Motor Initialization _________

  pinMode(ELBOW_STEP_PIN, OUTPUT);
  pinMode(ELBOW_DIR_PIN, OUTPUT);
  pinMode(ELBOW_ENABLE_PIN, OUTPUT);

  pinMode(WRIST_P_STEP_PIN, OUTPUT);
  pinMode(WRIST_P_DIR_PIN, OUTPUT);
  pinMode(WRIST_P_ENABLE_PIN, OUTPUT);

  pinMode(WRIST_R_STEP_PIN, OUTPUT);
  pinMode(WRIST_R_DIR_PIN, OUTPUT);
  pinMode(WRIST_R_ENABLE_PIN, OUTPUT);

  pinMode(SHOULDER_STEP_PIN, OUTPUT);
  pinMode(SHOULDER_DIR_PIN, OUTPUT);
  pinMode(SHOULDER_ENABLE_PIN, OUTPUT);

  pinMode(BASE_STEP_PIN, OUTPUT);
  pinMode(BASE_DIR_PIN, OUTPUT);
  pinMode(BASE_ENABLE_PIN, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  // activate all stepper drivers
  digitalWrite(ELBOW_ENABLE_PIN, HIGH);
  digitalWrite(WRIST_P_ENABLE_PIN, HIGH);
  digitalWrite(WRIST_R_ENABLE_PIN, HIGH);
  digitalWrite(SHOULDER_ENABLE_PIN, LOW);
  digitalWrite(BASE_ENABLE_PIN, HIGH);

  // _________ Calibration Routine _________
  
  calibration(90, SHOULDER_STEP_PIN);   // calibrate shoulder stepper angle
}

void loop() {
//    double angle = 0;
//    int stepper = 0; // position in array
//    int stepDelay = NEMA_DELAY;
//    int dirPin = BASE_DIR_PIN;
//    int enablePin = 0;
//    double angleSteps = 1.8;
//    int mpuPin = 0;
//
//    digitalWrite(dirPin, LOW);
//    digitalWrite(SHOULDER_ENABLE_PIN, LOW);
//    
//    if (millis() % 1000 < 500)
//      digitalWrite(LED_PIN, HIGH);
//    else
//      digitalWrite(LED_PIN, LOW);
//
//    Serial.println("Step");
//    digitalWrite(SHOULDER_STEP_PIN, LOW);
//    delayMicroseconds(1000);
//    digitalWrite(SHOULDER_STEP_PIN, HIGH);
//    delayMicroseconds(1000);
}
void initMPU()
{
  mpu.begin();

  // Try to initialize! Retry every second if not activated
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (!mpu.begin()) {
      Serial.println("Retrying. . . ");
      mpu.begin();
      delay(1000);
    }
  }
  
  Serial.println("MPU6050 Found!");
  delay(1000);
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
    delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
    delay(10);
  }

  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
    delay(10);
  }
}


void calibration(int target, int stepPin)
{
  double angle = 0;
  int stepper = 0; // position in array
  int stepDelay = NEMA_DELAY;
  int dirPin = BASE_DIR_PIN;
  int enablePin = 0;
  double angleSteps = 1.8;
  int mpuPin = 0;

  if(stepPin == SHOULDER_STEP_PIN)
  {
    stepper = 1;
    mpuPin = MPU_SHOULDER;
    dirPin = SHOULDER_DIR_PIN;
    angleSteps = stepAngleShoulder;
    enablePin = SHOULDER_ENABLE_PIN;
  }
  if(stepPin == ELBOW_STEP_PIN)
  {
    stepper = 2;
    mpuPin = MPU_ELBOW;
    dirPin = ELBOW_DIR_PIN;
    angleSteps = stepAngleElbow;
    enablePin = ELBOW_ENABLE_PIN;
  }
  if(stepPin == WRIST_P_STEP_PIN)
  {
    stepper = 3;
    mpuPin = MPU_WRIST_P;
    dirPin = WRIST_P_DIR_PIN;
    angleSteps = stepAngleWristPitch;
    enablePin = WRIST_P_ENABLE_PIN;
  }
  if(stepPin == WRIST_R_STEP_PIN)
  {
    stepper = 4;
    mpuPin = MPU_WRIST_R;
    dirPin = WRIST_R_DIR_PIN;
    angleSteps = stepAngleWristRotate;
    stepDelay = MINI_DELAY;
    enablePin = WRIST_R_ENABLE_PIN;
  }

  // activate the corresponding MPU6050 Pin and check current angle
  digitalWrite(mpuPin, LOW);

  initMPU();
  
  mpu_accel = mpu.getAccelerometerSensor();
  mpu_accel->printSensorDetails();
  
  sensors_event_t acc;
  
  mpu_accel = mpu.getAccelerometerSensor();

  Serial.println("Stepper enabled on: ");
  Serial.println(enablePin);
  Serial.println("Stepper stepping on: ");
  Serial.println(stepPin);
  Serial.println("Stepper direction on: ");
  Serial.print(dirPin);
  Serial.println("");
  
  mpu_accel->getEvent(&acc);
  double yAcc = acc.acceleration.y;
  double xAcc = acc.acceleration.x;

  if(yAcc > 9.81)
  {
    yAcc = 9.80;
  }
  if(yAcc < -9.81)
  {
    yAcc = -9.80;
  }
  angle = asin( yAcc/ 9.81 ) / PI * 180; //angle in degrees
  
  Serial.println("Current Angle: ");
  Serial.println(angle);
  Serial.print("Current X Accel: ");
  Serial.println(acc.acceleration.x);

  // Move Shoulder to Calibration position
  delay(1000);
  
    
  while(abs(angle - target) > (ANGLE_TOLERANCE*2))
  {

    mpu_accel->getEvent(&acc);

    //Serial.println("0");
    yAcc = acc.acceleration.y;
    xAcc = acc.acceleration.x;
    
    if(yAcc > 9.81)
    {
      yAcc = 9.80;
      Serial.println("Upper Limit!");
    }
    if(yAcc < -9.81)
    {
      yAcc = -9.80;
      Serial.println("Lower Limit!");
    }
    
    angle = asin( yAcc / 9.81 ) / PI * 180; // angle in degrees
    
    Serial.print("Current Angle: ");
    Serial.println(angle);
    Serial.print("Target: ");
    Serial.println(target);
    Serial.print("Current Y Accel: ");
    Serial.println(yAcc);
    Serial.print("Current X Accel: ");
    Serial.println(xAcc);
    Serial.println("2");

    if(angle - ANGLE_TOLERANCE < target && xAcc > 0) // when current angle is ahead of the target, spin CW
    { 
      //mpu.getEvent(&acc, &gyro, &temp);
      if(stepPin == SHOULDER_STEP_PIN) // reverse it for this stepper
      {
        digitalWrite(dirPin, LOW);
        Serial.println("Moving CW Shoulder");
      }
      else
      {
        digitalWrite(dirPin, HIGH);
        Serial.println("Moving CW");
      }
    }
    else
    {   
      //mpu.getEvent(&acc, &gyro, &temp);
      if(stepPin == SHOULDER_STEP_PIN) // reverse it for this stepper
      {
        digitalWrite(dirPin, HIGH);
        Serial.println("Moving CCW Shoulder");
      }
      else
      {
        digitalWrite(dirPin, LOW);
        Serial.println("Moving CCW");
      }
    }
    
//    Serial.println("3");
//    //digitalWrite(dirPin, HIGH);
//    //digitalWrite(enablePin, LOW);
//    // blink LED
//    Serial.println("4");
    if (millis() % 1000 < 500)
      digitalWrite(LED_PIN, HIGH);
    else
      digitalWrite(LED_PIN, LOW);

    Serial.println("Step");
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
    Serial.println("5");
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    
    
    //Serial.println("6");
    //delay(100);
  
  }

  // save current position and deactivate MPU6050
  positions[stepper] = angle;
  digitalWrite(mpuPin, HIGH);
  Serial.println("Position Set!");
  Serial.println(positions[stepper]);
  digitalWrite(LED_PIN, HIGH);
  //end of calibrate
}








//void moveStepper(double tgt, int stepPin)
//  {
//    int stepper = 0; // position in array
//    int stepDelay = NEMA_DELAY;
//    int dirPin = BASE_DIR_PIN;
//    double angleSteps = 1.8;
//    
//    if(stepPin == SHOULDER_STEP_PIN)
//    {
//      stepper = 1;
//      dirPin = SHOULDER_DIR_PIN;
//      angleSteps = stepAngleShoulder;
//    }
//    if(stepPin == ELBOW_STEP_PIN)
//    {
//      stepper = 2;
//      dirPin = ELBOW_DIR_PIN;
//      angleSteps = stepAngleElbow;
//    }
//    if(stepPin == WRIST_P_STEP_PIN)
//    {
//      stepper = 3;
//      dirPin = WRIST_P_DIR_PIN;
//      angleSteps = stepAngleWristPitch;
//    }
//    if(stepPin == WRIST_R_STEP_PIN)
//    {
//      stepper = 4;
//      dirPin = WRIST_R_DIR_PIN;
//      angleSteps = stepAngleWristRotate;
//      stepDelay = MINI_DELAY;
//    }
//
//    double current = positions[stepper];
//    double difference = abs(current-tgt);
//
//    if(difference + ANGLE_TOLERANCE > 0) // position is ahead of target
//    {
//  
//      digitalWrite(dirPin, LOW); // ccw for most stepper
//      angleSteps = -angleSteps;
//      if(stepPin == SHOULDER_STEP_PIN) // cw for shoulder stepper
//      {
//        digitalWrite(dirPin, HIGH);
//      }
//    }
//    else // position is behind of target
//    {
//      digitalWrite(dirPin, HIGH); // cw for most steppers
//      angleSteps = abs(angleSteps);
//      if(stepPin == SHOULDER_STEP_PIN) // ccw for shoulder stepper
//      {
//        digitalWrite(dirPin, LOW);
//      }
//    }
//    
//    while(difference + ANGLE_TOLERANCE > 0 || difference - ANGLE_TOLERANCE < 0)
//    {
//      difference += angleSteps;
//      positions[stepper] += angleSteps;
//      digitalWrite(stepPin, LOW);
//      delayMicroseconds(stepDelay);
//      digitalWrite(stepPin, HIGH);
//      delayMicroseconds(stepDelay);
//    }
//    
//   // end of moveStepper   
//  }

 
