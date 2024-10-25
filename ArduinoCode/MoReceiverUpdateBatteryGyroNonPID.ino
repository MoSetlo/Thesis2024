#include <esp_now.h> //esp-idf
#include <WiFi.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



#define CHANNEL 6 //Must Match Slave
#define FRONT_MOTOR_PIN 12
#define BACK_MOTOR_PIN 2
#define LEFT_MOTOR_PIN 13
#define RIGHT_MOTOR_PIN 3
#define BattADCPin 5
#define FREQUENCY 40000
#define PWM_RESOLUTION 8
#define MPU_SDA 16
#define MPU_SCL 18

/** This is all the data about the peer **/
esp_now_peer_info_t slave;  //MacAdress of Receiver
uint8_t broadcastAddress[] = {0xCC,0x8D,0xA2,0x88, 0x63, 0x4A}; //CC:8D:A2:88:63:4A
esp_err_t result;



/** The all important data**/
uint8_t dataSend = 0; //Should be 250 bytes or less - can be int, struct. Struct could be useful or partition xy adc values

typedef struct joystick{ //struct to hold xy values of two joysticks from transmitter
  uint16_t xL, yL, xR, yR=500; uint8_t leftStickClick =1; 
  float Pyaw,Ppitch,Iyaw,Ipitch,Dyaw,Dpitch=0;
};
joystick joyStk;
joystick* newJoyStk = &joyStk;


typedef struct dataLog{

int_fast8_t frontMotor=0;
int_fast8_t backMotor=0;
int_fast8_t leftMotor=0;
int_fast8_t rightMotor=0;
float RateRoll, RateYaw=0;
float PitchAngle=0;
float battStatus=0;
int recLoop=0;
float Pyaw,Ppitch,Iyaw,Ipitch,Dyaw,Dpitch=0;
};

dataLog recData;

/**Global Variables**/
int_fast8_t frontMotor;
int_fast8_t backMotor;
int_fast8_t leftMotor;
int_fast8_t rightMotor;
int_fast8_t yawStick;
int_fast8_t throttleLeftStick;
int_fast8_t pitchStick;
int_fast8_t throttleRightStick; //int incase of negative signage

//BatteryVoltage============================
float battVoltage;
uint8_t batteryLow;

//IMU Gyro===================================
Adafruit_MPU6050 mpu;

float RateRoll, RatePitch, RateYaw;
float RateRollCalib, RatePitchCalib, RateYawCalib;
int RateCalibNum;
float AccelX, AccelY, AccelZ;
float AnglePitchCalib,AnglePitch=0;
int Looptimer;

//PID Terms====================================
float Pyaw,Ppitch,Iyaw,Ipitch,Dyaw,Dpitch;
float ErrorYaw,ErrorPitchRate,ErrorPitchAngle,PrevErrorYaw,PrevErrorPitchRate,PrevErrorPitchAngle,PrevItermYaw,PrevItermPitch,PrevItermPitchAngle;
float DesiredYawRate, DesiredPitchAngle;
float PIDInputYaw,PIDInputPitch=0;

float PIDReturnIterm;
//Other

uint8_t startLoop=0;



void gyro_rates()
{

/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  RatePitch = g.gyro.y * 57.2958;

  RateRoll = g.gyro.x * 57.2958;

  RateYaw = g.gyro.z * 57.2958;

  AccelX = a.acceleration.x/9.80665;

  AccelY = a.acceleration.y/9.80665;

  AccelZ = a.acceleration.z/9.80665;


}

float PIDcontroller(float Pconst, float Iconst, float Dconst, float Error, float PrevError, float PrevIterm,float MaxOut)
{
  float Pterm = Pconst*Error;
  float Iterm = PrevIterm + (Iconst*(Error+PrevError)*0.005/2);
  float Dterm = Dconst*(Error-PrevError)/0.005;

  //AntiWindup

  if(Iterm>MaxOut){
    Iterm=MaxOut;
  }
  else if(Iterm<-MaxOut)
  {
    Iterm=-MaxOut;
  }

  float PIDout= Pterm+Iterm+Dterm;

  if(PIDout>MaxOut){
    PIDout=MaxOut;
  }
  else if(PIDout<-MaxOut)
  {
    PIDout=-MaxOut;
  }

  //PIDReturn[0] = Error;
  PIDReturnIterm = Iterm;

  return PIDout;

}







void setup(){
//blade spin stop
  pinMode(FRONT_MOTOR_PIN, OUTPUT);  // Set as output
  digitalWrite(FRONT_MOTOR_PIN, LOW); // Ensure it's off at boot
pinMode(BACK_MOTOR_PIN, OUTPUT);  // Set as output
  digitalWrite(BACK_MOTOR_PIN, LOW); // Ensure it's off at boot
  pinMode(LEFT_MOTOR_PIN, OUTPUT);  // Set as output
  digitalWrite(LEFT_MOTOR_PIN, LOW); // Ensure it's off at boot
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);  // Set as output
  digitalWrite(RIGHT_MOTOR_PIN, LOW); // Ensure it's off at boot


  /*ESPNow Setup====================================*/
  Serial.begin(115200); //Serial com
  delay(1000);
  WiFi.mode(WIFI_STA);  //Set mode to station mode
  WiFi.setChannel(CHANNEL);
  
  if (esp_now_init() != ESP_OK) { //initialises espnow protocol
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent); //send callback on delivery
  esp_now_register_recv_cb(OnDataRecv); //send callback on reception

  memcpy(slave.peer_addr, broadcastAddress, 6); //copy broadcast address memory into slave object
  slave.channel = CHANNEL; 
  slave.encrypt =0;
  esp_now_add_peer(&slave);


  /*MPU SETUP*/
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
 
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //therefore in degrees per seconds

  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);


  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);

  delay(1000);

  digitalWrite(15, LOW);
  delay(1000);

  digitalWrite(15, HIGH);
  delay(1000);

  digitalWrite(15, LOW);


  for(RateCalibNum =0 ; RateCalibNum<2000; RateCalibNum++)
  {
    gyro_rates();
    RateRollCalib+=RateRoll;
    RatePitchCalib+=RatePitch;
    RateYawCalib += RateYaw;
    AnglePitchCalib=0.98*(AnglePitchCalib+(RatePitch*0.005)) + 0.02*(atan(-AccelX/sqrt(AccelY*AccelY + AccelZ*AccelZ)) * 57.2958);
    delay(1);
  }
  
  RateRollCalib/=2000;
  RatePitchCalib/=2000;
  RateYawCalib/=2000;

  Serial.println("Calibration Complete");
  digitalWrite(15, HIGH);
  delay(1000);

  /*Joystick Setup===================================*/
  frontMotor,backMotor, leftMotor, rightMotor, yawStick, throttleLeftStick, pitchStick, throttleRightStick = 0;

  /*PWM SETUP===============================*/
  ledcAttach(FRONT_MOTOR_PIN, FREQUENCY, PWM_RESOLUTION);
  ledcWrite(FRONT_MOTOR_PIN, 0);

  ledcAttach(BACK_MOTOR_PIN, FREQUENCY, PWM_RESOLUTION);
  ledcWrite(BACK_MOTOR_PIN, 0);
  

  ledcAttach(LEFT_MOTOR_PIN, FREQUENCY, PWM_RESOLUTION);
  ledcWrite(LEFT_MOTOR_PIN, 0);

  ledcAttach(RIGHT_MOTOR_PIN, FREQUENCY, PWM_RESOLUTION);
  ledcWrite(RIGHT_MOTOR_PIN, 0);



  //ADC Battery Check
  analogReadResolution(10);
  batteryLow=0;

  //PID Setup variables
  PrevErrorYaw,PrevErrorPitchAngle,PrevItermYaw,PrevItermPitchAngle=0;

  Pyaw=(newJoyStk->Pyaw);
  Iyaw=(newJoyStk->Iyaw);
  Dyaw=(newJoyStk->Dyaw);

  Ppitch=(newJoyStk->Ppitch);
  Ipitch=(newJoyStk->Ipitch);
  Dpitch=(newJoyStk->Dpitch);

  //PIDReturn[0]=0;
  PIDReturnIterm=0;

  delay(3000);
  digitalWrite(15, LOW);
}

void loop(){
  //Remote control Lock 

  if(!newJoyStk->leftStickClick && newJoyStk->yR >850){
    startLoop=0; //Loop paused

    ledcWrite(FRONT_MOTOR_PIN, 0);
    ledcWrite(BACK_MOTOR_PIN, 0);
    ledcWrite(LEFT_MOTOR_PIN, 0 );
    ledcWrite(RIGHT_MOTOR_PIN, 0);

    PrevErrorYaw,PrevErrorPitchAngle,PrevItermYaw,PrevItermPitchAngle=0;
    digitalWrite(15, LOW);

    delay(3000);

  }

  while(!startLoop) //loop paused
  {
    //delay(3000);
    Serial.println("Waiting for Stick Click");
    if(!newJoyStk->leftStickClick && newJoyStk->yR >550)
    {
      startLoop=1;
      digitalWrite(15, HIGH);
  
  delay(1000);

  digitalWrite(15, LOW);
  delay(1000);

  digitalWrite(15, HIGH);
  delay(1000);

  digitalWrite(15, LOW);


  for(RateCalibNum =0 ; RateCalibNum<2000; RateCalibNum++)
  {
    gyro_rates();
    RateRollCalib+=RateRoll;
    RatePitchCalib+=RatePitch;
    RateYawCalib += RateYaw;
    AnglePitchCalib=0.98*(AnglePitchCalib+(RatePitch*0.005)) + 0.02*(atan(-AccelX/sqrt(AccelY*AccelY + AccelZ*AccelZ)) * 57.2958);
    delay(1);
  }
  
  RateRollCalib/=2000;
  RatePitchCalib/=2000;
  RateYawCalib/=2000;

  Serial.println("Calibration Complete");
  digitalWrite(15, HIGH);
  delay(2000);

    }
    delay(3000);
    //calll reset errror
  }

  //Loop Timer
  Looptimer = micros();

  /*Battery Voltage Check*/
  //=============================================================
  battVoltage = (float)analogRead(BattADCPin)/205+0.3; //1024 to 5V mapp

  if(battVoltage<3.45) //assumption one would need to restart system
  {
    //batteryLow=1;
    digitalWrite(15, LOW);
    // dataSend = 1;

    // /*ESP_Now*/
    // result =  esp_now_send(slave.peer_addr, &dataSend, sizeof(dataSend)); // sends data to slave
  }
  else //assumption one would need to restart system
  {
    //batteryLow=0;
    digitalWrite(15, HIGH);
    // dataSend = 1;

    // /*ESP_Now*/
    // result =  esp_now_send(slave.peer_addr, &dataSend, sizeof(dataSend)); // sends data to slave
  }

  /*JoyStick PWM DUTY Mapping====================*/
  //====================================================
  if(newJoyStk->yL >470)  //Throttle input left joystick from joystick middle (forward propulsion) 
  {
    throttleLeftStick=0;
  }

  else if(newJoyStk->yL <=470)
  {
    throttleLeftStick = map(newJoyStk->yL, 470, 0, 0, 85); //maximum 60% speed Note int8
    
  }

  //============================================
  // Throttle input right joystick from joystick middle(lift)

  if(newJoyStk->yR>=550)
  {
    throttleRightStick=0; //10% hover
  }

  else if(newJoyStk->yR>470 && newJoyStk->xL <550)
  {
    throttleRightStick= 0; //1% hover, just to show it works
  }

  else if(newJoyStk->yR<=470)
  {
    throttleRightStick = map(newJoyStk->yR,470,0,2,125); //maximum 60% lift speed hover 
  }

  //=====================================================
  if(newJoyStk->xL <=470) //Right yaw
  {
    yawStick = map(newJoyStk->xL, 470,0,0,-95); //maximum 20% might change for no massive pwm
  }

  else if(newJoyStk->xL >=550) //Left yaw
  {
    yawStick = map(newJoyStk->xL, 550,1023,0,+95);
  }


  else if(newJoyStk->xL >470 && newJoyStk->xL <550 )
  {
    yawStick = 0;
  }

//MPU======================================
  //Looptimer = micros();
  gyro_rates();
  RateRoll-=RateRollCalib;
  RatePitch -= RatePitchCalib;
  RateYaw -= RateYawCalib;
  AnglePitch=0.98*(AnglePitch+(RatePitch*0.005)) + 0.02*((atan(-AccelX/sqrt(AccelY*AccelY + AccelZ*AccelZ)) * 57.2958)-AnglePitchCalib);

  Pyaw=(newJoyStk->Pyaw);
  Iyaw=(newJoyStk->Iyaw);
  Dyaw=(newJoyStk->Dyaw);

  Ppitch=(newJoyStk->Ppitch);
  Ipitch=(newJoyStk->Ipitch);
  Dpitch=(newJoyStk->Dpitch);

  

  // DesiredYawRate = 0.19*(yawStick);
  // ErrorYaw= DesiredYawRate-RateYaw;

  // DesiredPitchAngle=0;
  // ErrorPitchAngle = DesiredPitchAngle-AnglePitch;


  // PIDInputYaw = PIDcontroller(Pyaw, Iyaw, Dyaw, ErrorYaw, PrevErrorYaw, PrevItermYaw,51);
  // PrevErrorYaw = ErrorYaw;
  // PrevItermYaw = PIDReturnIterm;


  // PIDInputPitch = PIDcontroller(Ppitch, Ipitch, Dpitch, ErrorPitchAngle, PrevErrorPitchAngle, PrevItermPitchAngle, 51);
  // PrevErrorPitchAngle = ErrorPitchAngle;
  // PrevItermPitchAngle = PIDReturnIterm;


  leftMotor = throttleLeftStick - yawStick;
  rightMotor = throttleLeftStick + yawStick;
  frontMotor = throttleRightStick;
  backMotor = throttleRightStick;

  if(leftMotor<0)
  {
    leftMotor=0;
  }
  if(rightMotor<0)
  {
    rightMotor=0;
  }
  if(frontMotor<0)
  {
    leftMotor=0;
  }
  if(backMotor<0)
  {
    rightMotor=0;
  }

  if(leftMotor>254)
  {
    leftMotor=254;
  }
  if(rightMotor>254)
  {
    rightMotor=254;
  }
  if(frontMotor>254)
  {
    frontMotor=254;
  }
  if(backMotor>254)
  {
    backMotor=254;
  }




  ledcWrite(FRONT_MOTOR_PIN, frontMotor);
  ledcWrite(BACK_MOTOR_PIN, frontMotor);
  ledcWrite(LEFT_MOTOR_PIN, leftMotor );
  ledcWrite(RIGHT_MOTOR_PIN, rightMotor);


  recData.frontMotor = frontMotor;
  recData.backMotor = backMotor;
  recData.leftMotor = leftMotor;
  recData.rightMotor = rightMotor;
  recData.PitchAngle = AnglePitch;
  recData.RateYaw = RateYaw;
  recData.RateRoll = RateRoll;
  recData.battStatus = battVoltage;
  recData.recLoop = micros()-Looptimer;
  recData.Ppitch = Ppitch;
  recData.Pyaw = Pyaw;
  recData.Ipitch = Ipitch;
  recData.Iyaw = Iyaw;
  recData.Dpitch = Dpitch;
  recData.Dyaw = Dyaw;
  

  

  result =  esp_now_send(slave.peer_addr, (uint8_t*)&recData, sizeof(recData)); // sends data to slave

  //Serial.printf("%d \t %d \t %.2f \t %d \t %d \t %.2f \t %.2f \t %d \t %.2f\n", frontMotor,backMotor,AnglePitch,leftMotor, rightMotor,RateYaw, RateRoll, micros()-Looptimer ,battVoltage);
  while(micros()-Looptimer < 5000);
  

}


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) //callback, expectation for 0 output
{
  // Serial.print("Data status ->");
  // Serial.println(status);
}

void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len) //callback on reception
{
  newJoyStk = (joystick*) data;
  // Serial.print("Data received by ->");
  // Serial.println(*data);
  // memcpy(&newData, data, sizeof(newData));

}
