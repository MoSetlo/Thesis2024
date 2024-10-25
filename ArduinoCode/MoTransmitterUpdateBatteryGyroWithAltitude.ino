#include <esp_now.h> //esp-idf
#include <WiFi.h>
#include <esp_timer.h>

#define CHANNEL 6 //Must Match Slave
#define X_Left 5
#define Y_Left 3
#define X_Right 9
#define Y_Right 7
#define altitudeHoldBtn 12



/** This is all the data about the peer **/
esp_now_peer_info_t slave;  //MacAdress of Receiver
uint8_t broadcastAddress[] = {0xCC,0x8D,0xA2,0x8B, 0xFF, 0x7A};
esp_err_t result;



/** The all important data! **/
//uint8_t data = 0; //Should be 250 bytes or less - can be int, struct. Struct could be useful or partition xy adc values
typedef struct joystick{ //struct to hold xy values of two joysticks
  uint16_t xL, yL, xR, yR=500; uint8_t leftStickClick=1; 
  float Pyaw,Ppitch,Iyaw,Ipitch,Dyaw,Dpitch=0;
};
joystick joyStk;

typedef struct dataLog{

int_fast8_t frontMotor=0;
int_fast8_t backMotor=0;
int_fast8_t leftMotor=0;
int_fast8_t rightMotor=0;
float RateRoll, RateYaw=0;
float PitchAngle=0;
float battStatus=4;
int recLoop=0;
float Pyaw,Ppitch,Iyaw,Ipitch,Dyaw,Dpitch=0;

};

dataLog recData;
dataLog *newRecData = &recData;


uint8_t startLoop=0;

//dataReceice
uint8_t dataReceive=0;


/**Packet timer rate**/
uint_fast8_t counter=0;
uint_fast64_t start=0;
uint_fast64_t end = 0;

//Others
String PIDsym;
float PIDval;
int LoopTimer=0;

//ExternalInterrupt
uint8_t altHoldStatus=0;
int_fast8_t externalCounter=0;
int_fast8_t altHoldVal=512;
uint32_t debounceTimer=0;


// void IRAM_ATTR altitudeHold()
// {

  
//   if(!altHoldStatus)
//   {
//     altHoldStatus=1;
//     altHoldVal= analogRead(X_Right); 
//     digitalWrite(39,HIGH);
//     externalCounter=0;
//   }
//   else if(altHoldStatus && altHoldVal>470)
//   {
//     altHoldStatus=0;
//     digitalWrite(39,LOW);

//   }



//}

void setup(){
  /*ESP_Now setup*/
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
  

  /*Joystick Module Setup*/
  analogReadResolution(10); //10 bit resolution
  pinMode(11,INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  //attachInterrupt(altitudeHoldBtn, altitudeHold, FALLING);
  
  /*Digital LED Pin 15 setup*/
  pinMode(15,OUTPUT);
  digitalWrite(15,HIGH);

  pinMode(39, OUTPUT);
  digitalWrite(39, LOW);
  
}

void loop(){ 
if(Serial.available()>0)
{
  PIDsym = Serial.readString();
  while(Serial.available()==0){Serial.println("Waiting float");}

  PIDval = Serial.parseFloat();


  if(PIDsym.equalsIgnoreCase("PP"))
  {
    joyStk.Ppitch = PIDval;
  }
  else if(PIDsym.equalsIgnoreCase("IP"))
  {
    joyStk.Ipitch = PIDval;
  }
  else if(PIDsym.equalsIgnoreCase("DP"))
  {
    joyStk.Dpitch = PIDval;
  }
  else if(PIDsym.equalsIgnoreCase("PY"))
  {
    joyStk.Pyaw = PIDval;
  }
  else if(PIDsym.equalsIgnoreCase("IY"))
  {
    joyStk.Iyaw = PIDval;
  }
  else if(PIDsym.equalsIgnoreCase("DY"))
  {
    joyStk.Dyaw = PIDval;
  }



}

if(!digitalRead(altitudeHoldBtn) && (micros()-debounceTimer)>200000)
{
  
  if(!altHoldStatus)
  {
    altHoldStatus=1;
    altHoldVal= analogRead(X_Right); 
    digitalWrite(39,HIGH);
    externalCounter=0;
  }
  else if(altHoldStatus && altHoldVal>470)
  {
    altHoldStatus=0;
    digitalWrite(39,LOW);

  }

  debounceTimer=micros();
}

LoopTimer = micros();

  /*Joystick Module Loop*/
  

  joyStk.yL = analogRead(X_Left); joyStk.xL= analogRead(Y_Left); joyStk.xR = analogRead(Y_Right), joyStk.leftStickClick = digitalRead(11) ; joyStk.yR = analogRead(X_Right);  //Read of all joysticks

  if(altHoldStatus)
  {
    externalCounter++;

    if(externalCounter==100){digitalWrite(39,HIGH);}

    else if (externalCounter==200){
    
    externalCounter=0;
      

    if(joyStk.yR <470 && joyStk.yR>150)
    {
      altHoldVal-=5;
      digitalWrite(39,LOW);

    }
    else if(joyStk.yR <=150) //20 seconds to max power of 150
    {
      altHoldVal-=25;
      digitalWrite(39,LOW);
    }
    else if(joyStk.yR >550 && joyStk.yR <870)
    {
      altHoldVal+=5;
      digitalWrite(39,LOW);
    }
    else if(joyStk.yR >=870) //20 seconds to max power of 150
    {
      altHoldVal+=25;
      digitalWrite(39,LOW);
    }
    }


    if(altHoldVal<0)
    {
      altHoldVal=0;
    }
    else if(altHoldVal>1000)
    {
      altHoldVal=1000;
    }
    joyStk.yR = altHoldVal;



  }
  else
  {
    joyStk.yR = analogRead(X_Right); 
  }

  // while(digitalRead(11) && !startLoop )
  // {
  //   Serial.println("Waiting for Start");
  // }

  //startLoop=1;

  /*Battery Low Voltage Tranmission*/
  if(newRecData->battStatus<3.45)
  {
    digitalWrite(15, LOW);
  }
  else
  {
    digitalWrite(15, HIGH);
  }

  /*ESP_Now Loop*/
  result =  esp_now_send(slave.peer_addr, (uint8_t*)&joyStk, sizeof(joystick)); // sends data to slave

  if(result == ESP_OK && counter==50)
  {
    start = esp_timer_get_time();
  }

  if(result == ESP_OK)
  {
    counter++;
  }


  if(result == ESP_OK && counter==250)
  {
    counter=0;
    end = esp_timer_get_time();
    //Serial.printf("%llu\n",(end - start)/1000);
    

  }

  Serial.printf("%d \t, %d \t, %.2f \t, %d \t, %d \t, %.2f \t, %.2f \t, %d \t, %.2f\t, %.3f\t, %.3f\t, %.3f\t, %.3f\t, %.3f\t, %.3f\t,", newRecData->frontMotor/*altHoldStatus*/,newRecData->backMotor/*altHoldVal*/,newRecData->PitchAngle,newRecData->leftMotor, newRecData->rightMotor,newRecData->RateYaw,newRecData-> RateRoll,newRecData->recLoop,
  newRecData->battStatus,newRecData->Ppitch, newRecData->Ipitch,newRecData->Dpitch,newRecData->Pyaw, newRecData->Iyaw, newRecData->Dyaw );

  Serial.println(micros()-LoopTimer);
  


  //delay(3000);
  // data++;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) //callback, expectation for 0 output
{
  // Serial.print("Data status ->");
  // Serial.println(status);
}

void OnDataRecv(const esp_now_recv_info_t * esp_now_info, const uint8_t *data, int data_len) //callback on reception
{
  //dataReceive = *data;

  newRecData = (dataLog*)data;
  // Serial.print("Data received by ->");
  // Serial.println(*data);
  // memcpy(&newData, data, sizeof(newData));

}
