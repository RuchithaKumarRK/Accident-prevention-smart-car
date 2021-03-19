#include <string.h>

/******************** send and recieve from/to cloud start *****************/

/******** send to cloud start ********/
unsigned long sendingTimer = 0;
/*float temperature=32.20;
float humidity=50.20;*/
/******** send to cloud end ********/


/****** recieve and phrase data start *******/

#define CHAR_TIMEOUT           1
#define TOTAL_TIMEOUT          2
#define READ_SUCCESS           true
#define READ_FAIL              false

char robot_voice_command[15]={0};

unsigned int i = 0;
unsigned long timerStart, prevChar = 0;
char buffer1[80]={0};

/****** recieve and phrase data end *******/
bool phraseSerialdata(char  *buffer1);
void recivedata(char  *buffer1,int count);


/******************** send and recieve from/to cloud end *****************/

/********* CLOUD MONITORING PARAMETERS ********/
String alcohol_cloud="updating";
String speed_cloud="updating";
String seatbelt_cloud="updating";
char   heart_cloud[8]={0};
String location_cloud="updating";
String lat_cloud="NO";
String lon_cloud="NO";
String carmode_cloud="updating";
String licence_cloud="updating";
/********* CLOUD MONITORING PARAMETERS ********/

/************************* GSM START *******************************/
#include <SoftwareSerial.h>


#include <string.h>
#include "SoftReset.h"
#include "wevonix_GPRS_Shield_Arduino.h"
/**************************** GSM END ****************************/

/******************** heartbeat sensor start ****************/

#define HEART_BEAT_SEN  2   /*Interrupt usable pin on Mega*/

volatile bool initial_beat = true;
volatile bool first_beat = true;
volatile bool third_beat = false;

volatile unsigned long first_beat_time = 0;
volatile unsigned long second_beat_time = 0;
volatile unsigned long third_beat_time = 0;
volatile unsigned long difference_beat_time = 0;

volatile unsigned int beat_count=0;

/********************* heartbeat sensor end ***********/


/************************** GPS START ****************************/
#include <stdlib.h>
#include <TinyGPS++.h>
//ALSO ADD #include <SoftwareSerial.h>
/************************ GPS END ******************************/

/************************* GSM START *******************************/
#define PIN_TX    7   /*connect to TX1 ON MEGA NOT TO PIN 7*/
#define PIN_RX    8   /*connect to RX1 ON MEGA NOT TO PIN 8*/
//#define PIN_POWER   9
#define BAUDRATE  9600
#define MESSAGE_LENGTH 30

// Pump actuation pin
#define PUMP_ON   10
#define PUMP_OFF  11  

// For debugging interface
#define DEBUG
#define DELAY_TIME 1000

// Rings indentifier
#define RINGS_TURN_ON   3
#define RINGS_TURN_OFF    5
#define RINGS_STATUS    7
#define PB_ENTRY_INDEX_LOCATION 1
#define SMS_REPLY_LOCATION 2
#define DATA_REPLY_LOCATION 3
#define MISSEDCALL_LOCATION 4

// Global Constant
const char* company_name = "thingTronics";
const char* hardware_version = "v1.0";
const char* software_version = "v1.1";

// Global Varables
bool rebootFlag = false;
bool smsReplyFlag = true;
bool missedCallFlag = true;
bool dataFlag = true;
char *password = "ABCDEF";  // Default password
unsigned int PBEntryIndex = 1;
byte messageIndex = 0;
char gsmBuffer[64];
char *s = NULL;
bool pumpFlag = false;

/*char userNumber[15]="+919740075014";
char hospital[15]="+918073808403";*/

char userNumber[15]="+918073086970";
char hospital[15]="+918050891821";

// Create an instance of GPRS library
GPRS gsm(PIN_RX, PIN_TX, BAUDRATE);

// Main function starts here

/**************************** GSM END ****************************/

/************************** GPS START ***************************/
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;


// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

unsigned long gpsTimeout=0;

char gpsLAT[10]={0};
char gpsLON[10]={0};
/*********************** GPS END ******************************/


/******************** RFID START **************************/
#include <SPI.h>
#include <RFID.h>
#define SDA_DIO              9
#define RESET_DIO            8


#define TIME_OUT             10000
#define RFID_FOUND           true
#define RFID_NOT_FOUND       false

#define DL_VALID             1
#define DL_NOT_VALID         2
#define LL_VALID             3
#define LL_NOT_VALID         4
#define INVALID_CARD         0

#define BUTTON_PRESSED       false
#define BUTTON_NOT_PRESSED   true
#define CARD_MATCHED         false

RFID RC522(SDA_DIO, RESET_DIO);

String        cardNum;
char          cardNumber[15];
unsigned long card;


const char dlValid[]         = "1931179049223";
const char dlNotValid[]      = "17717817449156";
const char llValid[]         = "193702364990";
const char llNotValid[]      = "19518414449218";

/************************ RFID END ************************/

/********************** LCD START **********************/
#include <Wire.h>                 //LCD
#include <LiquidCrystal_I2C.h> 
LiquidCrystal_I2C lcd(0x27, 20, 4);
/******************* LCD END **************************/

#define MQ135                       A0
#define START_BUTTON                30
#define HEARTBEATABNORM_SW          32
#define ACCIDENT_SW                 34
#define IGNITION_OFF                36
#define PANIC_BUTTON                38
#define RED_LED                     40
#define GREEN_LED                   42
#define PARKING_LED                 12
#define BUZZER                      44
#define SEAT_BELT                   46
#define RIGHT_MOTOR_IN1             24
#define RIGHT_MOTOR_IN2             22
#define LEFT_MOTOR_IN3              28
#define LEFT_MOTOR_IN4              26
#define RIGHT_MOTOR_EN1             3
#define LEFT_MOTOR_EN2              4




int ignition_test_loop = 1;

bool seat_belt_count = false; //initially seat belt unplugged

int antitheft_system = 1; // 1 - default state car will run, 2- sms permission to run car, 3- stop the car via sms

void setup() 
{
   Serial.begin(9600); 
/******************** send and recieve from/to cloud start *****************/
  Serial2.begin(9600);
/******************** send and recieve from/to cloud end *****************/  
   SPI.begin();
   RC522.init();
   lcd.begin();   
   lcd.clear();
   lcd.setCursor(8,0);   /* (COL,ROW) */
   charbychar_disp("SMART");
   lcd.setCursor(5,1);   
   charbychar_disp("AUTOMOTIVE");
   lcd.setCursor(0,2);   
   charbychar_disp("--------------------");
   lcd.setCursor(0,3);   
   charbychar_disp("INITIALIZING . . . .");
   delay(5000);
   
   /********* gsm START void setup ***********/  
  InitHardware();
    
  SIMCardSetup();
  
  // Setup SMS Service
  SMSServiceSetup();  
  Serial.println(F("Listening for SMS or Call"));
/********* gsm END void setup ***********/

/*********** GPS START ******************/
  ss.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
/********* GPS END *********************/

/********** heartbeat start **********/
  pinMode(HEART_BEAT_SEN,INPUT);
  attachInterrupt(digitalPinToInterrupt(2), heart_beat_check, HIGH);
/*********** heartbeat end ***********/
 



   pinMode(SEAT_BELT,INPUT_PULLUP);
   pinMode(START_BUTTON,INPUT_PULLUP);
   pinMode(ACCIDENT_SW,INPUT_PULLUP);
   pinMode(HEARTBEATABNORM_SW,INPUT_PULLUP);
   pinMode(IGNITION_OFF,INPUT_PULLUP);
   pinMode(PANIC_BUTTON,INPUT_PULLUP);

   pinMode(RIGHT_MOTOR_IN1,OUTPUT);
   pinMode(RIGHT_MOTOR_IN2,OUTPUT);
   pinMode(LEFT_MOTOR_IN3,OUTPUT);
   pinMode(LEFT_MOTOR_IN4,OUTPUT);
   pinMode(RIGHT_MOTOR_EN1,OUTPUT);
   pinMode(LEFT_MOTOR_EN2,OUTPUT);

   digitalWrite(RIGHT_MOTOR_IN1,LOW);
   digitalWrite(RIGHT_MOTOR_IN2,LOW);
   digitalWrite(LEFT_MOTOR_IN3,LOW);
   digitalWrite(LEFT_MOTOR_IN4,LOW);
   
   pinMode(GREEN_LED,OUTPUT);
   pinMode(RED_LED,OUTPUT);
   pinMode(PARKING_LED,OUTPUT);
   pinMode(BUZZER,OUTPUT);
   digitalWrite(GREEN_LED,LOW);
   digitalWrite(RED_LED,LOW);
   digitalWrite(PARKING_LED,LOW);
   digitalWrite(BUZZER,LOW);


  gsm.sendSMS(userNumber, "Device: Smart Automotive Initialized.");

}

void loop() 
{
    // Wait for SMS or call
  if(gsm.readable()) {
    // Print the buffer
    sim900_read_buffer(gsmBuffer,sizeof(gsmBuffer),DEFAULT_TIMEOUT);
    Serial.println(gsmBuffer);
    // If incoming is call    
    // RING
    // +CLIP: "+91XXXXXXXXXX",145,"",,"TT+91XXXXXXXXXX",0
    if(NULL != strstr(gsmBuffer,"+CLIP:")) {
      if(missedCallFlag) {
        Serial.println(F("handling calls"));
        handleRings();
      }
    }
    
    // If incoming is SMS
    // +CMTI: "SM", 2
    messageIndex = gsm.isSMSunread();
    if(messageIndex > 0) {
      Serial.println(F("handling SMS"));
      handleSMS(messageIndex);
      messageIndex = 0;
    }
    
    sim900_clean_buffer(gsmBuffer,sizeof(gsmBuffer));
  }
  
  else
  {
    delay(100); 
  }

  if(ignition_test_loop == 1)
  {
    stop_motor();
    digitalWrite(GREEN_LED,LOW);
    digitalWrite(RED_LED,HIGH);
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   TEST       STATUS");
    lcd.setCursor(0,1);
    lcd.print("1] ALCOHOL  : ---   ");
    lcd.setCursor(0,2);
    lcd.print("2] SEAT-BELT: ---   ");
    lcd.setCursor(0,3);
    lcd.print("3] LICENCE  : ---   ");
    
    ignition_test();
    
    ignition_test_loop = 2;
    digitalWrite(RED_LED,LOW);
    digitalWrite(BUZZER,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
    digitalWrite(GREEN_LED,HIGH);
    
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("1] SPEED    : Normal");
    lcd.setCursor(0,1);
    lcd.print("2] ALCOHOL  : NO    ");
    lcd.setCursor(0,2);
    lcd.print("3] SEAT-BELT:Plugged");
    lcd.setCursor(0,3);
    lcd.print("3] HEART-BPM:       ");
    
  }
  ignition_success();

  if( digitalRead(PANIC_BUTTON) == 0)
  {
    delay(100);
    digitalWrite(RED_LED,HIGH);
    digitalWrite(BUZZER,HIGH);
    delay(600);
    digitalWrite(BUZZER,LOW);
    char sms_location_link[100]={0};
    gpsTimeout=millis();
    Serial.println("PANIC BUTTON PRESSED");
    gsm.sendSMS(userNumber, "Alert: Panic Button Triggered !!!");   //sms to 3rd person
    delay(100);
    if (triggerGPS() == true)
    {
        sprintf(sms_location_link,"Panic Location: http://maps.google.com/maps?q=%s,%s",gpsLAT,gpsLON);
        Serial.println(sms_location_link);
        lat_cloud=gpsLAT;
        lon_cloud=gpsLON;        
        gsm.sendSMS(userNumber, sms_location_link);   //sms to 3rd person
        location_cloud=sms_location_link;
    }
    else
    {
        sprintf(sms_location_link,"Panic Location: Updating ...");
        Serial.println(sms_location_link);
        lat_cloud="NO";
        lon_cloud="NO";  
        gsm.sendSMS(userNumber, sms_location_link);   //sms to 3rd person
        location_cloud=sms_location_link;
    }
    gsm.callUp(userNumber);
    send_to_cloud();
    digitalWrite(RED_LED,LOW);
  }

  if( digitalRead(ACCIDENT_SW) == 0)
  {
    delay(100);
    digitalWrite(RED_LED,HIGH);
    digitalWrite(BUZZER,HIGH);
    delay(600);
    digitalWrite(BUZZER,LOW);
    char sms_location_link[100]={0};
    gpsTimeout=millis();
    Serial.println("Accident switch PRESSED");
    gsm.sendSMS(userNumber, "Alert: Accident Condition detected!!!");   //sms to 3rd person
    delay(100);
    gsm.sendSMS(hospital, "Emergency: Please direct the nearest Ambulance");    //sms to hospital
    delay(100);
    if (triggerGPS() == true)
    {
        delay(100);
        sprintf(sms_location_link,"Accident Location: http://maps.google.com/maps?q=%s,%s",gpsLAT,gpsLON);
        Serial.println(sms_location_link);
        lat_cloud=gpsLAT;
        lon_cloud=gpsLON;
        gsm.sendSMS(userNumber, sms_location_link);   //sms to 3rd person
        delay(100);
        gsm.sendSMS(hospital, sms_location_link);     //sms to hospital
        location_cloud=sms_location_link;
    }
    else
    {
        sprintf(sms_location_link,"Accident Location: Updating ...");
        Serial.println(sms_location_link);
        lat_cloud="NO";
        lon_cloud="NO";
        gsm.sendSMS(userNumber, sms_location_link);   //sms to 3rd person
        delay(100);
        gsm.sendSMS(hospital, sms_location_link);     //sms to hospital
        location_cloud=sms_location_link;
    }
    delay(100);
    gsm.callUp(userNumber);     //call to 3rd person
    send_to_cloud();
    digitalWrite(RED_LED,LOW);
  }

  if( digitalRead(HEARTBEATABNORM_SW) == 0)
  {
    delay(100);
    digitalWrite(RED_LED,HIGH);
    digitalWrite(BUZZER,HIGH);
    digitalWrite(PARKING_LED,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
    digitalWrite(PARKING_LED,LOW);
    delay(500);
    digitalWrite(BUZZER,HIGH);
    digitalWrite(PARKING_LED,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
    digitalWrite(PARKING_LED,LOW);
    delay(500);
    digitalWrite(BUZZER,HIGH);
    digitalWrite(PARKING_LED,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
    digitalWrite(PARKING_LED,LOW);
    delay(300);
    char sms_location_link[100]={0};
    gpsTimeout=millis();
    Serial.println("Heart beat abnormal switch PRESSED");
    gsm.sendSMS(userNumber, "Alert: Abnormal Health Condition detected!!!");    //sms to 3rd person
    delay(100);
    gsm.sendSMS(hospital, "Emergency: Please direct the nearest Ambulance");    //sms to hospital
    delay(100);
    if (triggerGPS() == true)
    {
        delay(100);
        sprintf(sms_location_link,"Abnormal Health Location: http://maps.google.com/maps?q=%s,%s",gpsLAT,gpsLON);
        Serial.println(sms_location_link);
        lat_cloud=gpsLAT;
        lon_cloud=gpsLON;
        gsm.sendSMS(userNumber, sms_location_link);   //sms to 3rd person
        delay(100);
        gsm.sendSMS(hospital, sms_location_link);     //sms to hospital
        location_cloud=sms_location_link;
    }
    else
    {
        sprintf(sms_location_link,"Abnormal Health Location: Updating ...");
        Serial.println(sms_location_link);
        lat_cloud="NO";
        lon_cloud="NO";
        gsm.sendSMS(userNumber, sms_location_link);   //sms to 3rd person
        delay(100);
        gsm.sendSMS(hospital, sms_location_link);     //sms to hospital
        location_cloud=sms_location_link;
    }
    delay(100);
    gsm.callUp(userNumber);       //call to 3rd person
    send_to_cloud();
    digitalWrite(RED_LED,LOW);
  }
}

/************ igintion fun calls start *******************/
void ignition_test()
{
  delay(10000);
  carmode_cloud="PARK";
  speed_cloud="NO";
  licence_cloud="NO";
  location_cloud="NO";
  while(true)
  {
    if(analogRead(MQ135) < 400)
    {
      lcd.setCursor(13,1);
      lcd.print(" NO    ");
      alcohol_cloud="NO";
      while(true)
      {
        send_to_cloud();
        if(seat_belt_judgement() == true)
        {
          delay(300);
          lcd.setCursor(13,2);
          lcd.print("Plugged");
          lcd.setCursor(13,3);
          lcd.print("Waiting");
          seatbelt_cloud="PLUGGED";
          while(true)
          {
            send_to_cloud();
            if(seat_belt_judgement() == true) 
            {
              delay(300);
              lcd.setCursor(13,2);
              lcd.print("Plugged");
              lcd.setCursor(13,3);
              lcd.print("Waiting");
              seatbelt_cloud="PLUGGED";
              if(digitalRead(START_BUTTON) == 0)
              {
                break;
              }
            }
            if(seat_belt_judgement() == false)
            {
              lcd.setCursor(13,2);
              lcd.print("Unplug ");
              lcd.setCursor(13,3);
              lcd.print(" ----  ");
              seatbelt_cloud="UNPLUGGED";         
            }
          }
          break;
        }
        else
        {
              lcd.setCursor(13,2);
              lcd.print("Unplug ");
              lcd.setCursor(13,3);
              lcd.print(" ----  ");
              seatbelt_cloud="UNPLUGGED";          
        }

      }
      while(true)
      {
        send_to_cloud();
        if(startScreen() == true)
        {
          send_to_cloud();
          break;
        }
      }
      break;
    }
    else
    {
      lcd.setCursor(13,1);
      lcd.print("Yes!!! ");
      alcohol_cloud="FOUND";
      speed_cloud="NO";
      licence_cloud="NO";
      location_cloud="NO";
      send_to_cloud();
    }
  }
}

void ignition_success()
{
  if((antitheft_system == 1) || (antitheft_system == 2))
  {
      carmode_cloud="DRIVE";
    
      if(digitalRead(IGNITION_OFF) == 0)
      {
        delay(300);
        carmode_cloud="PARK";
        speed_cloud="NO";
        licence_cloud="NO";
        location_cloud="NO";
        ignition_test_loop = 1;
        stop_motor();
        digitalWrite(BUZZER,HIGH);
        delay(300);
        digitalWrite(BUZZER,LOW);
        delay(300);
        digitalWrite(BUZZER,HIGH);
        delay(300);
        digitalWrite(BUZZER,LOW);
      }
      if(analogRead(A0) >= 520)
      {
        alcohol_cloud="FOUND";
        speed_cloud="LOW";
        lcd.setCursor(14,0);
        lcd.print("LOW   ");
        lcd.setCursor(13,1);
        lcd.print(" Yes!!!");
        low_speed_run();
      }
      else
      {
        alcohol_cloud="NO";
        speed_cloud="NORMAL";
        lcd.setCursor(14,0);
        lcd.print("Normal");
        lcd.setCursor(13,1);
        lcd.print(" NO    ");
        high_speed_run();
      }
      if(seat_belt_judgement() == false)
      {
        seatbelt_cloud="UNPLUGGED";
        lcd.setCursor(13,2);
        lcd.print("Unplug ");
        digitalWrite(BUZZER,HIGH);
        delay(300);
        digitalWrite(BUZZER,LOW);
        delay(300);
      }
      else
      {
        seatbelt_cloud="PLUGGED";
        lcd.setCursor(13,2);
        lcd.print("Plugged"); 
      }
      heart_beat_count();
  }
  send_to_cloud();
}

/********************** ignition fun calls end ******************/

/*************************** GSM START **************************/

void InitHardware(void) {
  
  Serial.begin(BAUDRATE);
  
  //while(!Serial);
  
  
  // Printing company info
  Serial.print(F("Comapany Name: "));
  Serial.println(company_name);
  Serial.print(F("Hardware Version: "));
  Serial.println(hardware_version);
  Serial.print(F("Software Version: "));
  Serial.println(software_version);
  Serial.println(F("Setting up the hardware"));
  
  
  
  
  while(!gsm.init()) {
    Serial.println(F("Initialisation error"));
    delay(DELAY_TIME);
  //  soft_restart();
  }
  

  sim900_check_with_cmd(F("ATE0\r\n"),"OK\r\n",CMD);
  delay(DELAY_TIME);
  // Send AT+CLIP=1
  sim900_check_with_cmd(F("AT+CLIP=1\r\n"),"OK\r\n",CMD);
  delay(DELAY_TIME);
  
}


void SIMCardSetup(void) {
    
 
  char imei[16];
  // Send AT+GSN
  gsm.getIMEI(imei);
  Serial.print(F("Received IMEI "));
  Serial.println(imei);
  
 
  int rssi, ber;
  
  gsm.getSignalStrength(&rssi);

  Serial.print(F("Signal Strength is "));
  Serial.println(rssi);

  // Check for SIM registration
  // Send AT+CREG?
  int registered, networkType;
  gsm.getSIMRegistration(&networkType);

  Serial.print(F("registered is "));
  Serial.println(registered);
  Serial.print(F("network type is "));
  Serial.println(networkType);
/*
  if(registered == 0 || registered == 5) {
    Serial.println(F("SIM is registered"));
  }
  else {
    Serial.println(F("SIM is not registered"));
    soft_restart();
  }
  */
}


void SMSServiceSetup(void) {
  // Set SMS input mode
  Serial.println(F("Setting input mode"));
  // send AT+CMGF=1
  sim900_check_with_cmd(F("AT+CMGF=1\r\n"),"OK\r\n",CMD);
  delay(DELAY_TIME);
  
  // Set the input character set
  Serial.println(F("Setting character mode: GSM"));
  // Send AT+CSCS="GSM"
  sim900_check_with_cmd(F("AT+CSCS=\"GSM\"\r\n"),"OK\r\n",CMD);
  delay(DELAY_TIME);
  
  // Set SMS mode
  Serial.println(F("Setting SMS message indications"));
  // Send AT+CNMI=2,1,0,0,0
  sim900_check_with_cmd(F("AT+CNMI=2,1,0,0\r\n"),"OK\r\n",CMD);
  delay(DELAY_TIME);
  
  // Save the settings for SMS
  Serial.println(F("Saving settings to memory"));
  // Send AT+CSAS=0
  sim900_check_with_cmd(F("AT+CSAS=0\r\n"),"OK\r\n",CMD);
  delay(DELAY_TIME);
  
  Serial.println(F("SMS Service Ready"));
}

void handleRings(void) {
  byte counter = 1;
  byte state = -1;
  char authString[64];
  char mobileNumber[16];
  char *s;
  
  Serial.println("wevonix-Inside handleRing");
  strcpy(authString,gsmBuffer);
  // count the number of rings till you get RELEASE
  do {
    sim900_clean_buffer(gsmBuffer,sizeof(gsmBuffer));
    sim900_read_buffer(gsmBuffer,sizeof(gsmBuffer),DEFAULT_TIMEOUT);
  
    if(NULL != strstr(gsmBuffer,"+CLIP:")) {
      counter++;
      if(counter > 2)   /*value was 3*/
        break;
    }

  }while(NULL == strstr(gsmBuffer,"NO CARRIER"));
  
  Serial.print(counter);
  Serial.println(F(" RINGS"));
  if((NULL == strstr(gsmBuffer,"NO CARRIER") && counter > 2)) {
    getNumberFromString(authString, mobileNumber);
    Serial.print("wevonix-authString:");
    Serial.println(authString);
    Serial.print("wevonix-mobileNumber:");
    Serial.println(mobileNumber);
    if(checkIfNumberAuthorized(mobileNumber) > 0) {
      // pick the call;
      digitalWrite(BUZZER,HIGH);
      delay(400);
      digitalWrite(BUZZER,LOW);
      gsm.answer();
      
    }
    else{
      Serial.println("Call from other number.\n");
      gsm.hangup(); // verify the (from jagadevan)

    }
  }
  else{
    Serial.println("Call got disconnected");
  }
  Serial.println(F("call handling done"));
}



void handleSMS(byte messageIndex) {
  // Get the SMS content
  int state = -1;
  char message[100];
  char mobileNumber[16];
  char newMobileNumber[16];
  char dateTime[24];
  char *s, *p;
  char num[4];
  byte i = 0;
  
  // read SMS content
  gsm.readSMS(messageIndex, message, MESSAGE_LENGTH, mobileNumber, dateTime);
  strupr(message);
  Serial.println(F("Printing SMS content"));
  Serial.println(message);

  // delete SMS to save memory
  gsm.deleteSMS(messageIndex);

  if(checkIfNumberAuthorized(mobileNumber) > 0) {
    
    if(NULL != ( s = strstr(message,"CAR_LOCATION"))) 
    {
        char sms_current_location_link[100]={0};
        gpsTimeout=millis();
        if (triggerGPS() == true)
        {
            sprintf(sms_current_location_link,"Current Location: http://maps.google.com/maps?q=%s,%s",gpsLAT,gpsLON);
            Serial.println(sms_current_location_link);
            lat_cloud=gpsLAT;
            lon_cloud=gpsLON;
            gsm.sendSMS(userNumber,sms_current_location_link);
            location_cloud=sms_current_location_link;
            send_to_cloud();
        }
        else
        {
            sprintf(sms_current_location_link,"Current Location: Updating ...\n(Please raise a request after sometime.)");
            Serial.println(sms_current_location_link);
            lat_cloud="NO";
            lon_cloud="NO";
            gsm.sendSMS(userNumber, sms_current_location_link);
            location_cloud=sms_current_location_link;
            send_to_cloud();
        }
     
    }
    else if(NULL != ( s = strstr(message,"CAR_RING"))) 
    {

      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      delay(800);
      digitalWrite(BUZZER,LOW);
      digitalWrite(RED_LED,LOW);
      delay(800);     
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      delay(800);
      digitalWrite(BUZZER,LOW);
      digitalWrite(RED_LED,LOW);
      delay(800);
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      delay(800);
      digitalWrite(BUZZER,LOW);
      digitalWrite(RED_LED,LOW);
      delay(800);
      digitalWrite(BUZZER,HIGH);
      digitalWrite(RED_LED,HIGH);
      delay(800);
      digitalWrite(BUZZER,LOW);
      digitalWrite(RED_LED,LOW);
              
    }
    else if(NULL != ( s = strstr(message,"CAR_STOP"))) 
    {
      stop_motor();
      antitheft_system = 3;   // stop car loop
      carmode_cloud="PARK";
      speed_cloud="NO";
      gsm.sendSMS(userNumber, "Alert: Anti-Theft System Activated.\nVehicle Stopped.");
    }
    else if(NULL != ( s = strstr(message,"CAR_START"))) 
    {
      antitheft_system = 2;   // stop car loop
      carmode_cloud="DRIVE";
      speed_cloud="NORMAL";
      gsm.sendSMS(userNumber, "Alert: Anti-Theft System De-Activated.\nVehicle Normal.");
    }

    else if(NULL != strstr(message,"SMS ENABLE")) 
    {
       
    }
    
  }
  else{
    Serial.println("Msg from other number");
  }
  Serial.println(F("SMS handling done"));
}

bool getNumberFromString(char *inComingString, char *mobileNumber) {
  byte i = 0;
  char *p, *p2, *s;
  
  // check if string is call string or sms string
  if(NULL != ( s = strstr(inComingString,"+CLIP:"))) {
    // inComingString is a call
    // Response is like:
    // RING
    // +CLIP: "+91XXXXXXXXXX",145,"",,"TT+91XXXXXXXXXX",0
    
    // Extract mobile number from the string
    p = strstr(s,"\"");
    p++; // we are on first character of phone number
    p2 = strstr(p,"\"");
    if (NULL != p) {
      i = 0;
      while(p < p2) {
        mobileNumber[i++] = *(p++);
      }
      mobileNumber[i] = '\0';
    }
#ifdef DEBUG    
    Serial.print(F("Mobile number obtained "));
    Serial.println(mobileNumber);
#endif
    return true;
  }
  return false;
}

int checkIfNumberAuthorized(char *mobileNumber) {
  if(strcmp(mobileNumber,userNumber) == 0){
    return 1;
  }
  else{
    return -1;
  } 
  
}

/*********************** GSM END **************************/

/********************* GPS START **************************/
bool triggerGPS()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while(1)
  {
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
      {
        if( displayInfo() == true )
        {
          return true;
        }
  
        if ((millis() - gpsTimeout) > 10000 )
        {
          return false;
        }
      }
  }
}


bool displayInfo()
{
  Serial.print("wevonix_getlatlong");
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    dtostrf(gps.location.lat(),3,6,gpsLAT);
    dtostrf(gps.location.lng(),3,6,gpsLON);
    Serial.print("wevo lat:");
    Serial.println(gpsLAT);
    Serial.print("wevo lon:");
    Serial.println(gpsLON);
    return true;
  }
  else
  {
    Serial.print(F("INVALID"));
    return false;
  }
}
/******************** GPS END *****************************/


/************************ RFID START **************************/

bool startScreen(void) 
{

  lcd.setCursor(13,3);
  lcd.print("Scan.  ");
  bool tempCardMenu =  false;
  if(readRfidCard() == RFID_FOUND) 
  {
     unsigned int tempCardType = checkCardType();
     if(tempCardType == DL_VALID ) {
        lcd.setCursor(13,3);
        lcd.print("DL VAL ");
        delay(5000);
        tempCardMenu = true;
        licence_cloud="DL";
        return true;
     }
     else if(tempCardType == DL_NOT_VALID ) {
        lcd.setCursor(13,3);
        lcd.print("DL INV ");
        delay(3000);
        licence_cloud="NO";
        tempCardMenu = false;
     }
     else if(tempCardType == LL_VALID ) {
        lcd.setCursor(13,3);
        lcd.print("LL VAL ");
        delay(5000);
        lcd.setCursor(13,3);
        lcd.print("Scan DL");
        if(readRfidCard() == RFID_FOUND) {
           unsigned int tempCardType = checkCardType();
           if(tempCardType == DL_VALID ) {
              lcd.setCursor(13,3);
              lcd.print("DL VAL "); 
              delay(2500);
              tempCardMenu = true;
              licence_cloud="LL";
              return true;
           }
           else {
              lcd.setCursor(13,3);
              lcd.print("INVALID");
              delay(3000);
              licence_cloud="NO";
              tempCardMenu = false;
          }
       }
       else if(readRfidCard() == RFID_NOT_FOUND) {
         tempCardMenu = false;   
       }
    }
     else if(tempCardType == LL_NOT_VALID ) {
        lcd.setCursor(13,3);
        lcd.print("LL INV ");
        delay(5000);
        licence_cloud="NO";
        tempCardMenu = false;
     }
     else if(tempCardType == INVALID_CARD ) {
        lcd.setCursor(13,3);
        lcd.print("INVALID");
        delay(5000);
        licence_cloud="NO";
        tempCardMenu = false;
     }

     if(tempCardMenu == false){
         tempCardMenu = false;
         while(1) {
           if(digitalRead(START_BUTTON) == BUTTON_PRESSED) {
              delay(300);
              licence_cloud="NO";
              return false;
            }
        }
     
     }
  }   
  else if(readRfidCard() == RFID_NOT_FOUND) {
     delay(200);
      while(1) {
           if(digitalRead(START_BUTTON) == BUTTON_PRESSED) {
              delay(300);
              licence_cloud="NO";
              return false;
            }
      } 
   }
}

bool readRfidCard(void) {
  unsigned long timeout = 0,timeout_1 = 0;
  timeout = millis();
  memset(cardNumber,'\0',15);
  while(true) {
    if (RC522.isCard())
    {
      RC522.readCardSerial();
      //Serial.println("Card detected:");
      char rfid_card[5];
      for(int i=0;i<5;i++)
      {
        card =  RC522.serNum[i];
        cardNum += card; 
      }
      cardNum.toCharArray(cardNumber,15);
      cardNum.remove(0);
      Serial.print("Card detected:");
      Serial.print(cardNumber);
      Serial.println("***");
      //delay(2000);
      return RFID_FOUND;
   }

   if ((millis() - timeout) > TIME_OUT) {
        lcd.setCursor(13,3);
        lcd.print("NO CARD");
     
     delay(2000);
     return RFID_NOT_FOUND;
   }
  }
}

unsigned int checkCardType(void) {

  if(strcmp(cardNumber,dlValid)         == CARD_MATCHED) {
    return DL_VALID;
  }
  else if(strcmp(cardNumber,dlNotValid) == CARD_MATCHED) {
    return DL_NOT_VALID;
  }
  else if(strcmp(cardNumber,llValid)    == CARD_MATCHED) {
    return LL_VALID;
  }
  else if(strcmp(cardNumber,llNotValid) == CARD_MATCHED) {
    return LL_NOT_VALID;
  }
  else
    return INVALID_CARD;
}

/************************** RFID END *************************/

/************** LCD SCREEN START *************/

void screen1()
{
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("   TEST       STATUS");
   lcd.setCursor(0,1);
   lcd.print("1] ALCOHOL  : ---   ");
   lcd.setCursor(0,2);
   lcd.print("2] SEAT-BELT: ---   ");
   lcd.setCursor(0,3);
   lcd.print("3] LICENCE  : ---   ");
}


/************* LCD SCREEN END ****************/

/************************************************ LCD character by character disp on input of string *******************************************/

void charbychar_disp(char *s)
{
  char input;
  while(*s != '\0')
  {
    input=*s;
    lcd.print(input);
    delay(150);
    *s++;
  } 
  return;
}

/***************************************** LCD END ***********************************************************************************************/

/*********************** RUN MOTOR START ************************/

void low_speed_run()
{
   analogWrite(RIGHT_MOTOR_EN1,80);
   digitalWrite(RIGHT_MOTOR_IN1,HIGH);
   digitalWrite(RIGHT_MOTOR_IN2,LOW);
   analogWrite(LEFT_MOTOR_EN2,80);
   digitalWrite(LEFT_MOTOR_IN3,LOW);
   digitalWrite(LEFT_MOTOR_IN4,HIGH);
}

void high_speed_run()
{
   analogWrite(RIGHT_MOTOR_EN1,255);
   digitalWrite(RIGHT_MOTOR_IN1,HIGH);
   digitalWrite(RIGHT_MOTOR_IN2,LOW);
   analogWrite(LEFT_MOTOR_EN2,255);
   digitalWrite(LEFT_MOTOR_IN3,LOW);
   digitalWrite(LEFT_MOTOR_IN4,HIGH);
}

void stop_motor()
{
   
   digitalWrite(RIGHT_MOTOR_IN1,LOW);
   digitalWrite(RIGHT_MOTOR_IN2,LOW);

   digitalWrite(LEFT_MOTOR_IN3,LOW);
   digitalWrite(LEFT_MOTOR_IN4,LOW);
}

/********************** RUN MOTOR END ***************************/

/************** heartbeat start *****************************/

void heart_beat_check()
{
  if(initial_beat == true )
  {
    if(first_beat == true)
    {
      first_beat_time= millis();
      Serial.print("First_beat_time: ");
      Serial.println(first_beat_time);
      initial_beat = false;
      first_beat = false;
      third_beat = false;
    }
    if(third_beat == true)
    {
      third_beat_time= millis();
      Serial.print("Third_beat_time: ");
      Serial.println(third_beat_time);
      
      difference_beat_time= third_beat_time - first_beat_time;
      Serial.print("difference_beat_time: ");
      Serial.println(difference_beat_time);
      Serial.println(" ");
      
      initial_beat = true;
      first_beat = true;
      third_beat = false;

      /**********beat count start ***********/
        if( ((millis() - third_beat_time) < 5000UL) && (difference_beat_time >= 600 && difference_beat_time <= 1000) )
        {
          beat_count = (60000 / difference_beat_time);
          Serial.print("BPM: ");
          Serial.println(beat_count);
          Serial.println(" ");
        }
        else
        { 
          Serial.print("BPM: ");
          Serial.println("NO");
          Serial.println(" ");
        }
      /**********beat count end ***********/

    }
  }
  else
  {
    second_beat_time= millis();
    Serial.print("Second_beat_time: ");
    Serial.println(second_beat_time);
    initial_beat = true;
    first_beat = false;
    third_beat = true;  
  }
}

void heart_beat_count()
{

          Serial.print("BPM: ");
          Serial.println(beat_count);
          Serial.println(" ");
          lcd.setCursor(13,3);    //(col,row)
          lcd.print("     ");
          lcd.setCursor(13,3);    //(col,row)
          lcd.print(beat_count);
          lcd.setCursor(18,3);    //(col,row)
          lcd.print("  ");
          itoa(beat_count,heart_cloud,10);  

}


/************** heatbeat end *****************************/

bool seat_belt_judgement()
{
  if(digitalRead(SEAT_BELT) == 0)
  {
    delay(500);
    seat_belt_count = !seat_belt_count;
    Serial.print("seat_belt_count: ");
    Serial.println(seat_belt_count);
  }
  if(seat_belt_count == true)
  {
    seatbelt_cloud="PLUGGED";
    return true;
  }
  else if(seat_belt_count == false)
  {
    seatbelt_cloud="UNPLUGGED";
    return false;
  }
}

/******************** send and recieve from/to cloud start *****************/

void send_to_cloud()
{
  if((millis() - sendingTimer) > 2000UL)
  {

    sendingTimer = millis();
    Serial2.println("*start*");
    Serial2.print("*alcohol:");
    Serial2.print(alcohol_cloud);
    //Serial2.print("24.20");
    Serial2.println("*");
    Serial2.print("*speed:");
    Serial2.print(speed_cloud);
    //Serial2.print("60.20");
    Serial2.println("*");
    Serial2.print("*seatbelt:");
    Serial2.print(seatbelt_cloud);
    //Serial2.print("yes");
    Serial2.println("*");
    Serial2.print("*carmode:");
    Serial2.print(carmode_cloud);
    //Serial2.print("yes");
    Serial2.println("*");
    Serial2.print("*lat:");
    Serial2.print(lat_cloud);
    //Serial2.print("yes");
    Serial2.println("*");
    Serial2.print("*lon:");
    Serial2.print(lon_cloud);
    //Serial2.print("yes");
    Serial2.println("*");
    Serial2.print("*licence:");
    Serial2.print(licence_cloud);
    //Serial2.print("yes");
    Serial2.println("*");
    Serial2.print("*heart:");
    Serial2.print(heart_cloud);
    Serial2.println("*");
    Serial2.print("*location:");
    Serial2.print(location_cloud);
    Serial2.println("*");
    Serial2.println("*end*");    
  }
}

void recivedata(char  *buffer1,int count)
{
  int i = 0;
  unsigned long timerStart, prevChar;
  timerStart = millis();
  prevChar = 0;
  Serial.println("inside recieve data");
  while(1)
  {
    while (Serial2.available()) 
    {
      char c = Serial2.read();
      prevChar = millis();
      buffer1[i++] = c;
      if(i >= count)break;
    }
    if(i >= count)break;
    if ((unsigned long) (millis() - timerStart) > TOTAL_TIMEOUT * 1000UL) 
    {
      break;
    }
        //If interchar Timeout => return FALSE. So we can return sooner from this function. Not DO it if we dont recieve at least one char (prevChar <> 0)
    if (((unsigned long) (millis() - prevChar) > CHAR_TIMEOUT * 1000UL) && (prevChar != 0)) 
    {
      break;
    }
   }
  buffer1[i]='\0';  
}

bool phraseSerialdata(char  *buffer1) 
{
  char *st=NULL,*en=NULL,*te=NULL,*ed=NULL;
  unsigned int i=0;
  if((NULL != ( st = strstr(buffer1,"*start*"))) && (NULL != ( en = strstr(buffer1,"*end*"))))
  {
  
      if(NULL !=(te = strstr(st,"robo:")))
      {
        if(NULL !=(ed = strstr(te,"*")))
        {
          te = te + 5;
          memset(robot_voice_command,'\0',sizeof(robot_voice_command));
          i=0;
          while(te != ed)
          {
            robot_voice_command[i++] = *(te++);
          }
          robot_voice_command[i]='\0';
           //Firebase.setString("temperature", temperatureValue);
          Serial.print("robo : ");
          Serial.println(robot_voice_command);
        }
      }
      else
      {
        Serial.println("in robo false");
        return false;
      }
      
      return true;
  }
  else
  {
    Serial.println("in  false");
    return false;
  }
}


/******************** send and recieve from/to cloud end *****************/



