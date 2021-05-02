/*
 *  Arduino Temperature Data Logging
 *  
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 */
#include <LiquidCrystal_I2C.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"

#include <Arduino.h>
#include <SoftwareSerial.h>

#define RX        2    //Serial Receive pin
#define TX        3    //Serial Transmit pin
#define DERE_pin    4    //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW

// Anemometer Parameter
#define DEFAULT_DEVICE_ADDRESS  0x02
#define WIND_SPEED_REG_ADDR 0x002A
#define SLAVE_ADDRESS_REG_ADDR_H 0x20
#define SLAVE_ADDRESS_REG_ADDR_L 0x00
#define READ_HOLDING_REG  0x03      //Function code 3 
#define WRITE_SINGLE_REG  0x06      //Function code 3 

File myFile;
RTC_DS3231 rtc;
LiquidCrystal_I2C lcd(0x27,16,2); // Display  I2C 16 x 2
SoftwareSerial RS485Serial(RX, TX);

int pinCS = 10; // Pin 10 on Arduino Uno
int Interval =5000;
String kataawal= "T";
String katadua= "J";
String ketiga= ".txt";

int P1 = 7;
int P2 = 8;
int P3 = 6;

int hourupg;
int minupg;
int yearupg;
int monthupg;
int dayupg;
int menu =0;

unsigned int calculateCRC(unsigned char * frame, unsigned char bufferSize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order. 
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp; 
}

//-------------WIND SPEED---------------//
float getWindSpeed(byte address){
  float windSpeed = 0.0f;
  byte Anemometer_buf[8];
  byte Anemometer_request[] = {address, READ_HOLDING_REG, 0x00, WIND_SPEED_REG_ADDR, 0x00, 0x01, 0x00, 0x00}; // Request wind Speed Frame Data

  unsigned int crc16 = calculateCRC(Anemometer_request, sizeof(Anemometer_request) - 2);  
  Anemometer_request[sizeof(Anemometer_request) - 2] = crc16 >> 8; // split crc into 2 bytes
  Anemometer_request[sizeof(Anemometer_request) - 1] = crc16 & 0xFF;
  
  digitalWrite(DERE_pin, RS485Transmit);     // init Transmit
  RS485Serial.write(Anemometer_request, sizeof(Anemometer_request));
  RS485Serial.flush();

#ifdef PRINT_FRAME_DATA
  Serial.print("Send Request Frame data: "); 
  for(uint8_t i=0; i<sizeof(Anemometer_request); i++){
    Serial.print("0x");
    Serial.print(Anemometer_request[i], HEX); 
    Serial.print(" "); 
  }
  Serial.println();
#endif

  digitalWrite(DERE_pin, RS485Receive);      // Init Receive
  RS485Serial.readBytes(Anemometer_buf, 7);

#ifdef PRINT_FRAME_DATA
  Serial.print("Received Frame data: "); 
  for(uint8_t i=0; i<7; i++){
    Serial.print("0x");
    Serial.print(Anemometer_buf[i], HEX); 
    Serial.print(" "); 
  }
  Serial.println();
#endif

  byte dataH = Anemometer_buf[3];
  byte dataL = Anemometer_buf[4];
  windSpeed = ((dataH << 8) | dataL ) / 100.0;  //Range data 0~3000 = 0~30m/s

  return windSpeed;
  
}
//-------------WIND SPEED---------------//

int setSlaveAddress(byte current_address, unsigned int new_address){
  unsigned int new_addrH, new_addrL;
  byte Anemometer_buf[8];
  byte Anemometer_request[] = {current_address, WRITE_SINGLE_REG, SLAVE_ADDRESS_REG_ADDR_H, SLAVE_ADDRESS_REG_ADDR_L, 0x00, 0x00, 0x00, 0x00}; // Request Change Slave Addr

  //split address into 2 byte
  new_addrH = (new_address >> 8) & 0xFF;
  new_addrL = new_address & 0xFF;
  Anemometer_request[4] = new_addrH;
  Anemometer_request[5] = new_addrL;

  // calculate CRC16
  unsigned int crc16 = calculateCRC(Anemometer_request, sizeof(Anemometer_request) - 2);  
  Anemometer_request[sizeof(Anemometer_request) - 2] = crc16 >> 8; // split crc into 2 bytes
  Anemometer_request[sizeof(Anemometer_request) - 1] = crc16 & 0xFF;  

  digitalWrite(DERE_pin, RS485Transmit);     // init Transmit
  RS485Serial.write(Anemometer_request, sizeof(Anemometer_request));
  RS485Serial.flush();

#ifdef PRINT_FRAME_DATA
  Serial.print("Send Request Change addr Frame data: "); 
  for(uint8_t i=0; i<sizeof(Anemometer_request); i++){
    Serial.print("0x");
    Serial.print(Anemometer_request[i], HEX); 
    Serial.print(" "); 
  }
  Serial.println();
#endif

  digitalWrite(DERE_pin, RS485Receive);      // Init Receive
  RS485Serial.readBytes(Anemometer_buf, 8);

#ifdef PRINT_FRAME_DATA
  Serial.print("Received Frame data: "); 
  for(uint8_t i=0; i<8; i++){
    Serial.print("0x");
    Serial.print(Anemometer_buf[i], HEX); 
    Serial.print(" "); 
  }
  Serial.println();
#endif

  // calculate Received CRC16
  unsigned int crc16_received = ((Anemometer_buf[sizeof(Anemometer_buf) - 2] << 8) | Anemometer_buf[sizeof(Anemometer_buf) - 1] ); 
  unsigned int crc16_calculated = calculateCRC(Anemometer_buf, sizeof(Anemometer_buf) - 2); 
 
#ifdef PRINT_FRAME_DATA
  Serial.print("Received CRC data: "); 
  Serial.print("crc16: 0x"); Serial.println(crc16_received, HEX);
  Serial.print("Calculated CRC data: "); 
  Serial.print("crc16: 0x"); Serial.println(crc16_calculated, HEX);
#endif

  // verifikasi apakah data valid dengan cara melakukan kalkulasi CRC
  if(crc16_received == crc16_calculated){
    byte dataH = Anemometer_buf[4];
    byte dataL = Anemometer_buf[5];
    int address = (dataH << 8) | dataL ;
    return address;
  }
  else{
    return -1;
  }
  
}

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  pinMode(P1,INPUT_PULLUP);
  pinMode(P2,INPUT_PULLUP);
  pinMode(P3,INPUT_PULLUP);
  pinMode(DERE_pin, OUTPUT);
  
  RS485Serial.begin(9600);   
  delay(1000);  
  Serial.begin(9600);
  pinMode(pinCS, OUTPUT);
  
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
     lcd.setCursor(0, 1);
     lcd.print("--SD CARD SIAP--");
     lcd.clear();

  } else
  {
    Serial.println("SD card initialization failed");
    lcd.setCursor(0, 1);
    lcd.print("McSD BELUM SIAP");
    return;
  }
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    lcd.setCursor(0, 1);
    lcd.print("-RTC BELOM SIAP-");
    Serial.flush();
    abort();
  }
   // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
     //rtc.adjust(DateTime(2021, 03, 30, 21, 33, 20));
   //DisplayDateTime();
  
}
void loop() {
    int buttonA = digitalRead(P1);
    if(buttonA == LOW)
  {
   menu=menu+1;
  }
// in which subroutine should we go?
  if (menu==0)
    {
      siaplog();
      DisplayDateTime();
    }
  if (menu==1)
    {
    DisplaySetHour();
    }
  if (menu==2)
    {
    DisplaySetMinute();
    }
  if (menu==3)
    {
    DisplaySetYear();
    }
  if (menu==4)
    {
    DisplaySetMonth();
    }
  if (menu==5)
    {
    DisplaySetDay();
    }
  if (menu==6)
    {
    StoreAgg(); 
    delay(500);
    menu=0;
    }
    delay(100);
    
  //lcd.clear();
}

void siaplog()
{
    DateTime now = rtc.now();
    int juga=now.month();
    int tambah=now.day();
    int lagi=now.hour();
    String bisa= juga + kataawal + tambah + katadua + lagi + ketiga;
    Serial.println(bisa);
  myFile = SD.open(bisa, FILE_WRITE);
  if (myFile) {
    lcd.setCursor(0, 1);
      lcd.print(bisa);
      lcd.setCursor(12, 1);
      lcd.print(getWindSpeed(2));
      lcd.setCursor(0, 0);
    myFile.print(now.year(), DEC);
    myFile.print("/");
    myFile.print(now.month(), DEC);
    myFile.print("/");
    myFile.print(now.day(), DEC);
    myFile.print("/ ");
    myFile.print(now.hour(), DEC);
    myFile.print(":");
    myFile.print(now.minute(), DEC);
    myFile.print(":");
    myFile.print(now.second(), DEC);
    myFile.print(",");         
    myFile.print(getWindSpeed(2));
    myFile.println();
    myFile.close();
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("file error boy");
  }
  delay(Interval);
 }

void DisplayDateTime()
{
// We show the current date and time
  DateTime now = rtc.now();
  //lcd.print("");
  lcd.setCursor(0, 0);
  if (now.hour()<=9)
  {
    lcd.print("0");
  }
  lcd.print(now.hour(), DEC);
  //hourupg=now.hour();
  lcd.print(":");
  if (now.minute()<=9)
  {
    lcd.print("0");
  }
  lcd.print(now.minute(), DEC);
  
  minupg=now.minute();
  lcd.setCursor(5, 0);
  lcd.print(" ");
  if (now.day()<=9)
  {
    lcd.print("0");
  }
  lcd.print(now.day(), DEC);
  dayupg=now.day();
  lcd.print("-");
  if (now.month()<=9)
  {
    lcd.print("0");
  }
  lcd.print(now.month(), DEC);
  monthupg=now.month();
  lcd.print("-");
  lcd.print(now.year(), DEC);
  yearupg=now.year();
}

void DisplaySetHour()
{
// time setting
  lcd.clear();
  DateTime now = rtc.now();
  if(digitalRead(P2)==LOW)
  {
    if(hourupg==23)
    {
      hourupg=0;
    }
    else
    {
      hourupg=hourupg+1;
    }
  }
   if(digitalRead(P3)==LOW)
  {
    if(hourupg==0)
    {
      hourupg=23;
    }
    else
    {
      hourupg=hourupg-1;
    }
  }
  lcd.setCursor(0,0);
  lcd.print("Set time:");
  lcd.setCursor(0,1);
  lcd.print(hourupg,DEC);
  delay(200);
}

void DisplaySetMinute()
{
// Setting the minutes
  lcd.clear();
  if(digitalRead(P2)==LOW)
  {
    if (minupg==59)
    {
      minupg=0;
    }
    else
    {
      minupg=minupg+1;
    }
  }
   if(digitalRead(P3)==LOW)
  {
    if (minupg==0)
    {
      minupg=59;
    }
    else
    {
      minupg=minupg-1;
    }
  }
  lcd.setCursor(0,0);
  lcd.print("Set Minutes:");
  lcd.setCursor(0,1);
  lcd.print(minupg,DEC);
  delay(200);
}
  
void DisplaySetYear()
{
// setting the year
  lcd.clear();
  if(digitalRead(P2)==LOW)
  {    
    yearupg=yearupg+1;
  }
   if(digitalRead(P3)==LOW)
  {
    yearupg=yearupg-1;
  }
  lcd.setCursor(0,0);
  lcd.print("Set Year:");
  lcd.setCursor(0,1);
  lcd.print(yearupg,DEC);
  delay(200);
}

void DisplaySetMonth()
{
// Setting the month
  lcd.clear();
  if(digitalRead(P2)==LOW)
  {
    if (monthupg==12)
    {
      monthupg=1;
    }
    else
    {
      monthupg=monthupg+1;
    }
  }
   if(digitalRead(P3)==LOW)
  {
    if (monthupg==1)
    {
      monthupg=12;
    }
    else
    {
      monthupg=monthupg-1;
    }
  }
  lcd.setCursor(0,0);
  lcd.print("Set Month:");
  lcd.setCursor(0,1);
  lcd.print(monthupg,DEC);
  delay(200);
}

void DisplaySetDay()
{
// Setting the day
  lcd.clear();
  if(digitalRead(P2)==LOW)
  {
    if (dayupg==31)
    {
      dayupg=1;
    }
    else
    {
      dayupg=dayupg+1;
    }
  }
   if(digitalRead(P3)==LOW)
  {
    if (dayupg==1)
    {
      dayupg=31;
    }
    else
    {
      dayupg=dayupg-1;
    }
  }
  lcd.setCursor(0,0);
  lcd.print("Set Day:");
  lcd.setCursor(0,1);
  lcd.print(dayupg,DEC);
  delay(200);
}

void StoreAgg()
{
// Variable saving
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("SAVING IN");
  lcd.setCursor(0,1);
  lcd.print("PROGRESS");
  rtc.adjust(DateTime(yearupg,monthupg,dayupg,hourupg,minupg,0));
  delay(200);
}
