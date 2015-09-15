/*
  This code is been tested in an Arduino Uno clone wiring a comercial infrared
  sensor in pin 2. it will print to to the serial just when it detects a change 
  in Total Imports, Total exports or a change in direction (0=Importing , 1=Exporting)
  
  I have tried some IR sensors so far the only one working at the moment is RPM7138-R
  
  Based on Dave's code to read an elter a100c for more info on that vist: 
  http://www.rotwang.co.uk/projects/meter.html
  Thanks Dave.
*/



#include <Wire.h>
#include "RTClib.h"
#include <SoftwareSerial.h>
#include <stdlib.h>

RTC_Millis rtc;

const uint8_t intPin = 2;
#define BIT_PERIOD 860 // us
#define BUFF_SIZE 64
unsigned int statusFlag;
float imports=0;
float exports=0;
float transmittedImports=0;
float transmittedExports=0;
uint8_t transmittedsFlag=0;
float last_data;
uint8_t sFlag;
float imps;
float exps;
uint16_t idx=0;
uint8_t byt_msg = 0;
uint8_t bit_left = 0;
uint8_t bit_shft = 0;
uint8_t pSum = 0;
uint16_t BCC = 0;
uint8_t eom = 1;

// connect 10 to TX of Serial USB
// connect 11 to RX of serial USB
SoftwareSerial ser(8, 9); // RX, TX

// replace with your channel's thingspeak API key
String apiKey = "G9GVCHD9LTQY6ZPO";

volatile long data[BUFF_SIZE];
volatile uint8_t in;
volatile uint8_t out;
volatile unsigned long last_us;
uint8_t dbug = 0;

void setup() {
    //pinMode(intPin, INPUT);
    in = out = 0; 
    // enable software serial
    Serial.begin(9600);
    // enable Wifi serial
    ser.begin(9600);
    // reset ESP8266
    ser.println("AT+RST");
    Serial.println("8266 wifi initialised");
    EICRA |= 3;    //RISING interrupt
    EIMSK |= 1;    
    if (dbug) Serial.print("Start ........\r\n");
    last_us = micros();

    // following line sets the RTC to the date & time this sketch was compiled
    rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}

void loop() {
//    decode_buff();
 DateTime now = rtc.now();
  int rd = decode_buff();
     
  if (!rd) return;
  if (rd==3) 
  {
    //anonoymous readings are detected, do not know why
    //need to eliminate and tidy up code
    if (imports>=0 && exports>=0)
    {
       if (imports>exports)
       { 
          Serial.println("Transmitted Data Recorded");
          Serial.print(transmittedImports);    Serial.print(",");
          Serial.print(transmittedExports);    Serial.print(",");
          Serial.print(transmittedsFlag); Serial.println("");
          Serial.print(now.day(), DEC); 
          Serial.print('/');
          Serial.print(now.month(), DEC);
          Serial.print('/');
          Serial.print(now.year(), DEC);
          Serial.print(",");
          Serial.print(now.hour(), DEC);
          Serial.print(':');
          Serial.print(now.minute(), DEC);
          Serial.print(':');
          Serial.print(now.second(), DEC);
          Serial.print(',');
          Serial.print(imports);    Serial.print(",");
          Serial.print(exports);    Serial.print(",");
          Serial.print(statusFlag); Serial.println("");

          //only transmit if the status Flag is a 1 or 0, if not do not transmit
          //and check further.
          if (statusFlag==1 || statusFlag==0)
          {
            //if transmitted imports and transmitted exports are blank no history 
            //transmit data and then set transmitted prefix variables data to 
            //data transmitted.
            if (transmittedImports==0 && transmittedExports==0 && transmittedsFlag==0)
            {
                send8266Data(imports,exports,statusFlag);
                transmittedImports=imports;
                transmittedExports=exports;
                transmittedsFlag=statusFlag;
            }
            //check to see if imports and exports are within range of maximum values.
            else if (imports<=(transmittedImports+2) && exports<=(transmittedExports+2))
            {
               //check to see if data is greater than historic data saved
               if (exports>=transmittedExports && statusFlag==1)
               {
                  send8266Data(imports,exports,statusFlag);          
                  transmittedImports=imports;
                  transmittedExports=exports;
                  transmittedsFlag=statusFlag;         
               }
               else if (imports>=transmittedImports && statusFlag==0)
               {
                  send8266Data(imports,exports,statusFlag);          
                  transmittedImports=imports;
                  transmittedExports=exports;
                  transmittedsFlag=statusFlag;
               }
               else
               {
                  Serial.println("Above IR received results not transmitted, erratic and invalid!!!");
               }
            }
            else
            {
              Serial.println("Above IR received results not transmitted, erratic and invalid!!!");
            }
          }
          else
          {
            Serial.println("Above IR received results not transmitted, erratic and invalid!!!");
          }
         }
    }   
  }
//   unsigned long end_time = millis() + 6000;
//   while (end_time >= millis()) ;
}

static int decode_buff(void) 
{
   if (in == out) return 0;
   int next = out + 1;
   if (next >= BUFF_SIZE) next = 0;
   int p = (((data[out]) + (BIT_PERIOD/2)) / BIT_PERIOD);
   if (dbug) { Serial.print(data[out]); Serial.print(" "); if (p>500) Serial.println("<-"); }   
   if (p>500) 
   {
     idx = BCC = eom = imps = exps = sFlag = 0;   
     out = next;
     return 0;
   }
   bit_left = (4 - (pSum % 5));
   bit_shft = (bit_left<p)? bit_left : p;
   pSum = (pSum==10)? p : ((pSum+p>10)? 10: pSum + p);
   if (eom==2 && pSum>=7)
   {
      pSum=pSum==7?11:10;
      eom=0;   
   }

   if (bit_shft>0) 
   {
      byt_msg >>= bit_shft;
      if (p==2) byt_msg += 0x40<<(p-bit_shft);
      if (p==3) byt_msg += 0x60<<(p-bit_shft);
      if (p==4) byt_msg += 0x70<<(p-bit_shft);   
      if (p>=5) byt_msg += 0xF0;
    }
//    Serial.print(p); Serial.print(" ");Serial.print(pSum);Serial.print(" ");    
//    Serial.print(bit_left);Serial.print(" ");Serial.print(bit_shft);Serial.print(" ");    
//    Serial.println(byt_msg,BIN);
    if (pSum >= 10) {
       idx++;
       if (idx!=328) BCC=(BCC+byt_msg)&255;
//       if (dbug){Serial.print("[");Serial.print(idx);Serial.print(":");Serial.print(byt_msg,HEX); Serial.print("]");}
       if (idx>=95 && idx<=101)  
          imps += ((float)byt_msg-48) * pow(10 , (101 - idx));
       if (idx==103) 
          imps += ((float)byt_msg-48) / 10;
       if (idx>=114 && idx<=120) 
          exps += ((float)byt_msg-48) * pow(10 , (120-idx));
       if (idx==122) 
          exps += ((float)byt_msg-48) / 10;
       if (idx==210) 
          sFlag = (byt_msg-48)>>3; //1=Exporting ; 0=Importing
       if (byt_msg == 3) eom=2; 
       if (idx==328) {
          if ((byt_msg>>(pSum==10?1:2))==((~BCC)&0x7F)) {
             if (last_data != (imps + exps + sFlag)) {
                imports=imps;
                exports=exps;
                statusFlag=sFlag;
                last_data = imps + exps + sFlag;
                out = next;
                return 3;
             }
          }
          if (dbug) {
             Serial.println(""); Serial.print("---->>>>");
             Serial.print(imps); Serial.print("\t");
             Serial.print(exps); Serial.print("\t");
             Serial.print(sFlag); Serial.print("\t"); 
             Serial.print(pSum); Serial.print("\t");              
             Serial.print(byt_msg>>(pSum==10?1:2),BIN); Serial.print("\t"); //BCC read
             Serial.print((~BCC)&0x7F,BIN); Serial.print("\t"); //BCC calculated
          }
       }  
    }
    out = next;
    return 0;
}

ISR(INT0_vect) {
   unsigned long us = micros();
   unsigned long diff = us - last_us;
   if (diff > 20 ) {
      last_us = us;
      int next = in + 1;
      if (next >= BUFF_SIZE) next = 0;
      data[in] = diff;
      in = next;
   }
}

static void send8266Data(float importData,float exportData, unsigned int statusFlag) 
{
   //Function to send data to thingspeak via 8266 with AT commands  
   // TCP connection
  String cmd = "AT+CIPSTART=\"TCP\",\"";
 
  cmd += "144.212.80.11"; // api.thingspeak.com
  cmd += "\",80";
  Serial.println(cmd);
  ser.println(cmd);
   
  if(ser.find("Error")){
    Serial.println("AT+CIPSTART error");
   return;
  }

  // prepare GET string
  String getStr = "GET /update?api_key=";
  getStr += apiKey;
  getStr +="&field1=";
  getStr +=importData;
  getStr +="&field2=";
  getStr +=exportData;
  getStr +="&field3=";
  getStr +=statusFlag;
  getStr += "\r\n\r\n";
  Serial.print(getStr); Serial.println("");
  // send data length
  if (getStr.length()==79)
  {
     cmd = "AT+CIPSEND=";
     cmd += String(getStr.length());
     Serial.print(cmd); Serial.println("");
     ser.println(cmd);
     if(ser.find(">"))
     {
        ser.print(getStr);
     }
     else
     {
        ser.println("AT+CIPCLOSE");
        // alert user
        Serial.println("AT+CIPCLOSE");
       // thingspeak needs 15 sec delay between updates
       delay(16000); 
     }   
   }
   else
   {
      Serial.println("Did not send to thingspeak due to error in data");
   }  
}

