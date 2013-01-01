/*
This sketch only works on the MEGA
 
 This sketch will auto detect and map to 1000 - 2000:
 
 DSMx and DSM2 serial
 
 SBUS serial (provided the output is sent through an inverter such as the 74HC14)
 
 Standard RC signal
 
 Provided that the servo travel extents are left to the defualts and no trim is being used.
 
 It will work with different extents and trims, but the mapping will change slightly
 
 Spektrum is set to use Serial2 SBUS set to use Serial1
 
 
 Copyright (C) 2013  Michael Baker
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#define DSM2 0
#define DSMX 1
#define SBUS 2
#define RC 3
#define HEX_ZERO 0x00

long timer;
uint8_t rcType,readState,inByte;
boolean detected = false;
boolean newData = false;


const uint8_t syncArray1[14] = {
  1,0,3,2,5,4,7,6,9,8,11,10,13,12};
const uint8_t syncArray2[14] = {
  1,0,3,2,7,6,5,4,11,10,13,12,9,8};

int bufferIndex=0;

typedef union{
  struct{
    uint16_t aileron;//A8
    uint16_t aux1;//A9
    uint16_t elevator;//A10
    uint16_t gear;//A11
    uint16_t rudder;//A12
    uint16_t aux2;//A13
    uint16_t throttle;//A14
    uint16_t aux3;//A15 only for futaba or standard RC
  }
  values;
  byte buffer[16];
  uint16_t standardRCBuffer[8];
}
RadioControl_t;


RadioControl_t rcCommands;

uint8_t currentPinState = 0;
uint8_t previousPinState = 0;
uint8_t changeMask = 0;
uint8_t lastPinState = 0;
uint16_t currentTime = 0;
uint16_t timeDifference = 0;
uint16_t changeTime[8];
int16_t offset = 0;

uint8_t sBusData[25];




void setup(){
  Serial.begin(115200);
  Serial.println("begin");
  DetectRC();
  if (rcType == RC){
    DDRK = 0;//PORTK as input
    PORTK |= 0xFF;//turn on pull ups
    PCMSK2 |= 0xFF;//set interrupt mask for all of PORTK
    PCICR = 1<<2;//enable the pin change interrupt for K
    Center();
  }
}

void loop(){
  if (rcType != RC){
    FeedLine();
  }
  if (newData == true){
    newData = false;
    Serial.print(rcCommands.values.aileron);
    Serial.print(",");
    Serial.print(rcCommands.values.elevator);
    Serial.print(",");
    Serial.print(rcCommands.values.throttle);
    Serial.print(",");
    Serial.print(rcCommands.values.rudder);
    Serial.print(",");
    Serial.print(rcCommands.values.gear);
    Serial.print(",");
    Serial.print(rcCommands.values.aux1);
    Serial.print(",");
    Serial.print(rcCommands.values.aux2);
    Serial.print(",");
    Serial.println(rcCommands.values.aux3);


  }
}

void Center(){

  while (newData == false){
    delay(1);
  }

  offset = rcCommands.values.aileron - 1500;
}

ISR(PCINT2_vect){
  currentPinState = PINK;
  changeMask = currentPinState ^ lastPinState;
  //sei();
  lastPinState = currentPinState;
  currentTime = micros();
  for(uint8_t i=0;i<8;i++){
    if(changeMask & 1<<i){//has there been a change
      if(!(currentPinState & 1<<i)){//is the pin in question logic low?
        timeDifference = currentTime - changeTime[i];//if so then calculate the pulse width
        if (900 < timeDifference && timeDifference < 2200){//check to see if it is a valid length
          //rcCommands.standardRCBuffer[i] = timeDifference;
          rcCommands.standardRCBuffer[i] = (constrain((timeDifference - offset),1080,1920) - 1080) * 1.19 + 1000;
          newData = true;
        }
      }
      else{//the pin is logic high implying that this is the start of the pulse
        changeTime[i] = currentTime;
      }
    }
  }
}

void FeedLine(){
  switch(rcType){
  case 0:
    DSM2Parser();
    break;
  case 1:
    DSMXParser();
    break;
  case 2:
    SBusParser();
    break;
  }
}

void SBusParser(){
  if (Serial1.available() > 24){
    while(Serial1.available() > 0){
      inByte = Serial1.read();
      switch (readState){
      case 0:
        if (inByte != 0x0f){
          while(Serial1.available() > 0){//read the contents of in buffer this should resync the transmission
            inByte = Serial1.read();
          }
          return;
        }
        else{
          bufferIndex = 0;
          sBusData[bufferIndex] = inByte;
          sBusData[24] = 0xff;
          readState = 1;
        }
        break;
      case 1:
        bufferIndex ++;
        sBusData[bufferIndex] = inByte;
        if (bufferIndex < 24 && Serial1.available() == 0){
          readState = 0;
        }
        if (bufferIndex == 24){
          readState = 0;
          if (sBusData[0]==0x0f && sBusData[24] == 0x00){
            newData = true;
          }
        }
        break;
      }
    }
  }  

  if (newData == true){
    //credit to the folks at multiWii for this sBus parsing algorithm
    rcCommands.values.aileron  = constrain(((sBusData[1]|sBusData[2]<< 8) & 0x07FF),352,1695) ;
    rcCommands.values.aileron  = (rcCommands.values.aileron  - 352) * 0.7446 + 1000;
    rcCommands.values.elevator  = constrain(((sBusData[2]>>3|sBusData[3]<<5) & 0x07FF),352,1695);
    rcCommands.values.elevator  = (rcCommands.values.elevator  - 352) * 0.7446 + 1000;
    rcCommands.values.throttle  = constrain(((sBusData[3]>>6|sBusData[4]<<2|sBusData[5]<<10) & 0x07FF),352,1695);
    rcCommands.values.throttle  = (rcCommands.values.throttle  - 352) * 0.7446 + 1000;
    rcCommands.values.rudder  = constrain(((sBusData[5]>>1|sBusData[6]<<7) & 0x07FF),352,1695);
    rcCommands.values.rudder  = (rcCommands.values.rudder  - 352) * 0.7446 + 1000;
    rcCommands.values.gear = constrain(((sBusData[6]>>4|sBusData[7]<<4) & 0x07FF),352,1695);
    rcCommands.values.gear  = (rcCommands.values.gear  - 352) * 0.7446 + 1000;
    rcCommands.values.aux1 = constrain(((sBusData[7]>>7|sBusData[8]<<1|sBusData[9]<<9) & 0x07FF),352,1695);
    rcCommands.values.aux1  = (rcCommands.values.aux1  - 352) * 0.7446 + 1000;
    rcCommands.values.aux2  = constrain(((sBusData[9]>>2|sBusData[10]<<6) & 0x07FF),352,1695);
    rcCommands.values.aux2  = (rcCommands.values.aux2  - 352) * 0.7446 + 1000;
    rcCommands.values.aux3  = constrain(((sBusData[10]>>5|sBusData[11]<<3) & 0x07FF),352,1695);
    rcCommands.values.aux3  = (rcCommands.values.aux3  - 352) * 0.7446 + 1000;
  }

}

void DSMXParser(){

  if (Serial2.available() > 14){
    while(Serial2.available() > 0){
      inByte = Serial2.read();
      switch (readState){
      case 0:
        if (inByte == HEX_ZERO || inByte == 0x2D || inByte == 0x5A || inByte == 0x87 || inByte == 0xB4 || inByte == 0xE1 || inByte == 0xFF){
          readState = 1;
        }
        break;
      case 1:
        if (inByte == 0xA2){
          readState = 2;
          bufferIndex = 0;
        }
        break;
      case 2:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte & 0x07;
        }
        else{
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newData = true;
        }
        break;
      default:
        break;
      }
    }
    rcCommands.values.aileron  = (constrain(rcCommands.values.aileron,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.elevator  = (constrain(rcCommands.values.elevator,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.throttle  = (constrain(rcCommands.values.throttle,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.rudder  = (constrain(rcCommands.values.rudder,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.gear  = (constrain(rcCommands.values.gear,306,1738)  - 306) * 0.69832 + 1000;
    rcCommands.values.aux1  = (constrain(rcCommands.values.aux1,306,1738)  - 306) * 0.69832 + 1000;
    if (rcCommands.values.aux2 != 0){
      rcCommands.values.aux2  = (constrain(rcCommands.values.aux2,306,1738)  - 306) * 0.69832 + 1000;
    }
  }

}
void DSM2Parser(){

  if (Serial2.available() > 14){
    while(Serial2.available() > 0){
      inByte = Serial2.read();
      switch (readState){
      case 0:
        if (inByte == 0x03 || 0x21){
          readState = 1;
        }
        break;
      case 1:
        if (inByte == 0xA2){
          readState = 2;
          bufferIndex = 0;
        }
        if (inByte == 0xB2){
          readState = 3;
          bufferIndex = 0;
        }
        break;
      case 2:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray1[bufferIndex]] = (inByte & 0x07) | 0x04;
        }
        else{
          rcCommands.buffer[syncArray1[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newData = true;
        }
        break;
      case 3:
        if (bufferIndex % 2 == 0){
          rcCommands.buffer[syncArray2[bufferIndex]] = (inByte & 0x07) | 0x04;
        }
        else{
          rcCommands.buffer[syncArray2[bufferIndex]] = inByte;
        }
        bufferIndex++;
        if(bufferIndex == 14){
          readState = 0;
          newData = true;
        }
        break;
      default:
        break;

      }
    }
    rcCommands.values.aileron  = (constrain(rcCommands.values.aileron,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.elevator  = (constrain(rcCommands.values.elevator,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.throttle  = (constrain(rcCommands.values.throttle,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.rudder  = (constrain(rcCommands.values.rudder,1177,1893)  - 1177) * 1.3908 + 1003;
    rcCommands.values.gear  = (constrain(rcCommands.values.gear,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.aux1  = (constrain(rcCommands.values.aux1,1177,1893)  - 1177) * 1.3908 + 1000;
    rcCommands.values.aux2  = (constrain(rcCommands.values.aux2,1177,1893)  - 1177) * 1.3908 + 1000;
  }

}

void DetectRC(){
  readState = 0;
  Spektrum();
  if (detected == true){
    return;
  }
  readState = 0;
  SBus();
  if (detected == true){
    return;
  }
  else{
    rcType = RC;
  }
  readState = 0;
}

void SBus(){

  Serial1.begin(100000);
  timer = millis();
  while (Serial1.available() == 0){
    if (millis() - timer > 1000){
      return;
    }
  }

  rcType = SBUS;
  detected = true;

}

void Spektrum(){
  Serial2.begin(115200);
  timer = millis();
  while (Serial2.available() == 0){
    if (millis() - timer > 1000){
      return;
    }
  }  
  delay(1);
  while (Serial2.available() > 0){
    inByte = Serial2.read();
    switch(readState){
    case 0:
      if (inByte == 0x03 || 0x21){
        readState = 1;
      }
      if (inByte == HEX_ZERO || inByte == 0x2D || inByte == 0x5A || inByte == 0x87 || inByte == 0xB4 || inByte == 0xE1 || inByte == 0xFF){
        readState = 2;
      }
      break;
    case 1:
      if (inByte == 0xA2 || inByte == 0xB2){
        rcType = DSM2;
        detected = true;
        return;
      }
      else{
        readState = 0;
      }
      break;
    case 2:
      if (inByte == 0xA2){
        rcType = DSMX;
        detected = true;
        return;
      }
      else{
        readState = 0;
      }
      break;
    }

  }
}










