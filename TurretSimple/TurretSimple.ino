//#define HAL_CAN_MODULE_ENABLED
#include "STM32_CAN.h"
#include "hal_conf_extra.h"
#include <vector>
#include <algorithm>
#include <sstream>
#include <iostream>

//oridigl is PD_0 and PD_1
STM32_CAN Can1(PD_0, PD_1);  //by PinName. Finds matching peripheral automatically

bool dr16Recieved = false;

void setup() {                            //serial_8E1

  Can1.begin(false);
  Can1.setBaudRate(1000000);  //1M

  pinMode(PE11, OUTPUT);  //LED R
  pinMode(PF14, OUTPUT);  //LED G
  Serial.begin(9600);
}

void setCan(CAN_message_t& CAN, int decimal) {
  uint16_t canValue(decimal), canInvValue(-decimal);

  //Left side
  CAN.buf[0] = canValue >> 8;
  CAN.buf[1] = canValue & 0x00ff;

  CAN.buf[2] = canValue >> 8;
  CAN.buf[3] = canValue & 0x00ff;

  //Right side
  CAN.buf[4] = canInvValue >> 8;
  CAN.buf[5] = canInvValue & 0x00ff;

  CAN.buf[6] = canInvValue >> 8;
  CAN.buf[7] = canInvValue & 0x00ff;
}

bool pp = true;

void loop() {

  CAN_message_t CAN_shooter = {
    .id = 0x200,  // can identifier
    .len = 8,     // length of data
    .buf = {
      0xC1, 0x80,
      0x3E, 0x80,
      0x23, 0x28,
      0x23, 0x28 }  // data
  };

  Can1.write(CAN_shooter);

  delay(100);

  if (1) {
    pp = !pp;
  }
  digitalWrite(PE11, pp);   // turn the LED on (HIGH is the voltage level)
  digitalWrite(PF14, !pp);  // turn the LED on (HIGH is the voltage level)
}
