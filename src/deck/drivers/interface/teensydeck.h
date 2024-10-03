/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * teensydeck.h: teensy deck driver
 */

#ifndef _TEENSY_H_
#define _TEENSY_H_

#include "deck_core.h"
'''
/**
EXPECTED BY TEENSY CODE
/**
  inputs[0]  = myserial_control_in.pos_x;
  inputs[1]  = myserial_control_in.pos_y;
  inputs[2]  = myserial_control_in.pos_z;
  inputs[3]  = myserial_control_in.vel_body_x;
  inputs[4]  = myserial_control_in.vel_body_y;
  inputs[5]  = myserial_control_in.vel_body_z;
  inputs[6]  = myserial_control_in.quat_w;
  inputs[7]  = myserial_control_in.quat_x;
  inputs[8]  = myserial_control_in.quat_y;
  inputs[9]  = myserial_control_in.quat_z;
  inputs[10] = myserial_control_in.gyro_x;
  inputs[11] = myserial_control_in.gyro_y;
  inputs[12] = myserial_control_in.gyro_z;

void setOutputMessage(void) {
  myserial_control_out.motor_1 = saturateSignedInt16(controller.out[0]);
  myserial_control_out.motor_2 = saturateSignedInt16(controller.out[1]);
  myserial_control_out.motor_3 = saturateSignedInt16(controller.out[2]);
  myserial_control_out.motor_4 = saturateSignedInt16(controller.out[3]);
'''
struct __attribute__((__packed__)) serial_control_in {
    //position
    float pos_x;
    float pos_y;
    float pos_z;
    //velocity
    float vel_body_x;
    float vel_body_y;
    float vel_body_z;
    //attitude
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    //gyro
    float gyro_x;
    float gyro_y;
    float gyro_z;

    //CHECKSUM
    uint8_t checksum_in;
};

struct __attribute__((__packed__)) target_state {
    //position
    float pos_x;
    float pos_y;
    float pos_z;
    //velocity
    float vel_body_x;
    float vel_body_y;
    float vel_body_z;
    //attitude
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    //gyro
    float gyro_x;
    float gyro_y;
    float gyro_z;

    //CHECKSUM
    uint8_t checksum_in;
};


struct __attribute__((__packed__)) serial_control_out {
    //motor commands
    int16_t motor_1;
    int16_t motor_2;
    int16_t motor_3;
    int16_t motor_4;
    //CHECKSUM
    uint8_t checksum_out;
};

void teensyInit(DeckInfo* info);

bool teensyTest(void);
void teensyTask(void* arg);

void setTargetState(float* state);  //set the target state for the teensy
extern bool teensyGetStatus(void);
extern int16_t teensyGetMotor1(void);
extern int16_t teensyGetMotor2(void);
extern int16_t teensyGetMotor3(void);
extern int16_t teensyGetMotor4(void);

extern struct serial_control_out myserial_control_out;
extern bool status;

#endif /* _TEENSY_H_ */
