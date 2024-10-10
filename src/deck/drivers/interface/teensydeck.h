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

struct __attribute__((__packed__)) serial_control_in {
    //position
    float pos_x;
    float pos_y;
    float pos_z;
    //velocity
    float vel_x;
    float vel_y;
    float vel_z;
    //attitude
    float orient_1;
    float orient_2;
    float orient_3;
    float orient_4;
    float orient_5;
    float orient_6;
    float orient_7;
    float orient_8;
    float orient_9;
    //gyro
    float gyro_x;
    float gyro_y;
    float gyro_z;

    // type (0 for hover, 1 for follow)
    // uint8_t type;
    //CHECKSUM
    uint8_t checksum_in;
};

// struct __attribute__((__packed__)) target_state {
//     //position
//     float pos_x;
//     float pos_y;
//     float pos_z;
//     //velocity
//     float vel_x;
//     float vel_y;
//     float vel_z;
//     //attitude
//     float quat_w;
//     float quat_x;
//     float quat_y;
//     float quat_z;
//     //gyro
//     float gyro_x;
//     float gyro_y;
//     float gyro_z;

//     //CHECKSUM
//     uint8_t checksum_in;
// };


struct __attribute__((__packed__)) serial_control_out {
    //motor commands
    float motor_1;
    float motor_2;
    float motor_3;
    float motor_4;
    //CHECKSUM
    uint8_t checksum_out;
};

void teensyInit(DeckInfo* info);

bool teensyTest(void);
void teensyTask(void* arg);

void setTargetState(float* state);  //set the target state for the teensy
extern bool teensyGetStatus(void);
extern float teensyGetMotor1(void);
extern float teensyGetMotor2(void);
extern float teensyGetMotor3(void);
extern float teensyGetMotor4(void);

extern struct serial_control_out myserial_control_out;
extern bool status;

#endif /* _TEENSY_H_ */
