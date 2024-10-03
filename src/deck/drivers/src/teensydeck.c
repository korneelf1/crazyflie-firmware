/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 BitCraze AB
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
 * vl53l0x.c: Time-of-flight distance sensor driver
 */

#define DEBUG_MODULE "TEENSY"

#include "FreeRTOS.h"
#include "task.h"

#include "deck.h"
#include "system.h"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "range.h"
#include "static_mem.h"

#include "uart1.h"
#include "teensydeck.h"

#include "cf_math.h"

static bool isInit = false;
bool status = false;


//////////////COMMUNICATION VARIABLES///////////////////
#define START_BYTE_SERIAL_CF 0x9A

struct serial_control_in myserial_control_in;
struct serial_control_out myserial_control_out;
struct target_state mytarget_state;
uint8_t serial_cf_msg_buf_out[ 2*sizeof(struct serial_control_out) ] = {0};
uint16_t serial_cf_buf_out_cnt = 0;
int serial_cf_received_packets = 0;

bool receiving;
bool sending;

int receiving_outer = 0;
int sending_outer = 0;
int set_control_outer = 0;

/////////////INTERNAL LOG VARIABLES
logVarId_t idPosx, idPosy, idPosz, idVelBodyX, idVelBodyY, idVelBodyZ;
float posx, posy, posz, velBodyX, velBodyY, velBodyZ;

logvarID_t idQuatw, idQuatx, idQuaty, idQuatz;
float quatw, quatx, quaty, quatz;
logVarId_t idGyroX, idGyroY, idGyroZ, idAccX, idAccY, idAccZ;
float gyroX, gyroY, gyroZ, accX, accY, accZ;
logVarId_t idMotor1, idMotor2, idMotor3, idMotor4;
int motor1, motor2, motor3, motor4;

paramVarId_t idSnnType;
int snnType;

void serialParseMessageOut(void)
{
  //Copy received buffer to structure
  memmove(&myserial_control_out,&serial_cf_msg_buf_out[1],sizeof(struct serial_control_out)-1);
//   DEBUG_PRINT("Correct message received and storing\n");
//   DEBUG_PRINT("Stored roll p, i, d is %i, %i, %i\n", myserial_control_in.roll_p, myserial_control_in.roll_i, myserial_control_in.roll_d);
}

void setControlInMessage(void) 
{
    posx = logGetFloat(idPosx);
    posy = logGetFloat(idPosy);
    posz = logGetFloat(idPosz);
    velBodyX = logGetFloat(idVelBodyX);
    velBodyY = logGetFloat(idVelBodyY);
    velBodyZ = logGetFloat(idVelBodyZ);
    quatw = logGetFloat(idQuatw);
    quatx = logGetFloat(idQuatx);
    quaty = logGetFloat(idQuaty);
    quatz = logGetFloat(idQuatz);
    gyroX = logGetFloat(idGyroX);
    gyroY = logGetFloat(idGyroY);
    gyroZ = logGetFloat(idGyroZ);

    myserial_control_in.pos_x = posx;
    myserial_control_in.pos_y = posy;
    myserial_control_in.pos_z = posz;
    myserial_control_in.vel_body_x = velBodyX;
    myserial_control_in.vel_body_y = velBodyY;
    myserial_control_in.vel_body_z = velBodyZ;
    myserial_control_in.quat_w = quatw;
    myserial_control_in.quat_x = quatx;
    myserial_control_in.quat_y = quaty;
    myserial_control_in.quat_z = quatz;
    myserial_control_in.gyro_x = gyroX;
    myserial_control_in.gyro_y = gyroY;
    myserial_control_in.gyro_z = gyroZ;

}

void setTargetState(float* state) 
{
    target_state.pos_x = state[0];
    target_state.pos_y = state[1];
    target_state.pos_z = state[2];
    target_state.vel_body_x = state[3];
    target_state.vel_body_y = state[4];
    target_state.vel_body_z = state[5];
    target_state.quat_w = state[6];
    target_state.quat_x = state[7];
    target_state.quat_y = state[8];
    target_state.quat_z = state[9];
    target_state.gyro_x = state[10];
    target_state.gyro_y = state[11];
    target_state.gyro_z = state[12];
}
// Read a control out message over uart
void uartReadControlOutMessage(void) 
{
    uint8_t serial_cf_byte_in;
    // uart1Getchar(&serial_cf_byte_in);
    if (!uart1GetDataWithTimeout(&serial_cf_byte_in, 100)) {
        receiving = false;
        // if status was true, set it to false for debugging purposes
        if (status) {
            DEBUG_PRINT("Did not receive message on time, trying to resend\n");
            status = false;
        }
    };

    if ((serial_cf_byte_in == START_BYTE_SERIAL_CF) || (serial_cf_buf_out_cnt > 0)) {
        serial_cf_msg_buf_out[serial_cf_buf_out_cnt] = serial_cf_byte_in;
        serial_cf_buf_out_cnt++;
    }
    if (serial_cf_buf_out_cnt > sizeof(struct serial_control_out)  ) {
        serial_cf_buf_out_cnt = 0;
        uint8_t checksum_in_local = 0;
        for(uint16_t i = 1; i < sizeof(struct serial_control_out) ; i++){
            checksum_in_local += serial_cf_msg_buf_out[i];
        }
        if(checksum_in_local == serial_cf_msg_buf_out[sizeof(struct serial_control_out)]){
            serialParseMessageOut();
            serial_cf_received_packets++;
            // if status was false, set it to true for debugging purposes
            if (!status) {
                DEBUG_PRINT("Connection (re-)gained\n");
                status = true;
            }
        }
        else {
            DEBUG_PRINT("Incorrect message\n");
        }
        // receiving done; set to false
        receiving = false;
    }
}

// Send a ControlIn message via uart to Teensy
void uartSendControlInMessage(void)
{
    //Calculate checksum for outbound packet: 
    uint8_t *buf_send = (uint8_t *)&myserial_control_in;
    myserial_control_in.checksum_in = 0;
    for(uint16_t i = 0; i < sizeof(struct serial_control_in) - 1; i++){
        myserial_control_in.checksum_in += buf_send [i];
    }
    uint8_t startByte = START_BYTE_SERIAL_CF;
    uart1SendDataDmaBlocking(1, &startByte);
    uart1SendDataDmaBlocking(sizeof(struct serial_control_in), buf_send);
    // uart1SendData(1, &startByte);
    // uart1SendData(sizeof(struct serial_control_in), buf_send);
    // set sending is false after message is sent
    sending = false;
    receiving = true;
    // DEBUG_PRINT("Just sent data\n");
}

void teensyInit(DeckInfo* info)
{
  if (isInit)
    return;
  // initialize connection with the Teensy
  // at baudrate 115200, 500hz not achieved
  // at baudrate 460800, seems to work
  // at baudrate 921600, module seems to crash
  uart1Init(460800);


  // get the logVarIds that are used to get the state/target info
  idPosx = logGetVarId("state_input", "x");
  idPosy = logGetVarId("state_input", "y");
  idPosz = logGetVarId("state_input", "z");
  idVelBodyX = logGetVarId("state_input", "vx");
  idVelBodyY = logGetVarId("state_input", "vy");
  idVelBodyZ = logGetVarId("state_input", "vz");
  idQuatw = logGetVarId("state_input", "qw");
  idQuatx = logGetVarId("state_input", "qx");
  idQuaty = logGetVarId("state_input", "qy");
  idQuatz = logGetVarId("state_input", "qz");
    
  idGyroX = logGetVarId("state_input", "wx");
  idGyroY = logGetVarId("state_input", "wy");
  idGyroZ = logGetVarId("state_input", "wz");

  xTaskCreate(teensyTask, TEENSY_TASK_NAME, TEENSY_TASK_STACKSIZE, NULL, TEENSY_TASK_PRI, NULL);

  DEBUG_PRINT("FINISHED CREATING TASK\n");
  status = true;
  isInit = true;
}

bool teensyTest(void)
{
  if (!isInit)
    return false;
  else
    return true;
}

void teensyTask(void* arg)
{
  systemWaitStart();
  TickType_t xLastWakeTime;

  xLastWakeTime = xTaskGetTickCount();

  TickType_t xLastDebugTime;
  xLastDebugTime = T2M(xTaskGetTickCount());

  while (1) {
    if (sending) {
        uint32_t now_ms = T2M(xTaskGetTickCount());
        setControlInMessage();
        uint32_t after = T2M(xTaskGetTickCount());
        set_control_outer = set_control_outer + (after - now_ms);
        uartSendControlInMessage();
        uint32_t after2 = T2M(xTaskGetTickCount());
        sending_outer = sending_outer + (after2 - after);
    } else if (receiving) {
        uint32_t now_ms = T2M(xTaskGetTickCount());
        uartReadControlOutMessage();
        uint32_t after = T2M(xTaskGetTickCount());
        receiving_outer = receiving_outer + (after - now_ms);
    } else {
        vTaskDelayUntil(&xLastWakeTime, F2T(100));
        sending = true;
    }
    // Printing the amount of received messages over the last seconds
    uint32_t now_ms = T2M(xTaskGetTickCount());
    if (now_ms - xLastDebugTime > 1000) {
        DEBUG_PRINT("received %i messages in the last second, spent %i ms sending, %i, setting message, %i receiving\n", serial_cf_received_packets, sending_outer, set_control_outer, receiving_outer);
        serial_cf_received_packets = 0;
        sending_outer = 0;
        receiving_outer = 0;
        set_control_outer = 0;
        xLastDebugTime = now_ms;
    }
  }
}

int16_t teensyGetMotor1(void) {
    return myserial_control_out.motor_1;
}

int16_t teensyGetMotor2(void) {
    return myserial_control_out.motor_2;
}

int16_t teensyGetMotor3(void) {
    return myserial_control_out.motor_3;
}
int16_t teensyGetMotor4(void) {
    return myserial_control_out.motor_4;
}

bool teensyGetStatus(void) {
    return status;
}


static const DeckDriver teensy_deck = {
  .vid = 0xBC,
  .pid = 0x29,
  .name = "teensy",
  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1,
//   .requiredEstimator = StateEstimatorTypeKalman,
  .init = teensyInit,
  .test = teensyTest,
};

DECK_DRIVER(teensy_deck);

/**
 * Logging variables for the command and reference signals for the
 * attitude controller
 */
LOG_GROUP_START(snn_control)
/**
 * @brief SNN control motor output 1 to 4
 */
LOG_ADD(LOG_INT16, motor1, &myserial_control_out.motor_1)
LOG_ADD(LOG_INT16, motor2, &myserial_control_out.motor_2)
LOG_ADD(LOG_INT16, motor3, &myserial_control_out.motor_3)
LOG_ADD(LOG_INT16, motor4, &myserial_control_out.motor_4)

/**
 * @brief SNN control status
 */
LOG_ADD(LOG_UINT8, status, &status)
/**
 * @brief SNN inputs
 */
LOG_ADD(LOG_INT16, posx, &posx)
LOG_ADD(LOG_INT16, posy, &posy)
LOG_ADD(LOG_INT16, posz, &posz)
LOG_ADD(LOG_FLOAT, velBodyX, &velBodyX)
LOG_ADD(LOG_FLOAT, velBodyY, &velBodyY)
LOG_ADD(LOG_FLOAT, velBodyZ, &velBodyZ)
LOG_ADD(LOG_FLOAT, orientationRoll, &orientationRoll)
LOG_ADD(LOG_FLOAT, orientationPitch, &orientationPitch)
LOG_ADD(LOG_FLOAT, orientationYaw, &orientationYaw)
LOG_ADD(LOG_FLOAT, gyroX, &gyroX)
LOG_ADD(LOG_FLOAT, gyroY, &gyroY)
LOG_ADD(LOG_FLOAT, gyroZ, &gyroZ) 
LOG_GROUP_STOP(snn_control)

/**
 * Logging variables for the IMU
 */
LOG_GROUP_START(imu)
/**
 * @brief IMU readings
 */
LOG_ADD(LOG_FLOAT, accX, &accX)
LOG_ADD(LOG_FLOAT, accY, &accY)
LOG_ADD(LOG_FLOAT, accZ, &accZ)
LOG_ADD(LOG_FLOAT, gyroX, &gyroX)
LOG_ADD(LOG_FLOAT, gyroY, &gyroY)
LOG_ADD(LOG_FLOAT, gyroZ, &gyroZ)

LOG_GROUP_STOP(imu)

PARAM_GROUP_START(deck)

PARAM_ADD_CORE(PARAM_UINT8 | PARAM_RONLY, bcTeensy, &isInit)

PARAM_GROUP_STOP(deck)