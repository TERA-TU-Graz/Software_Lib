/*
 * can_defines.h
 *
 * ToDo: add file description here
 *
 * Copyright 2017 Pöschl Rene Copyright and related rights are licensed under the Solderpad Hardware License,
 * Version 0.51 (the “License”); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
 * Unless required by applicable law or agreed to in writing, software,
 * hardware and materials distributed under this License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */
#ifndef SRC_COMMUNICATION_CAN_DEFINES_FENNEK_H_
#define SRC_COMMUNICATION_CAN_DEFINES_FENNEK_H_

//
// alle vorhandenen Messages, der verschiedenen Platinen
// mögliche message'ids von 0 - 30, 32 möglich, 31 ist reserviert für unknown
// d.h.  pro platine sind 31 messages möglich
//
// alle vorhandenen Boards
// mögliche board id's boards von 0 - 14, 16 mögliche, 15 ist reserviert für unknown


//
// BOARD ID's
//

#define CAN_BOARD_BORDCOMPUTER        0  // bordcomputer neuer fennek
#define CAN_BOARD_KERS                1  // KERS
#define CAN_BOARD_BMS                 2  // BMS Fahrbatterie, versorgt Motoren
#define CAN_BOARD_SAFETYSYSTEM        3  // BMS Sicherheitsbatterie, versorgt Scheibenwischer, Licht, Hupe, ...
#define CAN_BOARD_MC_LEFT             4  // linke Motorplatine
#define CAN_BOARD_MC_RIGHT            5  // rechte Motorplatine
#define CAN_BOARD_MOTORCONTROL        CAN_BOARD_MC_LEFT   // wenn nur eine motorplatine verbaut
#define CAN_BOARD_INTERFACE           6  // Interface (Adapter zu Tablet)
#define CAN_BOARD_TELEMETRY           7  // Telemetry platine fennek neu
#define CAN_BOARD_BC_PANTHER          8  // Panther Bordcomputer

// im neuen Fennek
// gibts 4 Platinen mit CAN
//  BMS          - ID:
//  Motorplatine - ID: CAN_BOARD_MOTORCONTROL entspricht der linken motorplatine
//  Bordcomputer - ID:
//  Telemetrie   - ID:


//
// BATTERY-MANAGEMENT-SYSTEM
//

// can messages
#define CAN_MSG_BMS_HV_CURRENT           0
#define CAN_MSG_BMS_AUX_CURRENT          1
#define CAN_MSG_BMS_HV_VOLTAGE           2
#define CAN_MSG_BMS_AUX_VOLTAGE          3
#define CAN_MSG_BMS_HV_TEMPERATURE       4
#define CAN_MSG_BMS_AUX_TEMPERATURE      5
#define CAN_MSG_BMS_HV_POWER             6
#define CAN_MSG_BMS_AUX_POWER            7
#define CAN_MSG_BMS_HV_ID                8
#define CAN_MSG_BMS_AUX_ID               9
#define CAN_MSG_BMS_HV_PERCENTAGE        10
#define CAN_MSG_BMS_AUX_PERCENTAGE       11

// status bit's
#define CAN_STATE_BMS_AUX_OUTPUT_ENABLED       0x00000001
#define CAN_STATE_BMS_AUX_OVERCURRENT          0x00000002
#define CAN_STATE_BMS_AUX_UNDERVOLTAGE         0x00000004
#define CAN_STATE_BMS_AUX_OVERTEMPERATURE      0x00000008

#define CAN_STATE_BMS_HV_OUTPUT_ENABLED        0x000010000
#define CAN_STATE_BMS_HV_OVERCURRENT           0x000020000
#define CAN_STATE_BMS_HV_UNDERVOLTAGE          0x000040000
#define CAN_STATE_BMS_HV_OVERTEMPERATURE       0x000080000


//
// MOTOR CONTROL
//

// motor controller (left or right)
#define CAN_MSG_MC_CURRENT             0
#define CAN_MSG_MC_RPM_MOTOR           3
#define CAN_MSG_MC_RPM_WHEEL           4
#define CAN_MSG_MC_IDQ                 5
#define CAN_MSG_MC_VDQ                 6


// status bit's
#define CAN_STATE_MC_CONTROL_FEEDBACK          0x00000001UL  // enables speed control per feedback device
#define CAN_STATE_MC_CONTROL_CAN_TORQUE        0x00000002UL  // enables torque control over can
#define CAN_STATE_MC_CONTROL_USER_TORQUE       0x00000004UL  // enables torque control by the user

#define CAN_STATE_MC_DIRECTION                 0x00000008UL  // fahrrichtung des motors: 1 rückwärts, 0 vorwärts
#define CAN_STATE_MC_RECUPERATE                0x00000010UL  // enable recuperate
#define CAN_STATE_MC_OUTPUT_ENABLED            0x00000020UL  // half bridges on/off
#define CAN_STATE_MC_BRAKE_ON                  0x00000040UL  // motor phase short to ground
#define CAN_STATE_MC_POS_LEFT                  0x00000080UL  // motor on the left side, beide null, Motor in der Mitte
#define CAN_STATE_MC_POS_RIGHT                 0x00000100UL  // motor on the right side, beide null, Motor in der Mitte
#define CAN_STATE_MC_ONLY_ALIGN                0x00000200UL  // es soll nur ein align ausgeführt werden
#define CAN_STATE_MC_IDLE_ALIGN                0x00000400UL  // align using encoder
#define CAN_STATE_MC_FIRST_START               0x00000800UL  // platine wird eingeschaltet

// motor states
// 12 possible states
#define CAN_STATE_MC_STATE_IDLE                0x10000000UL  // nix tut sich
#define CAN_STATE_MC_STATE_INIT                0x20000000UL  // power state init
#define CAN_STATE_MC_STATE_ALIGN               0x40000000UL  // motor align
#define CAN_STATE_MC_STATE_RUN                 0x80000000UL  // run mode
#define CAN_STATE_MC_STATE_STOP                0x01000000UL  // motor is stopping
#define CAN_STATE_MC_STATE_AMS_READOUT         0x02000000UL  // motor is stopping


// mask's
#define CAN_STATE_MC_MASK_CONTROL                (CAN_STATE_MC_CONTROL_FEEDBACK | CAN_STATE_MC_CONTROL_CAN_TORQUE | CAN_STATE_MC_CONTROL_USER_TORQUE)
#define CAN_STATE_MC_MASK_STATE                  0xFFF00000UL
#define CAN_STATE_MC_MASK_RUN                    (CAN_STATE_MC_STATE_ALIGN | CAN_STATE_MC_STATE_RUN)  // in run mode, STATE_RUN, STATE_ALIGN


//
//  BORDCOMPUTER
//

// FENNEK

// can-messages
#define CAN_MSG_BC_ACCELERATION        0   // normiert 0 - 65535
#define CAN_MSG_BC_TEMP                2

// status bit's
#define CAN_STATE_BC_ALARM             0x000001
#define CAN_STATE_BC_BRAKE_FRONT       0x000010
#define CAN_STATE_BC_BRAKE_REAR        0x000020
#define CAN_STATE_BC_HORN              0x000040

#define CAN_STATE_BC_MASK_BRAKE        (CAN_STATE_BC_BRAKE_FRONT | CAN_STATE_BC_BRAKE_REAR)


//
//  Panther Bordcomputer
//  zusätzliche messages
#define CAN_MSG_BC_BRAKE                  1   // normiert 0 - 65535
#define CAN_MSG_BC_DIRECTION              2   // 0: forward; 1: backward
#define CAN_MSG_BC_PARK                   3   // 0: nix, 1: elek-feststellbremse



#define BC_STATE_ERROR_INIT              0x000001
#define BC_STATE_FAKE                    0x000002

#define BC_STATE_BACKWARDS               0x000400
#define BC_STATE_SHUTDOWN                0x000800

#define BC_STATE_LIGHT                   0x0001000
#define BC_STATE_LIGHT_BOOST             0x0002000
#define BC_STATE_BRAKE                   0x0004000
#define BC_STATE_BLINK_LEFT              0x0008000
#define BC_STATE_BLINK_RIGHT             0x0010000
#define BC_STATE_HORN                    0x0020000
#define BC_STATE_OK_BUTTON               0x0040000
#define BC_STATE_CRUISE_CONTROL_UP       0x0080000
#define BC_STATE_CRUISE_CONTROL_DOWN     0x0100000
#define BC_STATE_RAIN_INTENSITY_1        0x0200000
#define BC_STATE_RAIN_INTENSITY_2        0x0400000
#define BC_STATE_WSW_1                   0x0800000
#define BC_STATE_WSW_2                   0x1000000


//
//  TELEMETRIE
//

// can-messages
#define CAN_MSG_TELE_TIME             0   // normiert 0 - 65535
#define CAN_MSG_TELE_GPS_LATITUDE     1
#define CAN_MSG_TELE_GPS_LONGITUDE    2
#define CAN_MSG_TELE_GPS_ALTITUDE     3

// status bit's
#define CAN_STATE_TELE_GPS_VALID    0x000001



#endif /* SRC_COMMUNICATION_CAN_DEFINES_FENNEK_H_ */
