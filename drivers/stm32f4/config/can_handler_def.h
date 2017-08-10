/*
 * can_handler_def.h
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
#ifndef LIBRARIES_TERALIB_DRIVERS_STM32F4_CONFIG_CAN_HANDLER_DEF_H_
#define LIBRARIES_TERALIB_DRIVERS_STM32F4_CONFIG_CAN_HANDLER_DEF_H_

#include <stm32f4xx_hal_can.h>
#include <stm32f4xx_hal.h>

#define CAN_MAX_NUM_CHANNELS 2
#define CAN_CAN2_FIRST_FILTER_BANK_NR 14

typedef CAN_HandleTypeDef TeraCanChannel;


#endif /* LIBRARIES_TERALIB_DRIVERS_STM32F4_CONFIG_CAN_HANDLER_DEF_H_ */
