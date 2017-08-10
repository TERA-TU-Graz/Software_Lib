/*
 * systick.h
 *
 * ToDo: add file description here
 *
 * Copyright 2017 Pöschl Rene Copyright and related rights are licensed under the Solderpad Hardware License,
 * Version 0.51 (the License); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
 * Unless required by applicable law or agreed to in writing, software,
 * hardware and materials distributed under this License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */
#include <stdint.h>
#include <config/systick_def.h>

#ifndef LIBRARIES_TERALIB_DRIVERS_INCLUDE_SYSTICK_H_
#define LIBRARIES_TERALIB_DRIVERS_INCLUDE_SYSTICK_H_

#define MS_PER_SECOND (1000)
#define MS_PER_MIN (1000*60)
#define MS_PER_HOUR (1000*3600)
#define MS_PER_DAY (1000*3600*24)

#define TERA_MS_ONLY(ms) (ms%1000)
#define TERA_MS_TO_SECONDS(ms) ((ms/MS_PER_SECOND)%60)
#define TERA_MS_TO_MINUTES(ms) ((ms/MS_PER_MIN)%60)
#define TERA_MS_TO_HOURS(ms) ((ms/MS_PER_HOUR)%24)
#define TERA_MS_TO_DAYS(ms) (ms/MS_PER_DAY)

uint32_t TERA_SysTick_ms();


#endif /* LIBRARIES_TERALIB_DRIVERS_INCLUDE_SYSTICK_H_ */
