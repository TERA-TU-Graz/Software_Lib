/*
 * systick.c
 *
 * ToDo: add file description here
 *
 * Copyright 2017 PÃ¶schl Rene Copyright and related rights are licensed under the Solderpad Hardware License,
 * Version 0.51 (the â€œLicenseâ€�); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://solderpad.org/licenses/SHL-0.51.
 * Unless required by applicable law or agreed to in writing, software,
 * hardware and materials distributed under this License is distributed on an â€œAS ISâ€� BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#include "systick.h"



uint32_t TERA_SysTick_ms(){
  return SYSTM001_GetTime();
}
