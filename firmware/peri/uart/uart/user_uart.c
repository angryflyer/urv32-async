//////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText: 2021, Dinesh Annayya
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0
// SPDX-FileContributor: Dinesh Annayya <dinesha@opencores.org>
// //////////////////////////////////////////////////////////////////////////
#define SC_SIM_OUTPORT (0xf0000000)
#define uint32_t  unsigned int

// #define reg_mprj_gpio_afr0 (*(volatile uint32_t*)(0x50000000 + 0x1c))
#define reg_mprj_uart_cr   (*(volatile uint32_t*)(0x10010000 + 0x0))

#define reg_mprj_uart_reg0 (*(volatile uint32_t*)0x10010000)
#define reg_mprj_uart_reg1 (*(volatile uint32_t*)0x10010004)
#define reg_mprj_uart_reg2 (*(volatile uint32_t*)0x10010008)
#define reg_mprj_uart_reg3 (*(volatile uint32_t*)0x1001000C)
#define reg_mprj_uart_reg4 (*(volatile uint32_t*)0x10010010)
#define reg_mprj_uart_reg5 (*(volatile uint32_t*)0x10010014)
#define reg_mprj_uart_reg6 (*(volatile uint32_t*)0x10010018)
#define reg_mprj_uart_reg7 (*(volatile uint32_t*)0x1001001C)
#define reg_mprj_uart_reg8 (*(volatile uint32_t*)0x10010020)

int main()
{
    uint32_t rdata;

    // rdata = reg_mprj_gpio_afr0;
    // reg_mprj_gpio_afr0 = (rdata | 0x6);//mux to uart0
    reg_mprj_uart_cr   = 0x7;//same with tb

    while(1) {
       // Check UART RX fifo has data, if available loop back the data
       // Also check txfifo is not full
    //    if((reg_mprj_uart_reg8 != 0) && ((reg_mprj_uart_reg4 & 0x1) != 0x1)) { 
	//    reg_mprj_uart_reg5 = reg_mprj_uart_reg6;
    //    }
       if((reg_mprj_uart_reg4 & 0x1) != 0x1) { 
	   reg_mprj_uart_reg5 = 0x5a;
       }
    }

    return 0;
}
