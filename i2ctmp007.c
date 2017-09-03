/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2ctmp007.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <drivers/bq25120.h>

/* Example/Board Header files */
#include "Board.h"

#define TASKSTACKSIZE       640

//static Display_Handle display;
void btn0_callback(uint_least8_t);
void btn01_callback(uint_least8_t);

// Display Handle
//Display_Handle display;


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0) {

    /* Call driver init functions */
    Display_init();
    GPIO_init();
    I2C_init();

    GPIO_setCallback(Board_GPIO_BUTTON0, btn01_callback);
    GPIO_enableInt(Board_GPIO_BUTTON0);

    BQ25120_init();

    return(NULL);

}

void btn01_callback(uint_least8_t index) {

    GPIO_clearInt(Board_GPIO_BUTTON0);
    //BQ25120_write_default_regs();

    return NULL;
}


void btn0_callback(uint_least8_t index) {

    Display_init();
    Display_Handle disp;
    disp = Display_open(Display_Type_UART, NULL);
    if (disp == NULL) {
        while (1);
    }

    Display_doPrintf(disp,0,0,"InCallback!!!\n");
    GPIO_clearInt(Board_GPIO_BUTTON0);

    uint8_t regs[BQ25120_NUM_REGS];

    unsigned int    i;
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[1];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    Display_clear(disp);
    // Open I2C Connection
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        Display_printf(disp, 0, 0, "Error Initializing I2C in Callback\n");
        return 0;
    } else {
        Display_printf(disp, 0, 0, "I2C Initialized in _read_regs()!\n");
    }

    // Start with address 0x00
    txBuffer[0] = 0x00;
    // Set up I2C Transaction for BQ25120 IC to read all registers
    i2cTransaction.slaveAddress = BQ25120_I2C_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;

    // Read out all registers
    for (i = 0; i < BQ25120_NUM_REGS; i++) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            // Store register values and print out to UART
            regs[i] = rxBuffer[0];
            Display_printf(disp, 0, 0, "Register 0x%02x: 0x%02x\n", i, regs[i]);
        }
        else {
            Display_printf(disp, 0, 0, "I2C Bus fault\n");
            I2C_close(i2c);
            return 0;
        }

        txBuffer[0]++; // Next Register
    }

    // Close the I2C connection
    I2C_close(i2c);
    Display_printf(disp, 0, 0, "I2C closed!\n");

}
