/*
 * bq25120.c
 *
 *  Created on: Aug 24, 2017
 *      Author: tdawkins
 */


#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>

#include <drivers/bq25120.h>

/* Example/Board Header files
 * - MAP PINS FROM CC2640R2_LAUNCHXL.h
 *  */
#include "Board.h"

/*
 * Default Value Array
 *
 */
uint8_t default_reg_val[BQ25120_NUM_REGS] = {SM_DEFAULT, FM_DEFAULT, FM_DEFAULT, FC_DEFAULT, ITERM_DEFAULT,
                                             VBREG_DEFAULT, SYS_DEFAULT, LSLDO_DEFAULT, PB_DEFAULT, INLIMBUVLO_DEFAULT,
                                             VBMOM_DEFAULT, VINDPM_DEFAULT};



int BQ25120_init() {

    Display_Handle display;
    //I2C_init();
    //GPIO_init();

    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    //GPIO_write(BQ25120_PIN_CD,0x00);
    //GPIO_write(BQ25120_PIN_LSCTRL,0x01);


    unsigned int    i;
    uint8_t         reg[BQ25120_NUM_REGS];
    uint8_t         txBuffer[2];
    uint8_t         rxBuffer[1];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    // Light up the Red LED and print message to UART
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
    Display_printf(display, 0, 0, "Starting BQ25120_init() function\n");



    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open I2C Connection
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        return 0;
    } else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

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
            reg[i] = rxBuffer[0];
            Display_printf(display, 0, 0, "Register 0x%02x: 0x%02x\n", i, reg[i]);
        }
        else {
            Display_printf(display, 0, 0, "I2C Bus fault\n");
            I2C_close(i2c);
            return 0;
        }
        txBuffer[0]++; // Next Register
    }

    // Set up write initial state transaction
    i2cTransaction.slaveAddress = BQ25120_I2C_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 2;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 1;


    for ( i = 0; i < BQ25120_NUM_REGS; i++){
        usleep(1000);
        txBuffer[0] = i;
        txBuffer[1] = default_reg_val[i];
        if(I2C_transfer(i2c, &i2cTransaction)){
            // Store register values and print out to UART
            reg[i] = rxBuffer[0];
            Display_printf(display, 0, 0, "Write register 0x%02x: 0x%02x\n", i, txBuffer[i]);

        }
        else {
            Display_printf(display, 0, 0, "Failed to write initial value to register. REG 0x%02x: 0x%02x (0x%02x)\n",txBuffer[0], txBuffer[1], reg[i]);
            I2C_close(i2c);
            return 0;
        }
    }



    /* Deinitialized I2C */
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return 1;

}










