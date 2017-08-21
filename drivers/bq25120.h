/*
 * bq25120.h
 *
 * Drivers for the TI BQ25120 PMIC
 *
 *  Created on: Aug 19, 2017
 *      Author: tdawkins
 */

#ifndef DRIVERS_BQ25120_H_
#define DRIVERS_BQ25120_H_


#define BQ25120_I2C_ADDR 0x6A
#define BQ25120_NUM_REGS 12

// Define Memory Addresses
#define ADDR_STATUS_SHIP_MODE 0x00
#define ADDR_FAULTS 0x01
#define ADDR_TSCTRL 0x02
#define ADDR_FASTCHGCTRL 0x03
#define ADDR_TERMPRECHG_I2C 0x04
#define ADDR_BATTVOLTAGE 0x05
#define ADDR_SYSVOUT 0x06
#define ADDR_LSLDO 0x07
#define ADDR_PUSHBUTTON 0x08
#define ADDR_ILIM_BATTUVLO 0x09
#define ADDR_BATTMON 0x0A
#define ADDR_VINDPM 0x0B
//TODO: More


// Faults / Fault Masks
#define FAULT_VIN_OV        0x80
#define FAULT_VIN_UV        0x40
#define FAULT_BAT_UVLO      0x20
#define FAULT_BAT_OCP       0x10
#define FAULT_VIN_OV_M      0x08
#define FAULT_VIN_UV_M      0x04
#define FAULT_BAT_UVLO_M    0x02
#define FAULT_BAT_OCP_M     0x01

/*
 * Fast Charge Control Register
 * ICHRG_RANGE = 0:
 * ICHRG = 5 mA + ICHRG_CODE * 1 mA
 * ICHRG_RANGE = 1:
 * ICHRG = 40 mA + ICHRG_CODE * 10 mA
 *
 * Packet Data:
 * FC_ICHRG_RANG | FC_ICHR_CODE | FC_CHRG_X | FC_HIGHZ_X
 */
#define FC_ICHRG_RANGE     0x00 // Range 1
#define FC_ICHRG_CODE      0x08 // 5mA + 2mA (Zpower 6.8 mA)
#define FC_CHRG_ENABLE     0x00
#define FC_CHRG_DISABLE    0x02
#define FC_HIGHZ_ENABLE    0x01
#define FC_HIGHZ_DISABLE   0x00






#endif /* DRIVERS_BQ25120_H_ */
