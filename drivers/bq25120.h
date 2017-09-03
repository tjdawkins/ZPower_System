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

// TODO: Pin mapping
#define BQ25120_PIN_CD      Board_DIO0
#define BQ25120_PIN_INT     Board_DIO12
#define BQ25120_PIN_RESET   Board_DIO15
#define BQ25120_PIN_LSCTRL  Board_DIO21


// I2C Interface
#define BQ25120_I2C_ADDR 0x6A
#define BQ25120_NUM_REGS 12

// Define Memory Addresses
#define ADDR_STATUS_SHIP_MODE   0x00
#define ADDR_FM                 0x01
#define ADDR_TSCTRL             0x02
#define ADDR_FASTCHGCTRL        0x03
#define ADDR_TERMPRECHG_I2C     0x04
#define ADDR_BATTVOLTAGE        0x05
#define ADDR_SYSVOUT            0x06
#define ADDR_LSLDO              0x07
#define ADDR_PUSHBUTTON         0x08
#define ADDR_ILIM_BATTUVLO      0x09
#define ADDR_BATTMON            0x0A
#define ADDR_VINDPM             0x0B
//TODO: More


// Faults / Fault Masks
#define FAULT_VIN_OV            0x80
#define FAULT_VIN_UV            0x40
#define FAULT_BAT_UVLO          0x20
#define FAULT_BAT_OCP           0x10
#define FAULT_VIN_OV_M          0x08
#define FAULT_VIN_UV_M          0x04
#define FAULT_BAT_UVLO_M        0x02
#define FAULT_BAT_OCP_M         0x01

/*
 * Ship Mode Control Register 0x00h
 *
 */
#define SM_DEFAULT              0x00

/*
 * Faults Mask Register 0x01h
 *
 */
#define FM_DEFAULT              0x01


/*
 * TS Control and Faults Mask Register 0x02
 *
 */
#define TS_DEFAULT              0x88


/*
 * Fast Charge Control Register 0x03
 * ICHRG_RANGE = 0:
 * ICHRG = 5 mA + ICHRG_CODE * 1 mA
 * ICHRG_RANGE = 1:
 * ICHRG = 40 mA + ICHRG_CODE * 10 mA
 *
 * Packet Data:
 * FC_ICHRG_RANG | FC_ICHR_CODE | FC_CHRG_X | FC_HIGHZ_X
 */
#define FC_ICHRG_RANGE          0x00 // Range 1
#define FC_ICHRG_CODE           0x08 // 5mA + 2mA (Zpower 6.8 mA)
#define FC_CHRG_ENABLE          0x00
#define FC_CHRG_DISABLE         0x02
#define FC_HIGHZ_ENABLE         0x01
#define FC_HIGHZ_DISABLE        0x00
#define FC_DEFAULT              FC_CHRG_ENABLE | FC_HIGHZ_DISABLE | FC_ICHRG_CODE | FC_ICHRG_RANGE
/*
 * Termination/Precharge and I2C Regs 0x04
 * TERM_RANGE = 0:
 * ITERM = 500 uA + ITERM_CODE * 500 uA
 * ICHRG_RANGE = 1:
 * ICHRG = 6 mA + ICHRG_CODE * 1 mA
 *
 * Packet Data:
 * FC_ICHRG_RANG | FC_ICHR_CODE | FC_CHRG_X | FC_HIGHZ_X
 */
#define ITERM_ICHRG_RANGE     0x00 // Range 1
#define ITERM_ICHRG_CODE      0x20 // 500 uA + 4mA (Zpower 4.5 mA)
#define ITERM_TERM_ENABLE     0x02
#define ITERM_TERM_DISABLE    0x00
#define ITERM_BIT_0           0x00
#define ITERM_DEFAULT         ITERM_ICHRG_CODE | ITERM_ICHRG_RANGE | ITERM_TERM_ENABLE | ITERM_BIT_0

/*
 *  Battery Regulation        0x05
 *
 *  VBATREG = 3.6V + VBREG_CODE * 10 mV
 */
#define VBREG_CODE            0x08 // 3.6 V + 40 mV (ZPower 4V)
#define VBREG_DEFAULT         0x08
/*
 * SYS Vout Control Registers 0x06
 *
 * SYS_SEL: 00 (1.1 V - 1.2 V)
 *          01 (1.3 V - 2.8 V) => 1.3 V + SYS_VOUT_CODE * 100 mV
 *          11 (1.8 V - 3.3 V) => 1.8 V + SYS_VOUT_CODE * 100 mV
 *
 * SYS_VOUT_CODE B4:B1
 *
 */
#define SYS_ENABLE            0x80
#define SYS_DISABLE           0x00
#define SYS_SEL               0x60 // 1.8 V
#define SYS_CODE              0x1D // 1.8 V + 1.5 V = 3.3 V
#define SYS_DEFAULT           SYS_ENABLE |SYS_SEL | SYS_CODE
/*
 *  Load Switch and LDO Control Register 0x07
 *
 *  EN_LS_LDO       7       Enable
 *  LS_LDO_V        6:2     Voltage (0)010 10(00)
 *      VLSLDO = 0.8 V + LS_LDOCODE * 100 mV
 *      VLSLDO > 3.3 => passthrough: VLSLDO = VINLS - VDROPOUT
 *      NOTE: To change VLSLDO LSCTRL and EN_LS_LDO must be disabled
 *
 *  Unused          1       0
 *  MRRESET_VIN     0
 */

#define LSLDO_ENABLE        0x80
#define LSLDO_DISABLE       0x00
#define LSLDO_CODE          0x28 // Default change if necessary 0x64 = 3.3 V
#define LSLDO_MR_VIN        0x00 // TODO: Use this function?
#define LSLDO_DEFAULT       LSLDO_ENABLE | LSLDO_CODE | LSLDO_MR_VIN


/*
 * Push Button Control Register 0x08
 */
#define PB_DEFAULT          0x68


/*
 * INLIM and Battery UVLO Control Register 0x09
 *
 * INLIMBUVLO_x
 *
 * RESET        7   Write 1 to reset all registers to default
 * N/A          6   Not used
 * INLIM_x      5:3 Input Current LImit setting
 *                  INLIM = 50 mA + INLIM_CODE * 50mA
 *
 * BUVLO_x      2:0 000,001,010 = 3V
 *                  011         = 2.8 V
 *                  100         = 2.6 V
 *                  101         = 2.4 V
 *                  110         = 2.2 V
 *                  111         = Disabled
 */

#define INLIMBUVLO_RESET    0x00
#define INLIMBUVLO_INLIM    0x08 // 50 mA default
#define INLIMBUVLO_BUVLO    0x02 // 3V default
#define INLIMBUVLO_DEFAULT  INLIMBUVLO_BUVLO | INLIMBUVLO_INLIM | INLIMBUVLO_RESET
/*
 * Voltage Based Battery Monitor 0x0A
 * VBMOM_x
 *
 * VBMON_READ       Write 1 to initiate a read
 *                  See data sheet Table 23
 */

#define VBMOM_DEFAULT       0x00

/*
 * VIN_DPM and Timers       0x0B
 */

#define VINDPM_DEFAULT      0x4A

Display_Handle display;

/*
 * Initialize Register
 *
 * Write initial register initialize state
 *
 */
int BQ25120_init();

/*
 * Read all registers
 */
int BQ25120_read_regs(uint8_t*);

/*
 *  Write default register
 */
int BQ25120_write_default_regs();


#endif /* DRIVERS_BQ25120_H_ */
