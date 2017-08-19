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


// Define Memory Addresses
#define ADDR_STATUS_SHIP_MODE 0x00
#define ADDR_FAULTS_FAULT_MASK 0x01
//TODO: More


// Define Register Masks
#define REG_STATUS_STAT1 0b10000000
#define REG_STATUS_STAT2 0b01000000





#endif /* DRIVERS_BQ25120_H_ */
