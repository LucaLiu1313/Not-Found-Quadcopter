#ifndef __HMC5883L_H
#define __HMC5883L_H

#include <stdint.h>

void HMC5883L_WriteReg(uint8_t RegAddress,uint8_t Data);

uint8_t HMC5883L_ReadReg(uint8_t RegAddress);

void HMC5883L_Init(void);

void MPU_get_HMC(void);


void HMC5883L_GetData(int16_t * magn_x_gs,int16_t *magn_y_gs,int16_t *magn_z_gs);

void HMC5883L_GetData1(int16_t * magn_x_gs,int16_t *magn_y_gs,int16_t *magn_z_gs);

#endif
