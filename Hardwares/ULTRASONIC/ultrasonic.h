/**
  ******************************************************************************
  * @file    ultrasonic.h
  * @author  Mr.Lin
  * @version V21.03.03
  * @date    03-Mar-2021
  * @brief   
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 源动力科技</center></h2>
  *
  ******************************************************************************
  */

#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "main.h"

typedef struct
{
	float distance;				/* 距离 */
	uint8_t ultraIsOk;		/* 超声波是否在位 */
	uint16_t disCounter;	
	uint8_t enable;
	uint32_t ticks;
}ultra;

void ultrasonic_startMeasure(void);
uint8_t ultraCheck(void);
void get_distance(void);

extern ultra ultrasonic;

#endif /* __ULTRASONIC_H */
/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
