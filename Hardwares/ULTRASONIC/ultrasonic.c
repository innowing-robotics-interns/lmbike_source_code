
#include "ultrasonic.h"
#include "imath.h"
#include "tim.h"
#include "cmsis_os.h"
ultra ultrasonic = {0,1,0,0,0};

/*
 * 函数名：UltrasonicWave_StartMeasure
 * 描述  ：开始测距，发送一个>10us的脉冲，然后测量返回的高电平时间
 */
void ultrasonic_startMeasure(void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_SET); 		    //送>10US的高电平
	tdelay_us(20);		                                        			//延时20US
	HAL_GPIO_WritePin(TRIG_GPIO_Port,TRIG_Pin,GPIO_PIN_RESET);
}

/* 超声波外部中断回调函数 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if( 1 == ultrasonic.ultraIsOk ){
		return;																								/* 超声波不在位退出 */
	}
	__HAL_TIM_SET_COUNTER(&htim4,0);												/* 计数清零 */
	HAL_TIM_Base_Start_IT(&htim4);													/* 启动定时器 */
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin)){				/* 等待低电平 */
		ultrasonic.disCounter = __HAL_TIM_GET_COUNTER(&htim4);
		if( ultrasonic.disCounter > 1470 ) {  								/* 最远测量控制在1470计数值，换为距离为25cm */
			break;
		}
	}
	HAL_TIM_Base_Stop_IT(&htim4);
	ultrasonic.distance = __HAL_TIM_GET_COUNTER(&htim4) / 1000000.0f * 340.0f / 2.0f * 100.0f;	//us->s   声速340m/s  最终转为厘米cm	
}

/* 超声波自检是否正常标志，0模块正常，1模块未插上 */
uint8_t ultraCheck(void)
{
	static uint16_t ultraTicks = 0;
	if( ultrasonic.distance == 0.0F ) {
		ultraTicks++;
		if(ultraTicks > 100 ) {
			ultraTicks = 0;
			ultrasonic.ultraIsOk = 1;    
		}
	}
	else {
		ultrasonic.ultraIsOk = 0;
	}    
	return ultrasonic.ultraIsOk;
}

/****************** (C) COPYRIGHT Mr.Lin @ 源动力科技 *********END OF FILE****/
