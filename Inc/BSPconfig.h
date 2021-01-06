/****************************************************
*			Title:		所有底层配置接口
*			ChipType:	STM32F405RGT6
*			Version:	1.0.0
*			Date:			2017.09.14
*												LD.
*****************************************************/
#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H
#include "sys.h"

/****************************************************
*											CAN1接口											*
****************************************************/

//CAN过滤器采用16位idlist模式
//0x000关闭该过滤器
#define CAN1_FILTER_ID_FMI0	(0x201)//拨弹电机
#define CAN1_FILTER_ID_FMI1	(0x205)//拉线电机
#define CAN1_FILTER_ID_FMI2	(0x203)
#define CAN1_FILTER_ID_FMI3	(0x000)

//开启第二个
#define FILTER_NUMBER_2_OPEN	(0)
	#define CAN1_FILTER_ID_FMI4	(0x000)
	#define CAN1_FILTER_ID_FMI5	(0x000)
	#define CAN1_FILTER_ID_FMI6	(0x000)
	#define CAN1_FILTER_ID_FMI7	(0x000)



/****************************************************
*											CAN2接口											*
****************************************************/

//CAN过滤器采用16位idlist模式
//0x000关闭该过滤器
#define CAN2_FILTER_ID_FMI0	(0x205)//Yaw轴电机
#define CAN2_FILTER_ID_FMI1	(0x206)
#define CAN2_FILTER_ID_FMI2	(0x000)
#define CAN2_FILTER_ID_FMI3	(0x000)


/****************************************************
*											WatchDog接口										*
****************************************************/
#define CONFIG_USE_IWDG 0
#define CONFIG_USE_WWDG 0


//DRIVER
/****************************************************
*											Chassis接口										*
****************************************************/
#define	CONFIG_USE_CHASSIS		1

//DRIVER
/****************************************************
*											FeedMotor接口										*
****************************************************/
#define	CONFIG_USE_FEEDMOTOR		1


//DRIVER
/****************************************************
*											MPU9250接口										*
****************************************************/
#define	CONFIG_USE_MPU9250		0

#if	CONFIG_USE_MPU9250
		#define CONFIG_USE_MPU9250_DMP	1
#endif


/****************************************************
*											GIMBAL接口										*
****************************************************/
#define PITCH_INIT_VALUE_SET	(0.50f)
#define YAW_INIT_VALUE_SET	(0.850f)


/****************************************************
*											LostCounter接口										*
****************************************************/
#define	CONFIG_USE_LOSTCOUNTER		1

#if	CONFIG_USE_LOSTCOUNTER
	#define NUMBERS_OF_COUNT		10
	
	#define	CHASSIS_MOTOR_0					0
	#define CHASSIS_MOTOR_1					1
	#define	CHASSIS_MOTOR_2					2
	#define	CHASSIS_MOTOR_3					3
	#define	GIMBAL_MOTOR_PITCH			4
	#define	GIMBAL_MOTOR_YAW				5
	#define FEEDMOTOR_LOST_COUNT 		6
	#define	REMOTE_LOST_COUNT				7
	#define	VISION_LOST_COUNT				8
	#define	JUDGEMENT_LOST_COUNT		9
	
	#define CHASSIS_LOST_TOLERANCE_MS			200
	#define GIMBAL_LOST_TOLERANCE_MS			200
	#define REMOTE_LOST_TOLERANCE_MS			200
	#define VISION_LOST_TOLERANCE_MS			200
	#define JUDGEMENT_LOST_TOLERANCE_MS		200
	#define FEEDMOTOR_LOST_TOLERANCE_MS		200
	
	
#endif
/****************************************************
*										调试模式接口										*
****************************************************/
#define	DEBUG_USE_GIMBALMOTOR_CANSEND		1
#define DEBUG_USE_CHASSISMOTOR_CANSEND  0
#define DEBUG_USE_PULLMOTOR_CANSEND     1

/****************************************************
*											内存测试接口									*
****************************************************/
//测试任务堆栈使用情况首先把宏 INCLUDE_uxTaskGetStackHighWaterMark 置1 位于FreeRTOS.h
//测试完关闭次宏（较费时间）
//这里通过一个宏定义引过去
#define CHECKMEMORYTASK	0


/****************************************************
*										信号量接口										  *
****************************************************/
#define REMOTE_UART_RX_SIGNAL  ( 1 << 0 )



#endif
