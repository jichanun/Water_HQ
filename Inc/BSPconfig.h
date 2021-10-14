/****************************************************
*			Title:		���еײ����ýӿ�
*			ChipType:	STM32F405RGT6
*			Version:	1.0.0
*			Date:			2017.09.14
*												LD.
*****************************************************/
#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H
#include "sys.h"

/****************************************************
*											CAN1�ӿ�											*
****************************************************/

//CAN����������16λidlistģʽ
//0x000�رոù�����
#define CAN1_FILTER_ID_FMI0	(0x201)//�������
#define CAN1_FILTER_ID_FMI1	(0x205)//���ߵ��
#define CAN1_FILTER_ID_FMI2	(0x203)
#define CAN1_FILTER_ID_FMI3	(0x000)

//�����ڶ���
#define FILTER_NUMBER_2_OPEN	(0)
	#define CAN1_FILTER_ID_FMI4	(0x000)
	#define CAN1_FILTER_ID_FMI5	(0x000)
	#define CAN1_FILTER_ID_FMI6	(0x000)
	#define CAN1_FILTER_ID_FMI7	(0x000)

/****************************************************
*											������ʹ�ýӿڣ�ˮ�£�											*
****************************************************/
#define CONFIG_USE_GYROSCOPE 1
/****************************************************
*											�ƽ�����ֵ������ˮ�£�											*
****************************************************/
#define MIDDLE_PWM 630
/****************************************************
*									Number��ˮ�£�											*
****************************************************/
#define SELF_ID 2
/****************************************************
*											CAN2�ӿ�											*
****************************************************/

//CAN����������16λidlistģʽ
//0x000�رոù�����
#define CAN2_FILTER_ID_FMI0	(0x205)//Yaw����
#define CAN2_FILTER_ID_FMI1	(0x206)
#define CAN2_FILTER_ID_FMI2	(0x000)
#define CAN2_FILTER_ID_FMI3	(0x000)


/****************************************************
*											WatchDog�ӿ�										*
****************************************************/
#define CONFIG_USE_IWDG 0
#define CONFIG_USE_WWDG 0


//DRIVER
/****************************************************
*											Chassis�ӿ�										*
****************************************************/
#define	CONFIG_USE_CHASSIS		1

//DRIVER
/****************************************************
*											FeedMotor�ӿ�										*
****************************************************/
#define	CONFIG_USE_FEEDMOTOR		1


//DRIVER
/****************************************************
*											MPU9250�ӿ�										*
****************************************************/
#define	CONFIG_USE_MPU9250		0

#if	CONFIG_USE_MPU9250
		#define CONFIG_USE_MPU9250_DMP	1
#endif


/****************************************************
*											GIMBAL�ӿ�										*
****************************************************/
#define PITCH_INIT_VALUE_SET	(0.0f)
#define YAW_INIT_VALUE_SET	(0.0f)


/****************************************************
*											LostCounter�ӿ�										*
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
*										����ģʽ�ӿ�										*
****************************************************/
#define	DEBUG_USE_GIMBALMOTOR_CANSEND		0
#define DEBUG_USE_CHASSISMOTOR_CANSEND  0
#define DEBUG_USE_PULLMOTOR_CANSEND     0

/****************************************************
*											�ڴ���Խӿ�									*
****************************************************/
//���������ջʹ��������ȰѺ� INCLUDE_uxTaskGetStackHighWaterMark ��1 λ��FreeRTOS.h
//������رմκ꣨�Ϸ�ʱ�䣩
//����ͨ��һ���궨������ȥ
#define CHECKMEMORYTASK	0


/****************************************************
*										�ź����ӿ�										  *
****************************************************/
#define REMOTE_UART_RX_SIGNAL  ( 1 << 0 )



#endif
