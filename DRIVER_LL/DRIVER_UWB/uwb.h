#ifndef __UWB_H
#define __UWB_H	 
#include "main.h"
#include <string.h>
#include "sys.h"
//#include <stddef.h>
//#include <stdint.h>
//#include <stdio.h>
//#include <stdlib.h>

//C艹编译器兼容
#ifdef __cplusplus  
extern "C"
{
#endif

//宏定义
#define MULTIPLY_VOLTAGE 1000.0f
#define MULTIPLY_POS 1000.0f
#define MULTIPLY_DIS 1000.0f
#define MULTIPLY_VEL 10000.0f
#define MULTIPLY_ANGLE 100.0f
#define MULTIPLY_RSSI -2.0f
#define MULTIPLY_EOP 100.0f

typedef struct
{
	float x;
	float y;
	float z;
	float pitch;
	float yaw;
	float roll;
	u8 status;
	u8 validp;
	float init_x;
	float init_y;
}UWBStruct;

//非内存严格结构定义
typedef enum
{
	LINKTRACK_ROLE_NODE,
	LINKTRACK_ROLE_ANCHOR,
	LINKTRACK_ROLE_TAG,
	LINKTRACK_ROLE_CONSOLE,
	LINKTRACK_ROLE_DT_MASTER,
	LINKTRACK_ROLE_DT_SLAVE,
	LINKTRACK_ROLE_MONITOR,
} linktrack_role_e;
typedef uint32_t id_t;

typedef struct //被进一步嵌套进入下一个部分
{
	linktrack_role_e role;
	uint8_t id;
	float pos_3d[3];
	float eop_3d[3];
	float vel_3d[3];
	float dis_arr[8];
	float imu_gyro_3d[3];
	float imu_acc_3d[3];
	float angle_3d[3];
	float quaternion[4];
	uint32_t local_time;
	uint32_t system_time;
	float voltage;
} nlt_tagframe0_result_t;

typedef struct
{
	const size_t fixed_part_size;
	const uint8_t frame_header;
	const uint8_t function_mark;
	nlt_tagframe0_result_t result;

	uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
} nlt_tagframe0_t;

extern nlt_tagframe0_t g_nlt_tagframe0;

//内存严格结构宏定义
#pragma pack(1)
  typedef struct
  {
    uint8_t byteArray[3];
  } nint24_t;

  typedef struct
  {
    uint8_t byteArray[3];
  } nuint24_t;
#pragma pack()
#pragma pack(1)
	typedef struct
	{
		uint8_t header[2];
		uint8_t id;
		uint8_t role;
		nint24_t pos_3d[3];
		nint24_t vel_3d[3];
		nint24_t dis_arr[8];
		float imu_gyro_3d[3];
		float imu_acc_3d[3];
		uint8_t reserved1[12];
		int16_t angle_3d[3];
		float quaternion[4];
		uint8_t reserved2[4];
		uint32_t local_time;
		uint32_t system_time;
		uint8_t reserved3[1];
		uint8_t eop_3d[3];
		uint16_t voltage;
		uint8_t reserved4[5];
		uint8_t check_sum;
	} nlt_tagframe0_raw_t;
#pragma pack()
	
static nlt_tagframe0_raw_t g_frame;

//算法宏定义
#define ARRAY_LENGTH(X) (sizeof(X) / sizeof(X[0]))

#define NLINK_PROTOCOL_LENGTH(X) ((size_t)(X[2] | X[3] << 8))

#define NLINK_TRANSFORM_ARRAY(DEST, SRC, MULTIPLY)                             \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT)           \
  {                                                                            \
    DEST[_CNT] = SRC[_CNT] / MULTIPLY;                                         \
  }

#define NLINK_TRANSFORM_ARRAY_INT24(DEST, SRC, MULTIPLY)                       \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT)           \
  {                                                                            \
    DEST[_CNT] = NLINK_ParseInt24(SRC[_CNT]) / MULTIPLY;                       \
  }

#define TRY_MALLOC_NEW_NODE(NODE_POINTER, NODE_TYPE)                           \
  if (!NODE_POINTER)                                                           \
  {                                                                            \
    void *p = malloc(sizeof(NODE_TYPE));                                       \
    if (p != NULL)                                                             \
    {                                                                          \
      NODE_POINTER = (NODE_TYPE *)p;                                           \
    }                                                                          \
    else                                                                       \
    {                                                                          \
      printf("Memory allocation failed, please increase heap size to support " \
             "protocol unpack.\r\n");                                          \
      return 0;                                                                \
    }                                                                          \
  }

//内部功能函数声明
//int32_t NLINK_ParseInt24(nint24_t data);
//uint32_t NLINK_ParseUint24(nuint24_t data);
//uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length);
//void NLink_UpdateCheckSum(uint8_t *data, size_t data_length);
//size_t NLink_StringToHex(const char *str, uint8_t *out);
//static uint8_t UnpackData(const uint8_t *data, size_t data_length) ;

//外部接口声明
void Uwb_Get_Data(uint8_t* data,float* Pos_X,float* Pos_Y,float* Pos_Z,
									float* Imu_Pitch,float* Imu_Yaw,float* Imu_Roll,uint8_t* Validation) 	;

#ifdef __cplusplus  //C艹兼容
}
#endif

#endif

/*
// Macro to define packed structures
#ifndef _MSC_VER
#define #pragma pack(1)
 __Declaration__) __Declaration__ __attribute__((packed))
#else
#define #pragma pack(1)
 __Declaration__) \
    __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#endif
*/
