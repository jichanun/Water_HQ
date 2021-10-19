#ifndef __UWB_H
#define __UWB_H	 
#include "main.h"
#include <string.h>
#include "stdlib.h"
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

/******************************************/
/***********在这里修改标签数量*************/
/*********/#define TagNumber 6/************/
/******************************************/


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

/** 
	* Aimed Protocol:Tag_Frame0
	* First Created--2021.5
	* Last Modified--2021.10
	* Specific Mark--T0
	* Coder--DogeYellow
*/
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
/** 
	* Block Ending
	* Specific Mark--T0
*/

/** 
	* Aimed Protocol:Anchor_Frame0
	* First Created--2021.10
	* Last Modified--2021.10
	* Specific Mark--A0
	* Coder--DogeYellow
*/
typedef struct
{
	uint8_t id;
	linktrack_role_e role;
	float pos_3d[3];
	float dis_arr[8];
} nlt_anchorframe0_node_t;

typedef struct
{
	linktrack_role_e role;
	uint8_t id;
	uint32_t local_time;
	uint32_t system_time;
	float voltage;
	uint8_t valid_node_count;
	nlt_anchorframe0_node_t *nodes[30];
} nlt_anchorframe0_result_t;

typedef struct
{
	const size_t fixed_part_size;
	const uint8_t frame_header;
	const uint8_t function_mark;
	const uint8_t tail_check;
	nlt_anchorframe0_result_t result;

	uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
} nlt_anchorframe0_t;

extern nlt_anchorframe0_t nlt_anchorframe0_;
/** 
	* Block Ending
	* Specific Mark--A0
*/

/** 
	* Aimed Protocol:Node_Frame0
	* First Created--2021.10
	* Last Modified--2021.10
	* Specific Mark--N0
	* Coder--DogeYellow
*/
typedef struct 
{
	uint8_t Node_Frame0_Nodes ;
	uint8_t Sender ;
	uint8_t Receiver ;
	uint16_t Data_length ;
	uint8_t* Data ;
		//Data[0]为1表示数据为用户信息；为2表示数据为定位信息
} trans_data_t ;
extern trans_data_t Trans_Data ;

typedef struct 
{
	uint8_t Node_Frame0_Nodes ;
	uint8_t Sender ;
	uint8_t Receiver ;
	uint16_t Data_length ;
	uint8_t* Data ;
		//Data[0]为1表示数据为用户信息；为2表示数据为定位信息
} info_data_t ;
/** 
	* Block Ending
	* Specific Mark--N0
*/

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
//#pragma pack()
	
/** 
	* Aimed Protocol:Tag_Frame0
	* First Created--2021.5
	* Last Modified--2021.10
	* Specific Mark--T0
	* Coder--DogeYellow
*/
//#pragma pack(1)
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
//#pragma pack()
	
static nlt_tagframe0_raw_t g_frame_tag0;
/** 
	* Block Ending
	* Specific Mark--T0
*/

/** 
	* Aimed Protocol:Anchor_Frame0
	* First Created--2021.10
	* Last Modified--2021.10
	* Specific Mark--A0
	* Coder--DogeYellow
*/
//#pragma pack(1)
typedef struct
{
  uint8_t id;
  uint8_t role;
  nint24_t pos_3d[3];
  uint16_t dis_arr[8];
} nlt_anchorframe0_tag_raw_t;

typedef struct
{
  uint8_t header[2];
  nlt_anchorframe0_tag_raw_t nodes[30];
  uint8_t reserved0[67];
  uint32_t local_time;
  uint8_t reserved1[4];
  uint16_t voltage;
  uint32_t system_time;
  uint8_t id;
  uint8_t role;
  uint8_t tail_check;
} nlt_anchorframe0_raw_t;

static nlt_anchorframe0_raw_t g_frame_anchor0;
/** 
	* Block Ending
	* Specific Mark--A0
*/
#pragma pack()

	
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
      return 0;                                                                \
    }                                                                          \
  }
	
//      printf("Memory allocation failed, please increase heap size to support " \
//             "protocol unpack.\r\n");                                          \
//内部功能函数声明
//int32_t NLINK_ParseInt24(nint24_t data);
//uint32_t NLINK_ParseUint24(nuint24_t data);
//uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length);
//void NLink_UpdateCheckSum(uint8_t *data, size_t data_length);
//size_t NLink_StringToHex(const char *str, uint8_t *out);
//static uint8_t UnpackData(const uint8_t *data, size_t data_length) ;

//外部接口声明
#pragma pack(1)
typedef struct
{
	uint8_t ID ;
	uint8_t Validation ;
	//1:正常定位 0：定位错误（包括精度过低或者丢失信号）
	float Pos_X ;
	float Pos_Y ;
	float Pos_Z ;
} Position_Nodes ;
typedef struct
{
	uint8_t Present_Nodes ;
	Position_Nodes Tag[TagNumber] ;
} API_Position ;
#pragma pack()


uint8_t Uwb_Get_Data(uint8_t* data, API_Position* TagsPosition,
								float* Pos_X,float* Pos_Y,float* Pos_Z,uint8_t* Validation,
								uint8_t* Info_Data, uint16_t* Info_Length, uint8_t* Info_Sender ) ;	
//返回值：1表示当前标签坐标更新，2表示全体标签坐标更新，3表示当前标签通讯内容更新
uint8_t Uwb_BroadCast(uint8_t* data, uint8_t* TransData, uint16_t* Length);
	//需要主函数定义一个接收的字符型变量,trnasdata还需要设计一个区分位分开定位信息与用户信息
//返回0，数据解算无效，禁止转发数据；返回1，定位数据广播；返回2，通讯数据广播

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

/*
对于本模块的工作原理做一个简要说明：
1.模块内含几个重要组件：
2.定位信息解包类：这部分包都是定长、包内每个字符都有特定位置的，因此，通过预先设定好的内存严格的模板，
	直接将内容拷贝进入模板（实际上是一个结构体，内部已经划分好了各个变量的内存位置），在检验完没有传输错误后，
	直接提取每个变量位置上的内容。
3.透传数据解包类：此部分非定长信息，但是由于包内带有长度帧，且最大包长不会超过128字节，因此采取先读取包长，
	然后将内容放入指定变量区域内，并且附上信息长度以方便读取使用。
	
对于一些可以注意的信息做一些简要说明：
1.一个将错就错的错误：在移植Nlink系列的范例文档时，由于一开始使用的仅有tagframe0这一个包，因此我将
	内存结构严格的变量g_frame拿到了.h文件中定义。实际上，这一个变量在范例文档中，是定义在各个模块自己的.c文件
	中的，这种做法能够使用同一个名字g_frame来定义不同的包解析，有利于提高程序的易读性。在后期增加针对其他包
	的解析代码时，我才发现我的使用方式（也就是把所有的结构都放在同一个.h文件中定义，所有解析代码放在同一个.c文件中
	定义）不太利于这种模式。在后期移植时，不得不把g_frame全部改为各个包独有的变量名，如g_frame_tag0等.
2.结构体定义中带着“raw”的基本上都是内存结构严格的变量，基本上都是直接把包内容拷贝进去进行解析的模板。
3.WARNING! 程序缺陷：在解读NODE0数据包时，默认包内仅有一个有效信息。事实上，包内可以有多个信息，具体有多少
	信息，在程序中预留了一个变量Node_Frame0_Nodes,这个变量每次解包之后都会记录该包内存有的信息数量，
	如果在使用过程中怀疑因为本条所述原因发生丢包现象，可以查看该变量来判断是否为此原因。如果是，
	修改解包程序，再添加一个能够在该变量大于1时解包其他信息的程序代码就可以了。之所以这样做，是因为大多数情况下
	不会出现这个问题，因此这样的代码可以更加精简。
*/
