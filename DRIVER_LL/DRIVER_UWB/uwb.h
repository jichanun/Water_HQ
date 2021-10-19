#ifndef __UWB_H
#define __UWB_H	 
#include "main.h"
#include <string.h>
#include "stdlib.h"
//#include <stddef.h>
//#include <stdint.h>
//#include <stdio.h>
//#include <stdlib.h>

//Cܳ����������
#ifdef __cplusplus  
extern "C"
{
#endif

//�궨��
#define MULTIPLY_VOLTAGE 1000.0f
#define MULTIPLY_POS 1000.0f
#define MULTIPLY_DIS 1000.0f
#define MULTIPLY_VEL 10000.0f
#define MULTIPLY_ANGLE 100.0f
#define MULTIPLY_RSSI -2.0f
#define MULTIPLY_EOP 100.0f

/******************************************/
/***********�������޸ı�ǩ����*************/
/*********/#define TagNumber 6/************/
/******************************************/


//���ڴ��ϸ�ṹ����
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
typedef struct //����һ��Ƕ�׽�����һ������
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
		//Data[0]Ϊ1��ʾ����Ϊ�û���Ϣ��Ϊ2��ʾ����Ϊ��λ��Ϣ
} trans_data_t ;
extern trans_data_t Trans_Data ;

typedef struct 
{
	uint8_t Node_Frame0_Nodes ;
	uint8_t Sender ;
	uint8_t Receiver ;
	uint16_t Data_length ;
	uint8_t* Data ;
		//Data[0]Ϊ1��ʾ����Ϊ�û���Ϣ��Ϊ2��ʾ����Ϊ��λ��Ϣ
} info_data_t ;
/** 
	* Block Ending
	* Specific Mark--N0
*/

//�ڴ��ϸ�ṹ�궨��
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

	
//�㷨�궨��
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
//�ڲ����ܺ�������
//int32_t NLINK_ParseInt24(nint24_t data);
//uint32_t NLINK_ParseUint24(nuint24_t data);
//uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length);
//void NLink_UpdateCheckSum(uint8_t *data, size_t data_length);
//size_t NLink_StringToHex(const char *str, uint8_t *out);
//static uint8_t UnpackData(const uint8_t *data, size_t data_length) ;

//�ⲿ�ӿ�����
#pragma pack(1)
typedef struct
{
	uint8_t ID ;
	uint8_t Validation ;
	//1:������λ 0����λ���󣨰������ȹ��ͻ��߶�ʧ�źţ�
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
//����ֵ��1��ʾ��ǰ��ǩ������£�2��ʾȫ���ǩ������£�3��ʾ��ǰ��ǩͨѶ���ݸ���
uint8_t Uwb_BroadCast(uint8_t* data, uint8_t* TransData, uint16_t* Length);
	//��Ҫ����������һ�����յ��ַ��ͱ���,trnasdata����Ҫ���һ������λ�ֿ���λ��Ϣ���û���Ϣ
//����0�����ݽ�����Ч����ֹת�����ݣ�����1����λ���ݹ㲥������2��ͨѶ���ݹ㲥

#ifdef __cplusplus  //Cܳ����
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
���ڱ�ģ��Ĺ���ԭ����һ����Ҫ˵����
1.ģ���ں�������Ҫ�����
2.��λ��Ϣ����ࣺ�ⲿ�ְ����Ƕ���������ÿ���ַ������ض�λ�õģ���ˣ�ͨ��Ԥ���趨�õ��ڴ��ϸ��ģ�壬
	ֱ�ӽ����ݿ�������ģ�壨ʵ������һ���ṹ�壬�ڲ��Ѿ����ֺ��˸����������ڴ�λ�ã����ڼ�����û�д�������
	ֱ����ȡÿ������λ���ϵ����ݡ�
3.͸�����ݽ���ࣺ�˲��ַǶ�����Ϣ���������ڰ��ڴ��г���֡�������������ᳬ��128�ֽڣ���˲�ȡ�ȶ�ȡ������
	Ȼ�����ݷ���ָ�����������ڣ����Ҹ�����Ϣ�����Է����ȡʹ�á�
	
����һЩ����ע�����Ϣ��һЩ��Ҫ˵����
1.һ������ʹ�Ĵ�������ֲNlinkϵ�еķ����ĵ�ʱ������һ��ʼʹ�õĽ���tagframe0��һ����������ҽ�
	�ڴ�ṹ�ϸ�ı���g_frame�õ���.h�ļ��ж��塣ʵ���ϣ���һ�������ڷ����ĵ��У��Ƕ����ڸ���ģ���Լ���.c�ļ�
	�еģ����������ܹ�ʹ��ͬһ������g_frame�����岻ͬ�İ���������������߳�����׶��ԡ��ں����������������
	�Ľ�������ʱ���Ҳŷ����ҵ�ʹ�÷�ʽ��Ҳ���ǰ����еĽṹ������ͬһ��.h�ļ��ж��壬���н����������ͬһ��.c�ļ���
	���壩��̫��������ģʽ���ں�����ֲʱ�����ò���g_frameȫ����Ϊ���������еı���������g_frame_tag0��.
2.�ṹ�嶨���д��š�raw���Ļ����϶����ڴ�ṹ�ϸ�ı����������϶���ֱ�ӰѰ����ݿ�����ȥ���н�����ģ�塣
3.WARNING! ����ȱ�ݣ��ڽ��NODE0���ݰ�ʱ��Ĭ�ϰ��ڽ���һ����Ч��Ϣ����ʵ�ϣ����ڿ����ж����Ϣ�������ж���
	��Ϣ���ڳ�����Ԥ����һ������Node_Frame0_Nodes,�������ÿ�ν��֮�󶼻��¼�ð��ڴ��е���Ϣ������
	�����ʹ�ù����л�����Ϊ��������ԭ�����������󣬿��Բ鿴�ñ������ж��Ƿ�Ϊ��ԭ������ǣ�
	�޸Ľ�����������һ���ܹ��ڸñ�������1ʱ���������Ϣ�ĳ������Ϳ����ˡ�֮����������������Ϊ����������
	�������������⣬��������Ĵ�����Ը��Ӿ���
*/
