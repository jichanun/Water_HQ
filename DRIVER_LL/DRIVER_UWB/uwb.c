#include "uwb.h" 
//#include "string.h"	 
//#include "nlink_utils.h"

//**********************-----------------------***********************
//�ڲ����ܺ�������

int32_t NLINK_ParseInt24(nint24_t data)
{
  uint8_t *byte = (uint8_t *)(&data);
  return (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
}

uint32_t NLINK_ParseUint24(nuint24_t data)
{
  uint8_t *byte = (uint8_t *)(&data);
  return byte[0] | byte[1] << 8 | byte[2] << 16;
}

uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length)
{
  const uint8_t *byte = (uint8_t *)data;
  uint8_t sum = 0;
  for (size_t i = 0; i < data_length - 1; ++i)
  {
    sum += byte[i];
  }
  return sum == byte[data_length - 1];
}

void NLink_UpdateCheckSum(uint8_t *data, size_t data_length) //����У��λ
{
  uint8_t sum = 0;
  for (size_t i = 0; i < data_length - 1; ++i)
  {
    sum += data[i];
  }
  data[data_length - 1] = sum;
}

size_t NLink_StringToHex(const char *str, uint8_t *out) //HEX�ַ���<->���ͱ����ڴ�ת��
{
  size_t outLength = 0;
  size_t cnt = 0;
  uint8_t high = 0, low = 0;
  uint8_t current = 0;
  uint8_t value = 0;
  uint8_t isHighValid = 0;

	while ((current = str[cnt]) != 0)
  {
    ++cnt;
    if (current >= '0' && current <= '9')
    {
      value = (uint8_t)(current - '0');
    }
    else if (current >= 'a' && current <= 'f')
    {
      value = (uint8_t)(current - 'a' + 10);
    }
    else if (current >= 'A' && current <= 'F')
    {
      value = (uint8_t)(current - 'A' + 10);
    }
    else
    {
      continue;
    }
    if (!isHighValid)
    {
      high = value;
      isHighValid = 1;
    }
    else
    {
      low = value;
      out[outLength] = (uint8_t)(high << 4 | low);
      ++outLength;
      isHighValid = 0;
    }
  }
  return outLength;
}

static uint8_t UnpackData(const uint8_t *data, size_t data_length)
{
	//������鲿��
  if (data_length < g_nlt_tagframe0.fixed_part_size ||
      data[0] != g_nlt_tagframe0.frame_header ||
      data[1] != g_nlt_tagframe0.function_mark)
    return 0;
  if (!NLINK_VerifyCheckSum(data, g_nlt_tagframe0.fixed_part_size))
    return 0;
	//���ݽ��㲿��
  memcpy(&g_frame, data, g_nlt_tagframe0.fixed_part_size);
  g_nlt_tagframe0.result.role = (linktrack_role_e)g_frame.role;
  g_nlt_tagframe0.result.id = g_frame.id;
  g_nlt_tagframe0.result.local_time = g_frame.local_time;
  g_nlt_tagframe0.result.system_time = g_frame.system_time;
  g_nlt_tagframe0.result.voltage = g_frame.voltage / MULTIPLY_VOLTAGE;

  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.pos_3d, g_frame.pos_3d,
                              MULTIPLY_POS)
  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.vel_3d, g_frame.vel_3d,
                              MULTIPLY_VEL)
  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.dis_arr, g_frame.dis_arr,
                              MULTIPLY_DIS)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.imu_gyro_3d, g_frame.imu_gyro_3d,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.imu_acc_3d, g_frame.imu_acc_3d,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.quaternion, g_frame.quaternion,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.angle_3d, g_frame.angle_3d,
                        MULTIPLY_ANGLE)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.eop_3d, g_frame.eop_3d,
                        MULTIPLY_EOP)

  return 1;
}
nlt_tagframe0_t g_nlt_tagframe0 = {.fixed_part_size = 128,
                                   .frame_header = 0x55,
                                   .function_mark = 0x01,
                                   .UnpackData = UnpackData};

								
void Uwb_Get_Data(uint8_t* data,float* Pos_X,float* Pos_Y,float* Pos_Z,
									float* Imu_Pitch,float* Imu_Yaw,float* Imu_Roll,uint8_t* Validation) 											 
{
	size_t data_length;
	data_length = 128;
	*Validation = 0 ; //��Ч������
	if (g_nlt_tagframe0.UnpackData(data, data_length))
	{
		nlt_tagframe0_result_t *result = &g_nlt_tagframe0.result;
		
		*Pos_X = result->pos_3d[0] ;
		*Pos_Y = result->pos_3d[1] ;
		*Pos_Z = result->pos_3d[2] ;
		*Imu_Pitch = result->angle_3d[0] ;
		*Imu_Yaw = result->angle_3d[1] ;
		*Imu_Roll = result->angle_3d[2] ;
		
		if(result->eop_3d[0]<2 && result->eop_3d[1]<2 && result->eop_3d[2]<2) //��Ч����֤
			*Validation = 1 ;
	}
}


//���Ϊ�����������ֽ���ʮ��������������������ǶԱ�ʾ�ڴ�ռ���ַ�����ת����
//		����ܹ����ݴ�С��ģʽ�����𣨻���˵��С�˶������ת��û��ʲôӰ�죩
/*
//���Ի������м����� 
//Code Block for Operating Ability Verification
#pragma pack(1)
typedef struct
{
  uint8_t a;
  uint8_t b;
  uint32_t c;
  double d;
  uint8_t e;
} pack_test_t;
#pragma pack()
int main()
{
	//STM32ΪС��ģʽ��Ӧ������������
  {
    uint32_t check = 1;
    if (*(uint8_t *)(&check) != 1)
    {
      printf("Error: this code must run in little endian.");
      return EXIT_FAILURE;
    }
  }

	//STM32Ӧ���ܹ�����pack����д����Ҳ����ͨ���鿴�ڴ���м���
  if (sizeof(pack_test_t) != 15)
  {
    printf("Error: Pack do not work, pack size:%zu. Contact us for support",
           sizeof(pack_test_t));
    return EXIT_FAILURE;
  }
}
*/
/*
//��ڵ�NodeFrame1���㲿��
//Codes for Multiple Nodes Calculating in NodeFrame1
printf("LinkTrack NodeFrame1 data unpack successfully:\r\n");
printf("id:%d, system_time:%d, valid_node_count:%d\r\n", result->id,  //��Ч�����㲿�֣���ʱ���ã�����һ����Ч��㣩
			 result->system_time, result->valid_node_count);
for (int i = 0; i < result->valid_node_count; ++i)
{
	nlt_nodeframe1_node_t *node = result->nodes[i];
	printf("role:%d, id:%d, x:%f, y:%f\r\n", node->role, node->id,
				 node->pos_3d[0], node->pos_3d[1]);
}
*/
