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


/** 
	* Aimed Protocol:Tag_Frame0
	* First Created--2021.5
	* Last Modified--2021.10
	* Specific Mark--T0
	* Coder--DogeYellow
*/
static uint8_t UnpackData_Tag0(const uint8_t *data, size_t data_length)
{
	//������鲿��
  if (data_length < g_nlt_tagframe0.fixed_part_size ||
      data[0] != g_nlt_tagframe0.frame_header ||
      data[1] != g_nlt_tagframe0.function_mark)
    return 0;
  if (!NLINK_VerifyCheckSum(data, g_nlt_tagframe0.fixed_part_size))
    return 0;
	//���ݽ��㲿��
  memcpy(&g_frame_tag0, data, g_nlt_tagframe0.fixed_part_size);
  g_nlt_tagframe0.result.role = (linktrack_role_e)g_frame_tag0.role;
  g_nlt_tagframe0.result.id = g_frame_tag0.id;
  g_nlt_tagframe0.result.local_time = g_frame_tag0.local_time;
  g_nlt_tagframe0.result.system_time = g_frame_tag0.system_time;
  g_nlt_tagframe0.result.voltage = g_frame_tag0.voltage / MULTIPLY_VOLTAGE;

  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.pos_3d, g_frame_tag0.pos_3d,
                              MULTIPLY_POS)
  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.vel_3d, g_frame_tag0.vel_3d,
                              MULTIPLY_VEL)
  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.dis_arr, g_frame_tag0.dis_arr,
                              MULTIPLY_DIS)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.imu_gyro_3d, g_frame_tag0.imu_gyro_3d,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.imu_acc_3d, g_frame_tag0.imu_acc_3d,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.quaternion, g_frame_tag0.quaternion,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.angle_3d, g_frame_tag0.angle_3d,
                        MULTIPLY_ANGLE)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.eop_3d, g_frame_tag0.eop_3d,
                        MULTIPLY_EOP)

  return 1;
}
nlt_tagframe0_t g_nlt_tagframe0 = {.fixed_part_size = 128,
                                   .frame_header = 0x55,
                                   .function_mark = 0x01,
                                   .UnpackData = UnpackData_Tag0};
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
static uint8_t UnpackData_Anchor0(const uint8_t *data, size_t data_length)
{
  if (data_length < nlt_anchorframe0_.fixed_part_size ||
      data[0] != nlt_anchorframe0_.frame_header ||
      data[1] != nlt_anchorframe0_.function_mark ||
      data[nlt_anchorframe0_.fixed_part_size - 1] !=
          nlt_anchorframe0_.tail_check)
    return 0;

  static uint8_t init_needed = 1;
  if (init_needed)
  {
    memset(nlt_anchorframe0_.result.nodes, 0,
           sizeof(nlt_anchorframe0_.result.nodes));
    init_needed = 0;
  }

  memcpy(&g_frame_anchor0, data, nlt_anchorframe0_.fixed_part_size);
  nlt_anchorframe0_.result.role = (linktrack_role_e)g_frame_anchor0.role;
  nlt_anchorframe0_.result.id = g_frame_anchor0.id;
  nlt_anchorframe0_.result.local_time = g_frame_anchor0.local_time;
  nlt_anchorframe0_.result.system_time = g_frame_anchor0.system_time;
  nlt_anchorframe0_.result.voltage = g_frame_anchor0.voltage / MULTIPLY_VOLTAGE;

  nlt_anchorframe0_.result.valid_node_count = 0;
  for (size_t i = 0, count = ARRAY_LENGTH(nlt_anchorframe0_.result.nodes);
       i < count; ++i)
  {
    if (g_frame_anchor0.nodes[i].id == 0xff)
      continue;

    uint8_t index = nlt_anchorframe0_.result.valid_node_count;
    TRY_MALLOC_NEW_NODE(nlt_anchorframe0_.result.nodes[index],
                        nlt_anchorframe0_node_t)

    nlt_anchorframe0_.result.nodes[index]->role =
        (linktrack_role_e)g_frame_anchor0.nodes[i].role;
    nlt_anchorframe0_.result.nodes[index]->id = g_frame_anchor0.nodes[i].id;
    NLINK_TRANSFORM_ARRAY_INT24(nlt_anchorframe0_.result.nodes[index]->pos_3d,
                                g_frame_anchor0.nodes[i].pos_3d, MULTIPLY_POS)
    NLINK_TRANSFORM_ARRAY(nlt_anchorframe0_.result.nodes[index]->dis_arr,
                          g_frame_anchor0.nodes[i].dis_arr, 100.0f)

    ++nlt_anchorframe0_.result.valid_node_count;
  }
  return 1;
}

nlt_anchorframe0_t nlt_anchorframe0_ = {.fixed_part_size = 896,
                                        .frame_header = 0x55,
                                        .function_mark = 0x00,
                                        .tail_check = 0xee,
                                        .UnpackData = UnpackData_Anchor0};																	 
		 
/** 
	* Block Ending
	* Specific Mark--A0
*/
					
info_data_t Info_DATA ;
uint8_t Uwb_Get_Data(uint8_t* data, API_Position* TagsPosition, uint8_t* ID,
								float* Pos_X,float* Pos_Y,float* Pos_Z,uint8_t* Validation,
								uint8_t* Info_Data, uint16_t* Info_Length, uint8_t* Info_Sender)												
{
	size_t data_length;
	
	switch (data[1]){
	case 1:{  //Analysis for Tag_Frame0
		data_length = 128;
		*Validation = 0 ; //��Ч������
		if (g_nlt_tagframe0.UnpackData(data, data_length)) //��λ��Ϣ��Ч
		{
			nlt_tagframe0_result_t *result = &g_nlt_tagframe0.result;
			
			*ID = result->id ;
			*Pos_X = result->pos_3d[0] ;
			*Pos_Y = result->pos_3d[1] ;
			*Pos_Z = result->pos_3d[2] ;
			
			if(result->eop_3d[0]<2 && result->eop_3d[1]<2 && result->eop_3d[2]<2) //��Ч����֤
				*Validation = 1 ;
			return 0x01 ;
		}
		break ; }
	case 2:{   //Analysis for Node_Frame0
		data_length = data[3] << 8 | data[2] ;

		if(NLINK_VerifyCheckSum(data, data_length)) //���հ���Ч
		{
			if(data[15]==0x01)  //�����û���Ϣ
			{
				Info_DATA.Sender = data[16] ;
					//��ǩ���յ������ݷ����߲���ID��Ӧ�ı�ǩ���������ݶ��ڵڶ�λ��Ҳ����data[16]��
				Info_DATA.Receiver = NULL ;
				Info_DATA.Data_length = (data[13] | data[14]<<8) ;
				Info_DATA.Data = Info_Data ;
				Info_DATA.Node_Frame0_Nodes = data[10] ;
				memcpy(Info_DATA.Data, data+17, Info_DATA.Data_length-2) ;
				*Info_Length = Info_DATA.Data_length-2 ;
				*Info_Sender = Info_DATA.Sender ;
				return 0x03 ;
			}
			else if(data[15]==0x02) //����λ�㲥��Ϣ
			{
				memcpy(TagsPosition, data+16, 1+14*TagNumber) ;
				return 0x02 ;
			}
		}
		break ; }
	}
	return 0 ;
}


trans_data_t Trans_Data ;
//API_Position Position_Data ; //��Ȼ��API�ӿڽṹ�壬����ֻ���ڲ�ʹ��,�������ڴ�ת��
uint8_t Uwb_BroadCast(uint8_t* data, uint8_t* TransData, uint16_t* Length, //For Transfering
										API_Position* TagsPosition,  //For Positioning
										uint8_t* Info_Data, uint16_t* Info_Length, uint8_t* Info_Sender) //For Info
{
	size_t data_length;
	
	switch (data[1]){
	case 0:{  //Analysis for Anchor_Frame0
		data_length = 896;
		if (nlt_anchorframe0_.UnpackData(data, data_length))
		{
			nlt_anchorframe0_result_t *result = &nlt_anchorframe0_.result;
			TagsPosition->Present_Nodes = result->valid_node_count ;
			for(uint8_t i=0; i<TagsPosition->Present_Nodes; i++) //��Ч�ڵ�
			{
				TagsPosition->Tag[i].ID = result->nodes[i]->id ;
				TagsPosition->Tag[i].Pos_X = result->nodes[i]->pos_3d[0] ;
				TagsPosition->Tag[i].Pos_Y = result->nodes[i]->pos_3d[1] ;
				TagsPosition->Tag[i].Pos_Z = result->nodes[i]->pos_3d[2] ;
				TagsPosition->Tag[i].Validation = 0x01 ;
			}
			for(uint8_t i=TagsPosition->Present_Nodes; i<TagNumber; i++) //��Ч�ڵ�
			{
				TagsPosition->Tag[i].Validation = 0x00 ;
			}
			Trans_Data.Data = TransData ;
			Trans_Data.Data_length = 2+14*TagNumber ;
			*Trans_Data.Data = 0x02 ; //������Ϣ��Դ�ڶ�λ�㲥��Ϣ
			memcpy(Trans_Data.Data+1, TagsPosition, Trans_Data.Data_length-1) ;
			*Length = Trans_Data.Data_length ;
			
			return 1 ;
		}
		else //�����������
		{
			for(uint8_t i=0; i<TagNumber; i++)
			{
				TagsPosition->Tag[i].Validation = 0x00 ;
			}
		}
		
		break ; }
	case 2:{  //Analysis for Node_Frame0
		data_length = data[3] << 8 | data[2] ;

		if(NLINK_VerifyCheckSum(data, data_length))
		{
			Trans_Data.Sender = data[12] ; //��վ���յ������ݷ�����һ����ID��Ӧ�ı�ǩ
			Trans_Data.Receiver = NULL ;
			Trans_Data.Data_length = (data[13] | data[14]<<8) +2;
			Trans_Data.Data = TransData ; //ָ�붨��
			*Trans_Data.Data = 0x01 ; //������Ϣ��Դ���û���Ϣ
			*(Trans_Data.Data+1) = Trans_Data.Sender ;
			memcpy(Trans_Data.Data+2, data+15, Trans_Data.Data_length-2) ;
			Trans_Data.Node_Frame0_Nodes = data[10] ;
			*Length = Trans_Data.Data_length ;
			
			*Info_Sender = Trans_Data.Sender ;
			*Info_Length = Trans_Data.Data_length-2 ;
			memcpy(Info_Data, data+15, Trans_Data.Data_length-2) ;
			
			return 2 ;
		}
		else  //���ֽ������
		{
		}
		break ; }
	case 3:{  //Analysis for Anchor Command
		data_length = data[0] ;
		
		*TransData = 0x01 ; //�����û���Ϣ����;��
		TransData[1] = 0x10 ; //��վͳһID
		memcpy(TransData+2, data+2, data_length) ;
		*Length = data_length+2 ;
		
		return 3 ;
//		break ;}
	}
	default:
		break ;
	}
		return 0 ;
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
