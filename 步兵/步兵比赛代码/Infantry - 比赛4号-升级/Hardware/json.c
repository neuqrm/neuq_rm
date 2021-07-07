
#include "json.h"

Chassis_Rpm_stc  	Json_Chassis_Rpm		=	{0};
Gimbal_Yaw_stc  	Json_Gimbal_Yaw		=	{0};
Gimbal_Pitch_stc 	Json_Gimbal_Pitch	=	{0};

/*
*JSON�����ж����ݴ����� �������ݴ���
*����� ʹ������ȫ�ֱ���
*Jason_Receive_Flag JSON���ڹ�����־ ���ڹ���״̬���޷���������
*Jason_Data_Flag JSON���ݰ����ͱ�־
********************************************************************/
char Json_Buffer[MAX_LENGTH]; 
uint8_t Jason_Receive_Flag = JASON_STOP,Jason_Data_Flag = 0;
void JASON_Data_Solve(uint8_t ucTemp)
{
	static uint8_t json_count = 0;
	static char ReceiveBuffer[MAX_LENGTH];
	if(ucTemp == '*')	//��ʼ��־λ
	{
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //��ս��ջ�����,��ֹ���û�յ�������־λ��������δ��������
		json_count=0;
		Jason_Receive_Flag = JASON_START;	//��ʼ��־��һ
	}
	else if(ucTemp == ';')	//������־λ
	{
		json_count=0;
		//��־λ��һ����ʶ�յ�����
		Jason_Data_Flag = JASON_GIMBALL_ANGLE;
		Jason_Receive_Flag = JASON_STOP; //������־��һ
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
		strcpy(Json_Buffer,ReceiveBuffer);	//���ջ�������json������
		memset(ReceiveBuffer,    0, sizeof(ReceiveBuffer)); //��ս��ջ�����
	}//********************************************
	else if(ucTemp == '?')	//������־λ
	{
		json_count=0;
		//��־λ��һ����ʶ�յ�����
		Jason_Data_Flag = JASON_GIMBALL_ACTION;
		Jason_Receive_Flag = JASON_STOP; //������־��һ
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
		strcpy(Json_Buffer,ReceiveBuffer);	//���ջ�������json������
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //��ս��ջ�����	
	}//********************************************
	else if(ucTemp == '!')	//������־λ
	{
		json_count=0;
		//��־λ��һ����ʶ�յ�����
		Jason_Data_Flag = JASON_CHASSIC_SPEED;
		Jason_Receive_Flag = JASON_STOP; //������־��һ
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
		strcpy(Json_Buffer,ReceiveBuffer);	//���ջ�������json������
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //��ս��ջ�����
	}//********************************************
	else if(ucTemp == '.')	//������־λ
	{
		json_count=0;
		//��־λ��һ����ʶ�յ�����
		Jason_Data_Flag = JASON_GIMBALL_SHOOT;
		Jason_Receive_Flag = JASON_STOP; //������־��һ
		//printf("%s",receiveBuffer);
		memset(Json_Buffer, 0, sizeof(Json_Buffer));	//��ջ���������ֹ��һ�ε�����ʣ�ർ������
		strcpy(Json_Buffer,ReceiveBuffer);	//���ջ�������json������
		memset(ReceiveBuffer, 0, sizeof(ReceiveBuffer)); //��ս��ջ�����	
	}
	else if(Jason_Receive_Flag == JASON_START)
	{
		ReceiveBuffer[json_count++] = ucTemp;
	}
}

/**
  *@brief ������̨��Ϣ
  */
void Json_Gimball_Angle_Send(void)   
{
  json_t *root;
	char *out;           
	root = json_pack("[{sfsf}[ff]]",\
					  "yaw_angular", (Motion_Gimbal_Yaw.Yaw_rad),\
						"pitch_angular", (Motion_Gimbal_Pitch.Pitch_rad),\
						(Motion_Gimbal_Yaw.Yaw_rad),\
						(Motion_Gimbal_Pitch.Pitch_rad));   //(int),
	out = json_dumps(root, JSON_ENCODE_ANY);
	printf("%s\r\n", out);
	json_decref(root);
	//free(root);
	free(out);
}
/**
  *@brief ������̨�Ƕ�
  */
void Json_Gimball_Angle_Reslove(void)
{
	json_t *root;
	json_t *gimbal_obj;
	json_t *item_obj;
	json_error_t error;
	root = json_loads(Json_Buffer,0,&error);
	gimbal_obj = json_object_get( root, "gimbal" );
	item_obj = json_array_get( gimbal_obj, 0 );
  Json_Gimbal_Yaw.Yaw_rad = 1.0f*json_integer_value(item_obj); //10000;
	item_obj = json_array_get( gimbal_obj, 1 );
	Json_Gimbal_Pitch.Pitch_rad = 1.0f*json_integer_value(item_obj);
	json_decref(item_obj);
	json_decref(gimbal_obj);
	json_decref(root);	
}
