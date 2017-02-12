/*
ע������
0������10mm����ĵ����������ʱ����Ϊ600
1��������һ��ȫ�ֱ�������ͳ�Ƶ����λ�ã��ӳ�ʼ��֮ǰ�Ϳ�ʼͳ�ƣ���ֹ��ʼ��ײ���򴫸���ʧЧ
2����ͣ�Ӵ�֮�󣬱�����һ����Ƭ����λ��ʼ���Ĺ��̣��Ǵ�0��ʼ������ֹ�źŵļ�������ײ���Լ���ԭ���˶�
*/
//���˳�ʼ��Ϊ 0 �ĳ��������⣬�����ı����������ڴ˴����������
#include <STC89C5xRC.H>																																			//52��Ƭ��ͷ�ļ�
#include <stdio.h>																																					//����������
#include <math.h>
#include <intrins.h>
#define uchar unsigned char
#define uint  unsigned int

//����
sbit k1 = P2^0;																																							//ԭ���⣬�����⣬�˿ڳ����Ϊ�ߣ�����δ�ڵ������Ϊ��
sbit ori = P2^1;																																						//�¼��޼��
sbit k3 = P2^2;																																							//�ϼ��޼��
sbit flag_out = P2^3;																																				//������
sbit flag_in  = P2^4;																																				//������

//�����صı���
int p_imp;																																									//�����λ��
int a_imp;																																									//��ʱ����
#define speed_max 8									//��Ҫע����ĵĳ���																		//���������ٶ�
#define speed_start 120							//��Ҫע����ĵĳ���																		//�����ٶ�
#define step  601 									//��Ҫע����ĵĳ���																		//��������������Ҫƥ��
int speed_now;																																							//��ǰ�ٶȵı���

//��߱���
#define floor_max 50								//��Ҫע����ĵĳ���																		//��������������1��ʼ���Ӷ��㿪ʼ��
uchar idata floor_now;																																			//��ǰ��� 
uchar idata floor_nex;
long  idata floor_t;																																				//��Ϊ��������з��ŵģ������ֱȽϴ������ó�����
uchar idata floor_full;																																			//������־����-0 ��-1 ��-2

//������
uchar idata flag_ware_i = 0;																																//�������д��������Ĳ���
uchar idata flag_sare_j = 0;																																//�������д��������Ĳ���
uchar idata ware_con[floor_max];																														//���ڴ��ÿ�������Ƿ��а壬0-�ް壬1-�а�
uchar idata ware_opt[floor_max];																														//���ڼ�¼�����а��ӵ����ȼ�
uchar idata ware_opt_num;																																		//���ڼ�¼���ȼ��������

//״̬����
idata bit flag_iao;																																					//���ڱ���Ƿ���Ҫ��壬0-������ȣ�1-�������ȡ��趨�õ����ȼ�����ֹͣʹ��
idata bit flag_o = 1;																																				//ԭ���־��0-ֻ���������˶���1-���������˶�
idata bit flag_pull = 1;																																		//������ɱ�־��0-��ɣ�1-δ���
idata bit flag_push = 1;																																		//�ư���ɱ�־��0-��ɣ�1-δ���
idata bit flag_safe = 1;																																		//��ȫ��־��ÿ�ζ���ǰ���һ�δ˱�־����ѯ�ķ����������ж�
idata bit flag_up = 1;																																			//�ϰ���ɱ�־��0-��ɣ�1-δ���
idata bit flag_down = 1;																																		//�°���ɱ�־��0-��ɣ�1-δ���
idata bit flag_t = 1;																																				//��Ϣ������ɱ�־��0-��ɣ�1-δ���
idata bit flag_r = 1;																																				//��Ϣ������ɱ�־��0-��ɣ�1-δ���
idata bit flag_top = 1;																																			//�ϼ���
idata bit flag_bottom = 1;																																	//�¼���
idata bit flag_ot = 1;																																			//��ʱ��־

void delay_us(long idata time_us)																														//��ʱ��������ʵ��ֵ���ۣ���ʱʱ��Ϊ t*10 + 18 us 
{
	while(time_us--);      
}

void delay_ms(long idata time_ms)
{
	int time_ms_add;
	while(time_ms--)
	{
		time_ms_add = 21;
		while(time_ms_add--);
	}
}

uchar safe()																																								//��ȫ���
{
	flag_safe=0;
	if (flag_o==1) flag_safe=1; else P00 = 0;
	if (flag_pull==1) flag_safe=1; else P01 = 0;
	if (flag_push==1) flag_safe=1; else P02 = 0;
	if (flag_up==1) flag_safe=1; else P03 = 0;
	if (flag_down==1) flag_safe=1; else P04 = 0;
	if (flag_top==1) flag_safe=1; else P05 = 0;
	if (flag_bottom==1) flag_safe=1; else P06 = 0;
	return flag_safe;
}

void upanddown(floor_nex)																																		//���°庯��������Ŀ������
{
	long idata length_run;
	uint idata dir ;
	//if (safe()==1 && flag_ware==1)
	//{
		delay_ms(300);
		speed_now = speed_start;																																//���ٶȸ���ֵ
		length_run = floor_nex-floor_now;																												//��Ҫ�˶��Ĳ��� = Ŀ�Ĳ�� - ��ǰ���
		floor_t = length_run * step;																														//����ĳ��������˶�10mm����ĵ����ת���������ֵ�͵�������������й�
		if (floor_t > 0){dir = 0x05;}else{dir = 0x01;}																					//floor_tΪ���������˶���floor_tΪ���������˶�
		floor_t = abs(floor_t);																																	//ȡ����ֵ
		for (length_run = 0;length_run < floor_t;length_run++)
		{
			P1 = 0x00;
			delay_us(speed_now);
			P1 = dir;																																							//��Ҫ�˶��ķ�����dir��������
			delay_us(speed_now);
			if ((floor_t-length_run)>100){if(speed_now > speed_max)speed_now--;}else speed_now++;	//��ɼ��ٺͼ��ٵĹ��ܣ��������100��
		}
		floor_now = floor_nex;																																	//�˶���ɺ󣬵�ǰ��� = Ŀ�Ĳ��
	//}
}
uint init()																																									//��ʼ������
{
	long idata init_con;
	speed_now = speed_start;
	upanddown(2);																																							//�������˶����㣬��������ԭ��
	delay_ms(1000);
	while(ori==0)																																							//�ҵ�ԭ���Ժ����ѭ��
	{
		P1=0x00;
		delay_us(speed_now);
		P1=0x01;																																							//�����˶�
		delay_us(speed_now);
		a_imp++;
		if (speed_now > speed_max) speed_now--;
	}
	for(init_con=0;init_con<100;init_con++){P1=0x01;delay_us(speed_now++);P1=0x00;delay_us(speed_now);}//�ҵ�ԭ���Ժ����
	return 1;
}

uchar opt_out()
{
	long idata count;
	uchar opt_level;
	for (count = 0;count < 50;count++)
	{
		if ((opt_level < ware_opt[count]) && ware_con != 0)																					//ȡa=������ȼ�����С��һ��
		{
			opt_level = ware_opt[count];
		}
	}
	upanddown(count);
	return 0;
}

uchar opt_in()																																							//�Ż�����
{
	uchar idata count_up;
	uchar idata count_down;
	uchar opt_level;
	uint a;
	count_up = floor_now;
	count_down = floor_now;
	while(ware_con[count_up])																																	//���ϲ��ҵ�һ���ղ㣬������Ǵ�1��ʼ�ģ����Դ˴�Ϊ1
	{																																													//�����ǰ�㲻Ϊ�գ���ִ��������
		if(count_up <= 1)																																				//������� 1  ��Ϊ�գ��򽫲�����������Ϊһ���ϴ������������ѭ��
		{
			count_up = 999;
			break;
		}
		count_up--;
	}
	
	while(ware_con[count_down])																																//���²��ҵ�һ���ղ㣬������������ȫ�ֱ�������
	{
		if(count_down >= floor_max)
		{
			count_down = 999;
			break;
		}
		count_down++;
	}

	if(count_up == 999 && count_down == 999)																									//������²�߱�Ǿ�Ϊ���״̬���򸳲������Ϊ ��-2
	{
		floor_full = 2;
	}
	if(floor_full != 2)
	{
		if((floor_now - count_up) <= (count_down - floor_now))																	//�ҵ�����Ŀղ�
		{
			upanddown(count_up);
		}
		else																																										//���������Ŀղ�������˶�������Ŀղ㣬��֮��Ȼ
		{
			upanddown(count_down);
		}
		for(a = 0;a <= floor_max;a++)																														//�������ȼ������
		{
			if(ware_opt[a] > ware_opt[a-1])
			{
				ware_opt_num = ware_opt[a];
			}
			else
			{
				ware_opt_num = ware_opt[a-1];
			}
		}
		if(pullboard() == 1)
		{
			ware_con[floor_now] = 1;																															//�洢��ɺ󣬶Դ˲��״̬�� 1
			ware_opt[floor_now] = 1;																															//�洢��ɺ󣬶Դ˲�����ȼ�����
			return 0;																																							//��ɷ��� 0
		}
		else
		{
			return 1;																																							//ʧ�ܷ��� 1
		}
	}
	else
	{
																																														//�˴��������ʾ
	}
}

void main()
{
	P2 = 0xFF;																																								//P2����������
	if (init() == 1)																																					//���ó�ʼ�����ҳ�ʼ�������ȷ
	{
		floor_now = 0;																																					//��ʼ����ɺ��趨��ǰ���Ϊ 0
		while(1)																																								//��ѭ������
		{
			if (flag_out == 0 && floor_full != 0)																									//���崫�������źţ��Ҳ��ǲ�Ϊ�գ���ִ�г���
			{
				opt_out();
			}
			if (flag_in == 0 && floor_full != 2)																									//���崫�������źţ��Ҳ��ǲ�Ϊ������ִ�н���
			{
				opt_in();
			}
		}
	}
	else																																											//��ʼ��ʧ���Ժ�
	{
		while(1)
		{
			
		}
	}
}