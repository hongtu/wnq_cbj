/*
注意事项
0、上升10mm所需的电机步数，临时设置为600
1、必须有一个全局变量用来统计电机的位置，从初始化之前就开始统计，防止初始化撞机或传感器失效
2、急停接触之后，必须有一个单片机复位初始化的过程，非从0初始化，防止信号的继续发送撞机以及非原点运动
*/
//除了初始化为 0 的常量或常量外，其他的变量或常量均在此处定义或声明
#include <STC89C5xRC.H>																																			//52单片机头文件
#include <stdio.h>																																					//基本函数库
#include <math.h>
#include <intrins.h>
#define uchar unsigned char
#define uint  unsigned int

//检测点
sbit k1 = P2^0;																																							//原点检测，光耦检测，端口常输出为高，光耦未遮挡，检测为低
sbit ori = P2^1;																																						//下极限检测
sbit k3 = P2^2;																																							//上极限检测
sbit flag_out = P2^3;																																				//出板检测
sbit flag_in  = P2^4;																																				//进板检测

//电机相关的变量
int p_imp;																																									//电机的位置
int a_imp;																																									//临时数据
#define speed_max 8									//需要注意更改的常量																		//电机的最大速度
#define speed_start 120							//需要注意更改的常量																		//启动速度
#define step  601 									//需要注意更改的常量																		//与电机的驱动步数要匹配
int speed_now;																																							//当前速度的变量

//层高变量
#define floor_max 50								//需要注意更改的常量																		//最大层数，计数从1开始，从顶层开始计
uchar idata floor_now;																																			//当前层高 
uchar idata floor_nex;
long  idata floor_t;																																				//因为这个数是有符号的，且数字比较大，所以用长整形
uchar idata floor_full;																																			//层满标志，空-0 有-1 满-2

//库存变量
uchar idata flag_ware_i = 0;																																//储板箱中从上面数的层数
uchar idata flag_sare_j = 0;																																//储板箱中从下面数的层数
uchar idata ware_con[floor_max];																														//用于存放每层箱中是否有板，0-无板，1-有板
uchar idata ware_opt[floor_max];																														//用于记录箱子中板子的优先级
uchar idata ware_opt_num;																																		//用于记录优先级的最大数

//状态变量
idata bit flag_iao;																																					//用于标记是否需要存板，0-存板优先，1-出板优先。设定好的优先级，会停止使用
idata bit flag_o = 1;																																				//原点标志，0-只允许向上运动，1-允许上下运动
idata bit flag_pull = 1;																																		//拉板完成标志，0-完成，1-未完成
idata bit flag_push = 1;																																		//推板完成标志，0-完成，1-未完成
idata bit flag_safe = 1;																																		//安全标志，每次动作前检查一次此标志，轮询的方法，不用中断
idata bit flag_up = 1;																																			//上板完成标志，0-完成，1-未完成
idata bit flag_down = 1;																																		//下板完成标志，0-完成，1-未完成
idata bit flag_t = 1;																																				//信息发送完成标志，0-完成，1-未完成
idata bit flag_r = 1;																																				//信息接收完成标志，0-完成，1-未完成
idata bit flag_top = 1;																																			//上极限
idata bit flag_bottom = 1;																																	//下极限
idata bit flag_ot = 1;																																			//超时标志

void delay_us(long idata time_us)																														//延时函数，据实测值推论，延时时间为 t*10 + 18 us 
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

uchar safe()																																								//安全检查
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

void upanddown(floor_nex)																																		//上下板函数，传递目标层变量
{
	long idata length_run;
	uint idata dir ;
	//if (safe()==1 && flag_ware==1)
	//{
		delay_ms(300);
		speed_now = speed_start;																																//对速度赋初值
		length_run = floor_nex-floor_now;																												//需要运动的层数 = 目的层高 - 当前层高
		floor_t = length_run * step;																														//后面的常数代表运动10mm所需的电机运转步数，这个值和电机驱动的设置有关
		if (floor_t > 0){dir = 0x05;}else{dir = 0x01;}																					//floor_t为正，向上运动，floor_t为负，向下运动
		floor_t = abs(floor_t);																																	//取绝对值
		for (length_run = 0;length_run < floor_t;length_run++)
		{
			P1 = 0x00;
			delay_us(speed_now);
			P1 = dir;																																							//需要运动的方向由dir变量控制
			delay_us(speed_now);
			if ((floor_t-length_run)>100){if(speed_now > speed_max)speed_now--;}else speed_now++;	//完成加速和减速的功能，缓冲距离100步
		}
		floor_now = floor_nex;																																	//运动完成后，当前层高 = 目的层高
	//}
}
uint init()																																									//初始化函数
{
	long idata init_con;
	speed_now = speed_start;
	upanddown(2);																																							//先向上运动两层，再重新找原点
	delay_ms(1000);
	while(ori==0)																																							//找到原点以后结束循环
	{
		P1=0x00;
		delay_us(speed_now);
		P1=0x01;																																							//向下运动
		delay_us(speed_now);
		a_imp++;
		if (speed_now > speed_max) speed_now--;
	}
	for(init_con=0;init_con<100;init_con++){P1=0x01;delay_us(speed_now++);P1=0x00;delay_us(speed_now);}//找到原点以后减速
	return 1;
}

uchar opt_out()
{
	long idata count;
	uchar opt_level;
	for (count = 0;count < 50;count++)
	{
		if ((opt_level < ware_opt[count]) && ware_con != 0)																					//取a=存板优先级里最小的一个
		{
			opt_level = ware_opt[count];
		}
	}
	upanddown(count);
	return 0;
}

uchar opt_in()																																							//优化函数
{
	uchar idata count_up;
	uchar idata count_down;
	uchar opt_level;
	uint a;
	count_up = floor_now;
	count_down = floor_now;
	while(ware_con[count_up])																																	//向上查找第一个空层，因计数是从1开始的，所以此处为1
	{																																													//如果当前层不为空，则执行以下行
		if(count_up <= 1)																																				//如果层数 1  不为空，则将层数变量设置为一个较大的数，并跳出循环
		{
			count_up = 999;
			break;
		}
		count_up--;
	}
	
	while(ware_con[count_down])																																//向下查找第一个空层，最大层数常量在全局变量定义
	{
		if(count_down >= floor_max)
		{
			count_down = 999;
			break;
		}
		count_down++;
	}

	if(count_up == 999 && count_down == 999)																									//如果上下层高标记均为溢出状态，则赋层满标记为 满-2
	{
		floor_full = 2;
	}
	if(floor_full != 2)
	{
		if((floor_now - count_up) <= (count_down - floor_now))																	//找到最近的空层
		{
			upanddown(count_up);
		}
		else																																										//如果到上面的空层近，则运动到上面的空层，反之亦然
		{
			upanddown(count_down);
		}
		for(a = 0;a <= floor_max;a++)																														//查找优先级最大数
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
			ware_con[floor_now] = 1;																															//存储完成后，对此层的状态置 1
			ware_opt[floor_now] = 1;																															//存储完成后，对此层的优先级置数
			return 0;																																							//完成返回 0
		}
		else
		{
			return 1;																																							//失败返回 1
		}
	}
	else
	{
																																														//此处做溢出提示
	}
}

void main()
{
	P2 = 0xFF;																																								//P2口做输入用
	if (init() == 1)																																					//调用初始化，且初始化完成正确
	{
		floor_now = 0;																																					//初始化完成后，设定当前层高为 0
		while(1)																																								//主循环函数
		{
			if (flag_out == 0 && floor_full != 0)																									//出板传感器有信号，且层标记不为空，则执行出板
			{
				opt_out();
			}
			if (flag_in == 0 && floor_full != 2)																									//进板传感器有信号，且层标记不为满，则执行进板
			{
				opt_in();
			}
		}
	}
	else																																											//初始化失败以后
	{
		while(1)
		{
			
		}
	}
}