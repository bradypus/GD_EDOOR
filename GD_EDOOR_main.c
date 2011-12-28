
/* ==============================================================================
System Name:  	GD_EDOOR

File Name:	  	GD_EDOOR_main.c

Description:	Primary system file for the GD_EDOOR Control

Originator:		
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 11-30-2011	Version 1.0
=================================================================================  */

// Include header files used in the main function

#include "PeripheralHeaderIncludes.h"
#include "IQmathLib.h"
#include "ED_EDOOR.h"
#include "ED_EDOOR-Settings.h"
#include <math.h>

#ifdef FLASH
#pragma CODE_SECTION(MainISR,"ramfuncs");
#endif


#define uEWEN  0x3000
#define uEWDS  0x0000
#define uREAD  0x80
#define uWRITE 0x40


// Prototype statements for functions found within this file.
interrupt void MainISR(void);
void DeviceInit();
void MemCopy();
void InitFlash();



void FORMAT();
void DELAY();

void gpio_set_e2prom_EWEN();
void gpio_set_e2prom_EWDS();
void KEYB();
unsigned char read_key();
void BEGIN();
void KEY138();
void delay_loop(unsigned int ude);
void set_bit_data(unsigned int udat);
unsigned int read_bit_data();
void EBOOST();
void send_data(unsigned int udat, unsigned char ubit, unsigned char umode);
void EWRE(unsigned int udata, unsigned int uaddr);
Uint16 ERDE(unsigned int uaddr);
void INE2M();
void INRAM();
void DISPLY();
void DISPLY1();
void PUT8(unsigned char data);
void VADCH();
void AUTO();

// Used for running BackGround in flash, and ISR in RAM
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;



int16 AX = 0;	 
char AL = 0;
char AH	= 0;
int16 BX = 0;
char BL	= 0;
char BHH = 0;
int16 CX = 0;	
char CL = 0;
char CH	= 0;
int16 DX = 0;
char DL = 0;
char DH = 0;

int16 AX1 = 0;
unsigned char AL1 = 0;	 
unsigned char AH1 = 0;	 
int16 BX1 = 0;
char BL1 = 0;
char BH1 = 0;
int16 CX1 = 0;	
char CL1 = 0;
char CH1 = 0;
int16 DX1 = 0;
char DL1 = 0;
char DH1 = 0;

int16 EX1 = 0;
char EL1 = 0;
char EH1 = 0;


int16 AX8 = 0;	 				//存放对应功能码的设置参数  ;BACK FOR RAM
int16 AX9 = 0;	 				//6AH 存放对应功能码
char AL9 = 0;
char AH9 = 0;


Uint16 FLO = 0;			//存放EEPROM RAM区中的EFLO(最小频率数值)
Uint16 FOM = 0;      	//存放EEPROM RAM区中的EFOM(最高频率数值)
Uint16 VTRQ = 0;	   	//Johnson 2011-10 低频力矩提升电压数值

Uint16 KA = 0;	   		//2011-10 Johnson 这个参数没有用到  32H
Uint16 INCFOL = 0;	   	//2011-10 Johnson 当前工作状态下的增速值的低16bit(Hz)  34H
Uint16 DECFOL = 0;	   	//2011-10 Johnson 当前工作状态下的减速值的低16bit(Hz)  38H
Uint16 DECFOH = 0;	   	//2011-10 Johnson 当前工作状态下的减速值的高16bit(Hz)  3AH

Uint16 DFINR1 = 0;	   	//2011-10 Johnson ???? 奇怪只有两个地方用到?   3CH

Uint16 NUMBER = 0;	   	//2011-10 Johnson 当前获得的编码器脉冲数       3EH

Uint16 T1 = 0;	   		//40H SVPWM运算的ALPA轴的时间(程序中用比较计数器数值来表示)
Uint16 T2 = 0;	   		//42H SVPWM运算的BELTA轴的时间(程序中用比较计数器数值来表示)
Uint16 TZ = 0;	   		//44H 相当于载波周期 (实际是要写入WG_RELOAD的数值)(具体载波周期的计算公式见本文开始)  
Uint16 S = 0;	    	//46H 开门位置脉冲百分数 (百分比是放大1000倍)

Uint16 FIA = 0;		   	//2011-10 Johnson SVPWM矢量运算中的角度THETA缓冲器  48H
Uint16 TAB = 0;	 	   	//2011-10 Johnson 固定为2048 感觉是本程序1024个点的SINE表的点数2倍  4AH

Uint16 KV1 = 0;	 		//2011-10 Johnson 是根据VANDF曲线算出来的VF曲线斜率*T (周期v-f斜率系数) 4CH
Uint16 FGC = 0;	   		//2011-10 Johnson 当前的运行电机频率(速度)的缓存器 (Hz) 4EH
Uint16 FOL = 0;	 		//2011-10 Johnson 上次运行电机频率(速度)低16bit(Hz)     50H
Uint16 FOH = 0;			//2011-10 Johnson 上次运行电机频率(速度)高16bit(Hz)     52H
Uint16 FG = 0;		   	//2011-10 Johnson 当前的运行电机频率(速度) (Hz)         54H
Uint16 DTS = 0;	   		//2011-10 Johnson 存放的上次编码器脉冲数与本次编码器脉冲数的差 (编码器遇到障碍物,DTS=0) 56H


int16 AX3 = 0;	   		//2011-10 Johnson 缓存上次编码器脉冲数 58H
int16 BX3 = 0;	 		//2011-10 Johnson 本程序没有用到 5AH
Uint16 FUN = 0;			//5CH 存放当前的功能码数值

Uint16 DLY = 0;	 		//5EH 存放延时外循环次数
Uint16 DLYS = 0;	 	//60H 存放延时内循环次数
Uint16 FSSK = 0;		//2011-10 Johnson 频率试试看  发现主要是ZDK在采用CAPTURE方式测量编码器脉冲时用到 本程序基本没有意义 62H
Uint16 NMAX = 30000;	//#NMAX是在C196.inc定义30000  ASMUUE A MAX NUMBER WHEN ON
Uint16 TDC = 0;			//2011-10 Johnson 感觉是和运行时间有关的参数  64H
Uint16 JDLY = 0;	 	//2011-10 Johnson 继电器延时时间(s) 66H  Relay Delay

						

Uint16 DCCT = 0;	 	//2011-10 直流制动时间 (对应RDCCT)  6CH
Uint16 SPT = 0;	   		//2011-10 Johnson对应RSPT 自学习推进时间 (S)  6EH

Uint16 DOCPT = 0;	 	//2011-10 Johnson 对应RDOPT  70H
Uint16 OBPT = 0;	 	//72H 力矩监控(OBSTCL)推进时间 PUSH TIME
Uint16 VRMS = 0;	 	//2011-10 Johnson 电压的均方根值，即万用表测量数值,是峰值的0.707 74H
Uint16 FIX = 0;	      	//2011-10 Johnson SVPWM矢量运算中的角度THETA    76H
Uint16 FIY = 0;	       	//2011-10 Johnson SVPWM矢量运算中的角度60-THETA 78H
Uint16 VDC = 0;	        //2011-10 Johnson 直流母线电压 79H

char SECTOR = 0;    	//2011-10 Johnson 扇区号标志 首次让SECTOR.0=1然后左移，代表1-6个扇区，之后再次从1开始 7AH        75H
char MODE = 0;	   		//2011-10 Johnson 这个参数本程序没有使用           7BH        76H
char NEWVECT = 0;	   	//2011-10 Johnson 进入WG(SPWM)波形发生中断服务的入口切换开关标志       7CH        77H

char RT = 0;	 	   	//2011-10 Johnson 感觉是个运行模式下的状态切换标志 7DH        78H

char KEY = 0;	 	   	//2011-10 Johnson 存放当前读取的有效键值(这个键值是经过转译后的方便程序使用)7EH       79H
char FRSH = 0;	 	   	//2011-10 Johnson 时间计数器  定时闪烁刷新标志，常用到FRSH.4 =10000=2ms FRSH.3=1000=1ms 7FH       7AH
char DYCT = 0;	 	   	//2011-10 Johnson 显示时间计数器                   80H	7BH
char CONTROL = 0;		//2011-10 Johnson 感觉是整个程序的控制切换状态     81H	7CH
char MST = 0;           //2011-10 Johnson 脉冲编码器状态标志    MST.0=0开门 MST.0=1关门       82H	7DH
						//核查历史版本之前ZDK用这个MST采用CAPTURE方式读三轴编码器使用 MST=#XXXX RZ RY RX DIR
char DSCD = 0;	 		//2011-10 Johnson 对应EDSCD 显示模式?               83H	7EH
char COMD = 0;	 		//2011-10 Johnson 外部控制命令标志寄存器(低四位有效)，不区分当前是键盘控制 还是键盘点动 还是外部端子84H		7FH
						//无论当前是外部端子控制还是键盘控制都统一映射成COMD的不同标志位：
						//COMD= # 1111 1111 B 没有任何控制命令
						//COMD= # 1111 1110 B 停止命令
						//COMD= # 1111 1101 B 强迫关门命令 (GBV10不支持这个命令:键盘没有这个键，端子也没有这个端子)
						//COMD= # 1111 1011 B 开门命令
						//COMD= # 1111 0111 B 关门命令
char AL0 = 0;	 		//2011-10 Johnson 外部故障标志寄存器 #1111 1101B(过压) #1111 1110B(过流)      85H		80H

char SMT = 0;	 		//2011-10 Johnson 这个变量竟然只有一个地方调用, 断言这个SMT标志没有用         86H		81H
char AL2 = 0;	 		//2011-10 Johnson 发现这个变量本程序没有使用                 87H		82H
//char QX = 0;	 		//2011-10 Johnson 在SIN、SCT显示输出数据和时钟处理中的缓冲器 88H		83H
//char QY = 0;	 		//2011-10 Johnson 在SIN、SCT显示输出数据和时钟处理中的缓冲器 89H		84H
//char QZ = 0;	 		//2011-10 Johnson 在SIN、SCT显示输出数据和时钟处理中的缓冲器 9AH		85H

char DBUF[5] = {0};	 
//DBUF1	 EQU	DBUF+1	   ;9CH           87H 第一数码管显示内容
//DBUF2	 EQU	DBUF1+1    ;9DH           88H 第二数码管显示内容
//DBUF3	 EQU	DBUF2+1    ;9EH           89H 第三数码管显示内容
//DBUF4	 EQU	DBUF3+1    ;9FH           8AH 第四数码管显示内容
//DBUF5	 EQU	DBUF4+1    ;0A0H          8BH 第五数码管显示内容
char TERMAL = 0; 		//2011-10 Johnson 整机端子输入状态标志 TERMAL=#1111 XXXX 低四位是当前端子控制状态	0A1H          8CH
						//对于HAD板:  TERMAL=#1111 IN4   IN3   IN2   IN1   端子无效下输入是高，过反相器是低到CPU,程序通过NOT方式取反，所以无效为高
						//对于GDV10板:  TERMAL=#1111 IN4   IN3   AUTO   GND  端子有效下输入是低，进过反相器是高CPU,程序通过NOT方式取反，所以有效为低
						//关门  开门  AUTO(强迫关门 ) 屏蔽(停止)
char SAMP_T = 0;  		//2011-10 Johnson 取样时间，好像只有三个地方用到。			0A2H          8DH
char SD_FLG = 0;		//2011-10 Johnson 自学习标志(设置参数0,1,2)       			0A3H          8EH
						//2011-10 SD_FLG= 0:自学习开始；1:门关齐结束第一过程　2:自学习结束
char APPL = 0;	 		//0A4H            存放EEPROM RAM中的F58(可调频模式参数): 1->上电显示最大脉冲数目,0->上电显示最大频率值

Uint16 FX = 0;	 		//0A5H            增加的缓冲寄存器(用于在AT93C66的EESK) 2006-10-6 JJ

Uint16 PASS = 0;		//0A7H EEPROM授权密码,如果没有授权密码，则开机数码管没有显示,可以通过按写入键实现授权把3FH(63)-->(C8H)

Uint16 LSP0 = 0;	 	//0A8H 2011-10 Johnson 计算用的最小频率低16bit CACULATING MIN FREQ-LO
Uint16 LSP = 0;	 		//0A9H 2010-10 Johnson 计算用的最小频率高16bit CACULATING MIN FREQ-HI.running min freq.

Uint16 FUNM = 0;	 	//0AAH 2011-10 Johnson 临时实验恢复当初的ZDK 数值  58       ;最大功能码数目上限58                  63

//Uint16 TMIN0 = 0; 		//0ABH 2011-10 Johnson 本程序没有用到

                           
/*====================================================
*        EEPROM 定址变量(与RAM定址对应)
* E开头表示是EEPROM (AT93C66 16bit Data)
* 该段RAM存放从EEPROM读取的对应功能码的设置参数
* 实际程序中另有对应的中间RAM存放同样的参数进行
* 运算控制. (全部为偶数地址=FUN+FUN)
*====================================================*/
Uint16 E2M[160] = {0};	 						//0ACH 代表从EEPROM读取的功能码数值存放的RAM单元首地址     REFLECT THE SERIES-E2ROM


#define 	EFG		0     		//00 2011-10 Johnson "设定频率"是什么意思? 设定最高运行频率数(Hz 放大100倍)不可自己调整,出厂为5000,只能在自学习过程中程序自己改变
#define 	EFLO    1      		//01 2011-10 Johnson 最低始动频率 (Hz，放大100倍)        MIN FREQ.
#define 	EFOM    2      		//02 2011-10 Johnson 最高运行频率 (Hz,放大100倍)         MAX FREQ.
#define 	EOF1M   3      		//03 开门基本转矩频率(Hz) 该值越大转矩越小，反之亦然     OPEN BASE FREQ
#define		EOTRQ   4      		//04 开门低频转矩补偿(%)  该值越大转矩越大               OPEN TRQUE BOOST
#define		EACL0   5      		//05 自学习常规加速时间(S)    调整该参数可使自学习能顺利进行 STUDY SPEED ACL
#define		EDCL0   6      		//06 自学习常规减速时间(S)    调整该参数可使自学习能顺利进行 ----  ----- DCL

#define		EACL1   7		    //07 保留参数(反向加速时间)(建议屏蔽) 调整该参数可使遇光幕及发生力矩监控时能有平滑运行曲线。REVERSING SPEED ACL
#define		EDCL1   8	      	//08 反转减速时间(S)  调整该参数可使遇光幕及发生力矩监控时能有平滑运行曲线。 -------	 DCL

#define		EACL2   9      		//09 常规加速时间(S)  调整该参数能使门常规运行有平滑运行曲线。    NORMAL SPEED	 ACL
#define		EDCL2   10      	//10 常规减速时间(S)  调整该参数能使门常规运行有平滑运行曲线。    ----  -------	 DCL

#define		ECMD    11      	//11 控制模式  2=外部继电器端子控制，1是点动(目前不起作用)，0是键盘控制 控制模式(建议改成由外接开关设定) COMMAND MODE
								//HAD: F11=0面板控制模式；F11=1手动模式；F11=2正常模式(端子控制模式)
#define		ESTT    12      	//12 保留参数(建议取消?) 停止模式 0,1  好像有两种停止方式，自由停止还是减速停止?    STOP MODE
#define		EOPM    13     		//13 2011-10 Johnson 发现这个参数没有用到  保留参数(建议取消?)  0-SINGAL 1-DOUBLE
								//位置测量方式: 0=单通道，1=双沿单通道，2=双通道，出厂是2
#define		ENULL   14      	//14 门宽矫正(%)   此参数的设置主要为消除自学习过程中由于编码器误差、机械误差等造成的脉冲误差       PULSE OFFSET

#define		EEXCM   15      	//15 保留参数(外部通信模式)(建议屏蔽)                EXT COM MODE 22/12-1999
#define		EFBK	16      	//16 保留参数(直流制动起始频率)(建议屏蔽)(Hz)放大100倍   BRAKE FREQ.
								//制动频率一般是指变频器的直流制动起始频率，当变频器输出频率低于停机直流制动起始频率时，变频器将启动直流制动功能，
								//给电机注入直流电流，产生动态刹车快速停机
								//直流制动，一般指当变频器输出频率接近为零，电机转速降低到一定数值时，变频器改向异步电动机定子绕组中通入直流，形成
								//静止磁场，此时电动机处于能耗制动状态，转动着转子切割该静止磁场而产生制动转矩，使电动机迅速停止。

#define		ETBK    17      	//17 保留参数(直流制动时间)(建议屏蔽)                BRAKE TIME
#define		EVBK    18      	//18 保留参数(直流制动强度)(建议屏蔽)(Hz)            BRAKE AMOUNT
#define		EFC     19      	//19 载波频率(Hz,放大100倍)                          CARRIER FREQ.

#define		ESAV    20    		//20 数据保护(418可修改保存) (888恢复出厂数值)(8080全部显示功能码) DATA SAVE
								//129H 字存储
#define		XSAV    20     		//保存F20对应的EEPROM起始地址, 由于数据要求是16bit, 所以地址是偶数. 数据之所以要16bit是因为功能参数有个别需要到16bit才行

#define 	EDELY   21       	//21 保留参数(通电延时)(建议屏蔽)本软件未用 [软件版本号显示时间的长短，也是预充电继电器OUT1的延时时间 by 王禹?]  SWITIH ON DELAY
#define		ESYSTM  22      	//22 保留参数(波形输出方式正常/测试模式)(建议屏蔽) 本软件未用  NORMAL/TEST MODE
#define		EDEADT  23			//23 保留参数(死区时间)(us)(建议屏蔽)                          DEAD TIME

#define		EOBSTCL 24 			//24 门机力矩监控 0:力矩监控无效 1:力矩监控有效                DOOR TRQUE SUP
					//当轿门关闭时，对发生在特定范围内(门宽的8%~80%)的关门运动，若存在超过最大允许关门力矩的反向力矩施加，轿门会停止关闭，
					//并且自动执行开门，门开齐后，再自动关闭轿门。
					//正常运行关门过程中检测编码器脉冲计数是否停顿，1=检测 (检测有停顿一段时间就认为撞到物体，再开门)，0=失效（不检测一直关门输出下去）
					//目的：防止光幕失效的情况下，夹人或是物体。关门力矩监控有效范围在开门宽度的80%--8%，不能调整

#define		DOS0    25    		//25 开门换速位置0(%)  OPEN POSITION 0
#define		DOS1    26     		//26 开门换速位置1(%)  ------------- 1
#define		DOS2    27    		//27 开门换速位置2(%)  ------------- 2
#define		DOS3    28			//28 开门换速位置3(%)  ------------- 3
			   //2011-10 Johnson 四个换速点把开门分成五段

#define		DCS0    29			//29 关门换速位置0(%)  CLOSE POSITION 0
#define		DCS1    30			//30 关门换速位置1(%)  -------------- 1
#define		DCS2    31			//31 关门换速位置2(%)  -------------- 2
#define		DCS3    32			//32 关门换速位置3(%)  -------------- 3
			   //2011-10 Johnson 四个换速点把关门分成五段

#define		DOV0    33			//33 开门分段速度0(开始速度)(Hz)    OPEN VELOCITY 0 (START SPEED)
#define		DOV1    34			//34 开门分段速度1(高速)(Hz)        ------------- 1 (HIGH SPEED)
#define		DOV2    35			//35 开门分段速度2(结束速度)(Hz)    ------------- 2 (END SPEED)

#define		DOV3    36			//36 开门分段速度3(外拉速度)(Hz)    ------------- 3 (PUSH SPEED)

#define		DCV0    37			//37 关门分段速度0(开始速度)(Hz)    CLOSE VELOCITY 0 (START SPEED)
#define		DCV1    38			//38 关门分段速度1(高速)(Hz)        -------------- 1 (HIGH SPEED)
#define		DCV2    39			//39 关门分段速度2(结束速度)(Hz)    -------------- 2 (END SPEED)

#define		DCV3    40			//40 关门分段速度3(内推速度)(Hz)    -------------- 3 (PUSH SPEED)

#define		EPOS    41			//41 位置继电器位置设定(%)          POSITION  RELAY
#define		EDOPT   42			//42 关门推进时间(S)                OPEN ?? PUSH TIME 指门关到位后，变频器输出低频夹持力，参数代表夹持力持续时间，F42=100，无限推进
			   //   端子指令关门使能，参数90秒，门关齐后，撤销端子关门指令，会在90秒后封锁PWM推进。例：参数为100秒，撤销端子关门指令，会无限时关门低速推进

#define		EDCPT   43			//43 开门推进时间(S)                CLOSE?? PUSH TIME 指门开到位后，变频器输出低频夹持力，参数代表夹持力持续时间，F43=100，无限推进
			   //2011-10 Johnson 感觉ZDK是有意把变量名字搞颠倒的
			   //   端子指令开门使能，门开齐后，撤销端子开门指令，参数100秒，会无限时开门低速推进。例：参数为99秒，撤销端子开门指令，会在99秒后封锁PWM推进。
			   //   现场一般都用100.

#define		ESPT    44			//44 自学习推进时间(S)              STUDY  PUSH

#define		EDSCD   45			//45 保留参数(显示模式)(建议屏蔽)(定义为字,实际只有低字节有意义) 0，1,2,3, Default 2  DISPLAY MODE
			   //0=显示设定频率，1=显示输出频率，2=显示位置百分数，3=显示位置脉冲数

#define		EFG0    46			//46 自学习常规速度(Hz) 调整该参数可改变自学习速度            	STUDY SPEED
#define		EFG1    47			//47 自学习推进速度(Hz) 调整该参数可改变自学习速度                    STUDY PUSH SPEED

#define		EFG2    48			//48 强迫关门速度(Hz)   轻推关门速度，调整该参数可改变轻推关门速度    NRDING SPEED
			   //   强迫关门是指慢速关门， (GDV10此参数现已不用，硬件端子慢速关门已去掉)

#define		EF1M0   49			//49 自学习力矩频率(Hz) 该值越大转矩越小，反之亦然                    STUDY TRQUE
#define		EPULSE  50			//50 自学习总脉冲数(个)             TOTAL PULSE
#define 	XPULSE  50			//存放F50 EPULSE(自学习总脉冲数)对应的EEPROM物理起始地址, 类似XSAV

#define		ECF1M   51			//51 关门基本转矩频率(Hz) 该值越大转矩越小，反之亦然                  CLOSE BASE FREQ.

#define		ECTRQ   52			//52 关门低频转矩补偿(%)  该值越大转矩越大    CLOSE TRQ BOOST
#define		EOPL    53			//53 门开齐继电器信号(%)  调整该参数可改变门开齐信号输出位置          OPEN  POS.
#define		ECLL    54			//54 门关齐继电器信号(%)  调整该参数可改变门关齐信号输出位置          CLOSE POS.
#define		ERST    155			//55 遇OU或OC故障恢复模式(保护后停机/保护后可自动重启) 出厂是1 ERST=0键盘处理故障模式 ERST=1 键盘和端子处理故障模式 RESET MODE
				//   1)F55=1  在出现故障时候，在遇到 键盘停止 或 端子开门 或端子关门时，系统复位自动从头执行， 否则一直在循环
				//   2)F55=0  在出现故障时候，只有遇到键盘停止键 才能系统复位，否则一直循环。
				//
				//F55=0遇OU或OC故障保护装置处于停止状态，当重新上电或接停止信号时可恢复正常状态。
                //   F55=1遇OU或OC故障保护，装置接外部任何信号可自动恢复正常状态。
#define		ESDY    56			//56 自学习记忆模式0,1,2 自学习脉冲数是否保存到F50 EEPROM单元 出厂是0   AUTO-STUDY MODE
			   //                 0: 保存到F50  1:差值<20则保存，否则不保存 2:不保存
			   //F56：自学习记忆模式。F56=0装置会在每次通电或复位重新自学习脉冲数，并记忆最后一次自学习脉冲数。
                           //F56=1装置只记忆(F56=0)时的最后一次自学习脉冲数，以后遇自学习如果所学脉冲数与最后一次自学习脉冲相差±15脉冲以上装置会重新学习，
                           //直到与记忆的脉冲数相符时，装置才会自动转入正常模式。F56=2装置会存储自学习脉冲数，遇断电上电后不会重新学习，按照存贮的脉冲数工作。
                           //自学习时F56应设定为"0"，正常工作可设定为"0"或"2"，不可设定为"1"；"1"厂家保留参数。

#define		EJDLY   57			//57 强迫关门延时发信号时间(S)           RELAY DELAY
			   //   强迫关门是指慢速关门，在门开齐的情况下，端子接到慢速关门指令，延时2秒后，再执行关门动作。(GDV10此参数现已不用，硬件端子慢速关门已去掉)

#define		EAPPL   58			//58 保留参数(可调频率模式)(建议屏蔽) [0,1] 1->上电显示最大脉冲数目,0->上电显示最大频率值   NORMAL/DOOR CON.
			   //   2011-10 Johnson 这个参数解释可能有问题，或许是 1->显示当前运行编码器脉冲数 0->显示当前运行电机频率值

/********以下变量是ZDK后来增加的，有些是具体配套用户要求的***************/

#define		ESSK    59			//59  PULSE    本程序没有用
#define		EF_HD   60			//60  参数隐含 本程序没有用

				//178H	   ;F60:  开门过程中丢码，门开到位后强行发出“门开齐”信号延时时间，这个时间由F60单元参数决定，即F60=10为门开到位10秒后发，
                //         ;F60=20  为门开到位20秒后发，以此类推，设定F60=100时，为无限延时发。
                //17AH     ;F61   门区信号是否有效 F61=1门区信号有效，开门时必须有门区信号方可开门，F61=0开门时无需门区信号，即可开门。
                //17CH     ;F62
                //17EH     ;F63   显示当前VDC的采样数 (本程序在FUNM=63时，F63有效)
				//好像F59--F63是当初HAD为进入天津奥迪斯被要求增加的

/********PWD是保存在EEPROM的授权密码参数*********************************/

#define		EPWD	100			//EEPROM的授权密码地址
#define 	XPWD	100      	//EEPROM的授权密码,每次开机都要检查对应的EEPROM物理单元的授权码,如果没有要按写入键,否则开始是没有LED显示的
			   					//具体授权密码内容看PASS参数定义

/********以下变量的地址数据是程序实际运行计算出来的**********************/

#define		RDCCT	104			//直流制动时间参考               DC-BRAKE TIME REFERENCE
#define		RDOPT	105			//关门推进时间参考               DOOR OPEN ??  TIME REF.

#define		RDCPT	106			//开门推进时间参考 程序好像未用  DOOR CLOSE??  TIME REF. 也没有设置配合的RAM 计数变量DCPT
				//2011-10 Johnson 感觉ZDK是有意把两个变量名字颠倒

#define		RSPT	107		   	//自学习时间参考                   STUDY TIME REF.
#define		RJDLY	108			//强迫关门继电器延时发信号时间参考 RELAY DELAY REF.

/********以下变量是根据EEPROM功能码参数经过程序计算出来的*****************/

#define		DOSA	112	   		//2011-10 Johnson DOSA=DOS1-DOS0   S曲线 开门平方特性     SQUARE CHARACTICS OF DOOR OP
#define		DOSB	113			//2011-10 Johnson DOSB=DOS3-DOS2
#define		DOVA	114			//2011-10 Johnson DOVA=2*(DOV1-DOV0)
#define		DOVB	115			//2011-10 Johnson DOVB=2*(DOV1-DOV2)

#define		DOS01	116			//2011-10 Johnson 应该是开门换速位置01，就是在第0个换速位置和第1个换速位置之间的点 (%) 确保S曲线正确润滑
#define		DOS23	117			//2011-10 Johnson 应该是开门换速位置23，就是在第2个换速位置和第3个换速位置之间的点 (%) 确保S曲线正确润滑

#define		DCS01	118			//2011-10 Johnson 应该是关门换速位置01，就是在第0个换速位置和第1个换速位置之间的点 (%) 确保S曲线正确润滑
#define		DCS23	119			//2011-10 Johnson 应该是关门换速位置01，就是在第2个换速位置和第3个换速位置之间的点 (%) 确保S曲线正确润滑

#define		DCSA	120			//2011-10 Johnson DCSA=DCS1-DCS0  S曲线 关门平方特性    SQUARE CHARACTICS OF DOOR CL
#define		DCSB	121			//2011-10 Johnson DSCB=DCS3-DCS2
#define		DCVA	122			//2011-10 Johnson DCVA=2*(DCV1-DCV0)
#define		DCVB	124			//2011-10 Johnson DCVB=2*(DCV1-DCV2)
			   //87C196MC 488byte RAM的最高地址是1FFH, 故而没有超越87C196MC RAM限制


char ERROR[] = 	{0xFF, 0x49, 0x63, 0xFF,	      			//SC   0      (错误指示代码表)
         		0xFF, 0xC5, 0xE5, 0xFF,	        			//oc   4
	 			0xFF, 0xE3, 0x83, 0x25,	        			//LU2  8
         		0xFF, 0xC5, 0xC7, 0xFF,	       				//ou   12
	 			0xFF, 0x61, 0xF5, 0xF5};	        		//Err  16

/*	  //	0   1	2   3	4   5	6   7
ADISP:	 DCB   03H,9FH,25H,0DH,99H,49H,41H,1FH	;0-7
	 DCB   01H,09H,11H,0C1H,63H,85H,61H,71H ;8-F

	  ; 14*16=224字节(0-222)
INPAR:	 DCW  10,    10,    3000,  3000,   0,	  1,	 1,	1     ;7
	 DCW  1,     1,     1,	   0,	   0,	  2,	 0,	1     ;15
	 DCW  45,    2,     20,    4000,   0,	  0,	 0,	2     ;23
	 DCW  0,     0 ,    150,   500,    800,   900,	 600,	200   ;31
	 DCW  0,     45,    1000,  45,	   45,	  45,	 1000,	45    ;39
	 DCW  45,    0,     0,	   0,	   0,	  0,	 45,	47    ;47
	 DCW  48,    4000,  0,	   3000,   0,	  900,	 10,	0     ;55
	 DCW  0,     0,     0,	   0,	   0,	  0,	 0,	0     ;63

UPPAR:	 DCW  50000, 400,   6000,  50000,  150,   999,	 999,	999   ;7
	 DCW  999,   999,   999,   2,	   1,	  2,	 300,	1     ;15
	 DCW  100,   2,     20,    16000,  9999,  400,	 3,	2     ;23
	 DCW  1,     150,   300,   800,    1000,  1000,  900,	500   ;31
	 DCW  350,   1000,  5000,  1000,   1000,  1000,  5000,	1000  ;39
	 DCW  1000,  340,   100,   100,    10,	  3,	 5000,	1000  ;47
	 DCW  2000,  10000, 9999,  50000,  150,   990,	 100,	1     ;55
	 DCW  2,     10,    1,	   1,	   1,	  1,	 1,	1     ;63*/

Uint16 FACTRY[] = {	5000,  45,    5000,  5500,   100,	200,	100,	300,   		//7
					100,   100,   150,   0,	   	 0,	    2,	 	20,		1,     		//15
					45,    2,     20,    14000,  418,   400,	1,	    2,     		//23
					1,     30,    180,   650,    980,   980,	850,	300,     	//31
					20,    400,   2800,  400,    400,   400,	2000,	400,		//39
					400,   300,   100,   100,    2,	  	2,	 	1200,	300,   		//47
					1200,  5000,  1000,  6500,   80,	950,	50,		1,     		//55
					0,     2,     1,	 0,	   	 0,	  	0,	 	0,		0  };     	//63
					
	
	 

interrupt void cpu_timer0_isr(void);	 

void main()
{
	
	DeviceInit();	// Device Life support & GPIO	
	// Only used if running from FLASH
	// Note that the variable FLASH is defined by the compiler
	
	#ifdef FLASH
	// Copy time critical code and Flash setup code to RAM
	// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files. 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM
	InitFlash();	// Call the flash wrapper init function
	#endif //(FLASH)
	
	// Disable CPU interrupts
  	DINT;
	
   	// Disable CPU interrupts and clear all CPU interrupt flags:
   	IER = 0x0000;
   	IFR = 0x0000;
   	// Initialize the PIE vector table with pointers to the shell Interrupt
	// Service Routines (ISR).
	// This will populate the entire table, even if the interrupt
	// is not used in this example.  This is useful for debug purposes.
	// The shell ISR routines are found in DSP2803x_DefaultIsr.c.

   	// Interrupts that are used in this example are re-mapped to
	// ISR functions found within this file.
  	EALLOW;  // This is needed to write to EALLOW protected registers
   	PieVectTable.TINT0 = &cpu_timer0_isr;
   	EDIS;    // This is needed to disable write to EALLOW protected registers
   	InitCpuTimers();   // For this example, only initialize the Cpu Timers
   	// Configure CPU-Timer 0, 1, and 2 to interrupt every second:
	// 60MHz CPU Freq, 1 second Period (in uSeconds)

   	ConfigCpuTimer(&CpuTimer0, 60, 1000000);
   	CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
   	// Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
	// which is connected to CPU-Timer 1, and CPU int 14, which is connected
	// to CPU-Timer 2:
   	IER |= M_INT1;
   	// Enable TINT0 in the PIE: Group 1 interrupt 7
   	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

	// Enable global Interrupts and higher priority real-time debug events:
   	EINT;   // Enable Global interrupt INTM
   	ERTM;   // Enable Global realtime interrupt DBGM
   	
   	EALLOW;
   	GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
   	GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
   	EDIS;
   	
   	GpioDataRegs.GPBSET.bit.GPIO34 = 1;
   	
   	// Step 6. IDLE loop. Just sit and loop forever (optional):
   
   	
   	

	MST = 0x05;         						//DIR=MST.0=1 关门  #0000 0101B; 
	SAMP_T = 0;									//清SAMP_T标志
	COMD = 0xFF;								//赋COMD为0FFH
	SD_FLG = 0;                					//清自学习标志  2011-10 Johnson 设为自学习开始状态 SET SELF_STUDY
	DOCPT = 0;									//清DOCPT标志
	OBPT = 0;									//清OBPT标志
	JDLY = 0;									//清JDLY标志
	FSSK = NMAX;            					//#NMAX是在C196.inc定义30000  ASMUUE A MAX NUMBER WHEN ON
	NUMBER = FSSK;								//把当前获得的编码器脉冲数置为30000
	//TIMER1[0] = NUMBER;						//在DSP中怎么表示，装到脉冲计数器中去？
	S = NMAX >> 1;              				//2011-10 Johnson感觉S定义不明确，这时候代表50%门宽的脉冲数。设置开机门位置为50%
												//SET INICAL DOOR POSTION AT 50.0%
												//在程序之后S 都代表当前的门宽位置的百分比(是放大1000倍)
	
	FORMAT();                					//初始化子程序
	INE2M();                 					//核实F20功能数值，如果是首次运行，则需要首次运行需要把功能码参数写入空EEPROM子程序
	BEGIN();                 					//辅助初始化程
	
	//------- 上电电压检测程序（2003-5-13）--------
 	/*VADCH();   
	while(VDC <= 384) {
		VADCH();
	}*/
	
	AX1 = VDC - 384;							//384=512*75% 实际512对应多少V? 为什么一定要大于384才能工作? 王禹说220Vac(310DC) 代表VDC=2.5V，那么170Vac代表1.93
                                        		//2011-10 Johnson 根据公式10bit AD: 384对应VDC=1.877V,应该是180V交流输入
                                        		//核奥达说明书支持交流输入范围：180V - 265V
												//实际测量HAD的板在VDC=1.877V时输入是174VAC，GDV10板是173VAC
	
												//逻辑或之前INT_MASK为00H,或后意味开启TOVF(Timer1 Or Timer2 溢出中断共享)
	  											//之前PI_MASK为00H
	   											//OPEN TF2
												//逻辑或之前INT_MASK1为00H,或后意味开启Extint外部中断
	 											//开指定的中断
	 											
	//***  03H,9FH,25H,0DH,99H,49H,41H,1FH  ;0-7
    //***  01H,09H,11H,0C1H,63H,85H,61H,71H ;8-F

	DBUF[0] = 0xC1;           					//b(#0C1H)准备显示厂家信息和软件版本号码: bb5.0
 	DBUF[1] = 0xC1;           					//b(#0C1H)
	DBUF[2] = 0x48;            					//5.(#48H)
	DBUF[3] = 0x03;           					//0(#03H)
	DBUF[4] = 0xFF;           					//关闭所有LED指示
	BX1 = 1000;
	while(BX1 --) {
												//显示版本号码持续约定时间 100xDelay TIME 100x2.4ms=2.4秒
		DELAY();
	   	DISPLY();								//间接(RT.2=1判断有效)数码管数据发送显示处理子程序
	}
	
	//----------  switch on pre-charger relay ----------
	//2011-10 P2.5 OUT1启动继电器开启 (启动继电器 有效)

	//逻辑或之前INT_MASK为00H,或后意味开启TOVF(Timer1 Or Timer2 溢出中断共享)
	//2011-10 Johnson 这段和START2有重复嫌疑
	//逻辑或之前INT_MASK1为00H,或后意味开启Extint外部中断
	//打开所有程序运行需要的中断,准备进入常规主循环程序
	
	
	
	//************************************************
	//*	      常规静态主循环程序	        *
	//************************************************
	/*GETPOS();											//调用位置脉冲子函数,位置存在NUMBER中
	VADCH();											//检测直流母线电压
	AUTO();												//判断AUTO键是否按下，影响ECMD[0]
	
	//P2_REG[0] |= #00010000B;							//设置P2.4(THR)为高,表示ZDK把PWM输出的245关闭，
														//本程序把THR替代为别的，实际没有THR, 意味本句对BB50没有作用
	DSCD = E2M[EDSCD] ;									//显示模式F45? Default 2 0=显示设定频率，1=显示输出频率，2=显示位置百分数，3=显示位置脉冲数
	//INT_MASK.bit0 = 1;									//OPEN TIMOV (Timer1 & 2 溢出中断)
	
	FOM = E2M[EFOM]										//字装载,把EEPROM RAM F02对应的最高频率数(Hz 放大100倍)存放到FOM中
	FLO = E2M[EFLO]										//字装载,把EEPROM RAM F01对应的最低始动频率数(Hz 放大100倍)存放到FLO中
	FG = E2M[EFG]											//字装载,把EEPROM RAM F00对应的设定(给定)频率数(Hz 放大100倍)存放到FG中
	
	S = NUMBER * 1000 / E2M[EPULSE];						//保存S 为当前的门的%位置 (%都是放大1000倍) 30000% ?  EPULSE[0]为F50 自学习总脉冲数(出厂是1000)	
														//出厂时由于EPULSE=1000,所以NUMBER * 1000 /1000 =NUMBER 应该是第一次加电 AX1还是30000才对
														//把当前获得的编码器脉冲数 * 1000 / 本机的自学习总脉冲数 应该是计算当前获得的门宽位置百分比并放大1000倍
														//有可能或导致结果 ( S ) >100% (1000), 只有通过自学习后才可能导致<=1000 (100%)
										
	KEYB();												//读当前有效键值,出口参数KEY,RT, 读当前键盘状态子程
	AUTODY();											//调用自动显示子程序
	
	if(KEY & 0xBF == 0) {								//是功能键?(功能键为#10111111B),首次上电KEY=#0FFH 
		//DELAY2(200);									//是功能键 则延时200个基本单位时间 200x2.4ms=480ms
		GETS();											//把当前功能码对应的设置参数读到对应的RAM单元中
		CONTROL.bit0 ^= 0x01;							//CONTROL异或#0000 0001B 有意让显示和不显示交替出现，造成人为的显示闪烁
	} //end KEY.bit6 == 0
	
	if(CONTROL & 0x01 == 0) {							//CONTROL.0=0 代表显示当前功能码参数
		PARS();											//查找到功能码对应设置参数处理子程序
		
		if(KEY & 0x20 == 0 || KEY & 0x10 == 0 || FRSH & 0x10 == 0){			//如果是UP或DOWN键则转DPAR处理程序(UP键=#11011111B,DOWN键=#11101111B) 
																		//FRSH的意思10000B x 16ms = 256ms
			DISPAR();									//设置参数显示(带单位)子程序
		} else {
			BLACK();									//调用数码显示清屏处理程序 只是把DBUF1,2,3,4,5置为0FFH，等下次DISPLAY来显示，出现闪烁
		}  //end KEY.bit5 == 0 || KEY.bit4 == 0 || FRSH.bit4 == 0
		
		WRS();											//当前功能码设置参数写入对应EEPROM 功能码单元子程序
	} else {							
		FUNS();											//CONTROL.0=1 代表显示当前功能码 调用功能码筛选处理程序
		DISFUN();										//调用数码管显示经过筛选后的功能码子程序
	}  //end CONTROL.bit0 == 0
	
	DISPLY();											//间接(RT.2=1判断有效)数码管数据发送显示处理子程序
	
	if(KEY & 0x04 == 0 || (KEY & 0x08 == 0 && KEY & 0x04 == 0)){		//是强迫关门键或软件复位键? (强迫关门键=#11111011B 软件键=#11110011B),则转READY处理程序
																		//是软件复位键?(软件键=#11110011B)?,则转READY [GDV10板不支持强迫关门键]
		LJMP	  START	             ;是软件复位键则从头开始执行     RESET AUTO-STUDY    这部分用C语言怎么处理还得具体考虑   ?????????????????
	} else {
		APPL = EAPPL[0];	        					//把RAM中对应EPPROM里F58的可调频率模式 参数装到APPL中 1->上电显示最大脉冲数目,0->上电显示最大频率值
		
		if(APPL & 0x01 == 1) {							//检查APPL.0是不是1?(1->上电显示最大脉冲数,0->上电显示最大频率值),如果是则转READY1程序
			COMMAND();									//(只处理上电最大脉冲显示模式)命令处理子程
			
			if(SD_FLG != 2) {							//自学习是否结束?  sd_flg=2 means self-study finished
				P2_REG[0] &= #00111111B;				//把OUT2，OUT3继电器关闭 (门开齐 门关齐失效)
				WG_OUT[0] &= #1111111101111111B;  		//把OUT4也关闭 (指定位置 失效)
				
				if(SD_FLG == 0) {						//是自学习开始状态吗? 如果是，则开始SFIRST(自学习第一阶段)   自学习第一阶段为关门操作
														//SD_FLG=0 	代表当前需要进入自学习第一阶段，不能直接自动运行
					if(COMD & 0x08 == 0 || COMD & 0x04 == 0) {			//COMD.3=0 (关门命令)? 则转SFIST1 self study first course begins  only by "close" pressed
																	//COMD.2=0 (开门命令)? 则装SFIST1 self study begins (open)   2004-03-10
						DELAY2(10);						//自学习第一阶段开始先延时2.4x10=24毫秒
						MST |= 0x01;					//让MST.0=1 关门 f/r dir  close
					} else {
						DELAY2(1);						//延时一个基本单位时间2.4ms
						LJMP	  STAY			;重新开始常规静态主循环程序
					}
				} else {								//自学习第二阶段为开门操作
					DELAY2(10);							//自学习第二阶段开始先延时2.4x10=24毫秒
					MST &= 0xFE;						//MST.1=0 开门 direction open
				}  //end SD_FLG == 0
			} else {
				if(COMD & 0x02 == 0) {		        	//2011-10 (COMD.1=0代表强迫关门命令) 如果是强迫关门命令 则转RCLOSE1程序
					MST |= 0x01;						//让MST.0=1 关门 f/r dir  close
				} else if(COMD & 0x04 == 0) {         	//(COMD.2=0代表开门指令) 如果是开门命令 则转ROPEN1程序
					MST &= 0xFE;						//MST.1=0 开门 direction open	
				} else if(COMD & 0x08 == 0) {        	//(COMD.3=0代表关门命令) 如果是关门命令 则转RCLOSE1程序
					MST |= 0x01;						//让MST.0=1 关门 f/r dir  close
				} else {
					J123();                  			//根据当前的S (门宽位置百分比) 调用继电器输出子程序
					DELAY2();							//延时一个基本单位时间2.4ms
					EXCOM3:    LJMP	  STAY			;重新开始常规静态主循环程序
				}  //end COMD.bit1 == 0
			}  //end SD_FLG != 2
		} else {
			if(KEY & 0x01 == 1) {  						//在上电显示最大频率模式下，如果不是开门键(开门键=#11111110B),则转EXCOM3 重新开始常规静态主循环
				EXCOM3:    LJMP	  STAY		;重新开始常规静态主循环程序
			} else {
				MST &= 0xFE;							//MST.1=0 开门 direction open
			}  //end KEY.bit0 == 1
		}  //end APPL.bit0 == 1
		
		
		//------------------正常运行(不是循环方式)-----------------------------------
		//分析程序后 通过MST来区分当前是开门还是关门方向
		
		INICIAL();										//调用运行初始化INICIAL子程序
		ADCL0();	                					//STUDY SPEED 频率跳变点预处理--自学习常规加减速处理子程序
														//获得INCFOL,INCFOH; DECFOL,DECFOH
														//自学习阶段不算DECFOL,DECFOH
		
		DELAY2(10);					               		//延时2.4 * 10 = 24ms
	   
		P2_REG[0] &= #11101111B;						//这时候相当于THR低，打开245的输出，准备输出PWM波
		
		DELAY();										//延时24ms
		OPWAVE();										//允许PWM波形CPU输出子程序
		DELAY();										//延时24ms
		OPSPWM();										//启动SPWM波形发生子程序
		DSCD = EDSCD[0];								//显示模式F45? Default 2 0=显示设定频率，1=显示输出频率，2=显示位置百分数，3=显示位置脉冲数
		RT |= 1;										//RT.0=1 停止?
		GETPOS();										//调用当前门宽获得当前门位置脉冲数子程序
		VADCH();										//调用直流母线电压检测子程序--VDC
        AUTO();											//调用AUTO子程序，判断AUTO是否按下? 是手动模式(键盘控制) 还是自动模式?
		
        if(APPL & 0x01 != 0 && SD_FLG != 0) {			//0->上电显示最大频率值 模式则转RUNDIR
														//1->上电显示最大频率值 模式继续下面操作
			S = NUMBER * 1000 / EPULSE[0]; 				//否则AX1 = NUMBER * 1000 (放大1000倍) AX1=乘法的低16bit BX1=乘法的高16bit
														//AX1 = AX1 / 自学习总脉冲数  (AX1 = 除法的商   BX1=除法的余数)
														//S 应该是当前门位置的%比 (%都是放大1000倍的)
			J123();										//调用继电器输出子程序
		}  //end APPL.bit0 != 0 && SD_FLG != #0
		
		if(MST & 0x01 == 1) {							//MST.0=1? 关门方向 则跳DREV
			DBUF[5] &= 0xBF;
			DBUF[5] |= 0x80;							//准备点亮"关门LED"
		} else {
			DBUF[5] &= 0x7F;
			DBUF[5] |= 0x40;							//准备点亮"开门LED"
		}  //end MST.bit0 == 1
		
		KEYB();											//读当前键盘状态子程序
		AUTODY();										//调用自动显示子程序
		
		if(KEY & 0x40 == 0) {								//KEY.6=0? 是功能键
			DELAY2(200);                				//延时200 * 2.4ms=480ms
			CONTROL ^= 0x01;						//异或CONTROL和#00000001B ??

			GETS();										//把当前功能码对应的设置参数读到对应的RAM单元中	
		}  //end  KEY.bit6 == 0
		
		if(CONTROL.bit2 == 0) {							//CONTROL.2=0? 则跳转DIS_MD		2011.11.22 liuhao 显示的是参数或者是功能码
			if(CONTROL.bit0 == 0) { 					//CONTROl.0=0? 则跳转DIS_MD1   	2011.11.22 liuhao 显示参数
				PARS();		     						//查找到功能码对应设置参数处理子程序
				DISPAR();								//设置参数显示(带单位)子程序
					
				if(FUN == #11) {						//是F11吗? F11 是控制模式 COMD 是0->键盘控制 1-> 点动  2->外部端子?
					WRS();								//F11 控制模式功能码设置参数写入对应EEPROM 功能码单元子程序
				}
			} else {
				FUNS();									//调用功能码筛选处理程序
				DISFUN();								//调用显示经过筛选后的功能码子程序
			}  //end CONTROL.bit0 == 0
		} else {
			JTDC();                  					//调用显示 "BCD-"
		}  //end CONTROL.bit2 == 0
		
		DISPLY();										//间接(RT.2=1判断有效)数码管数据发送显示处理子程序

		if(APPL.bit0 == 1) {							//1->上电显示最大脉冲数目 则跳装ADCL_SP 可调频率模式F58
														//0->上电显示最大频率数目 则继续下面操作
			if(SD_FLG != 2 || (SD_FLG == 2 && RT.bit1 == 0)) {
				if(SD_FLG == 2 && RT.bit1 == 0) {
					OBSTCL();							//调用门力矩检查子程序
				}  //end SD_FLG == 2 && RT.bit1 == 0
				
				COMMAND();								//(只处理上电最大脉冲显示模式)命令处理子程序
				
				if(S > #0) {							//比较S 和 0%
					if(COMD != #0FFH) {					//COMD=#0FFH? 有控制命令输入
						RT &= #01111111B;	        	//只要有控制命令(GDV10只有开门，关门两个控制指令)则让RT.7=0  CLR AUTO ClOSE DOOR MODE
					}  //end COMD == #0FFH
				} else {
					RT &= #01111111B;	        		//只要有控制命令(GDV10只有开门，关门两个控制指令)则让RT.7=0  CLR AUTO ClOSE DOOR MODE
				} //end S > #0
				
				if(RT.bit0 == 0) {		    			//RT.0=0? 则转EMG1
					 SJMP	  EMG1 ;	 ??????????		
				} else {
					if(COMD.bit0 == 0) {				//是停止命令? 则转EMG0
						if(SD_FLG == #2) {				//核查自学习是否结束?
							FG = #LSP;					//FG = LSP(运算的最小频率HZ的高16bit)
							ADCL1();					//频率跳变点预处理--反转加减速处理子程序
							
							if(FOH > #LSP) {			//FOH= LSP(运算的最小频率HZ的高16bit)
								LJMP	  RUNL5			;跳转RUNL5  怎么跳，看着办吧？？？？？？？？？
							}  //end FOH > #LSP
						} else {
							SD_FLG = 0;					//如果自学习还没有结束，则强行设置为自学习开始阶
						}  //end SD_FLG == #2
						
						 SJMP	  EMG1       ;       ?????????????
					} else {
						if(SD_FLG == #2) {		//核查自学习是否结束?
							if(RT.bit7 == 1) { 		//RT.7=1? 跳AUTO_DC(自动关门?)
								if(MST.bit0 ==1) {		//MST.0=1? 关门方向 则转DC_2
									ADCL2();			//频率跳变点预处理--常规加减速处理子程序
									CLOSE();			//关门阶段位置对应速度计算子程序	
								} else {
									ADCL1();			//频率跳变点预处理--反转加减速处理子程序
									FG = #LSP;			//FG = LSP (运算最小频率高16bit HZ)
									
									if(FOH <= #LSP) {		//FOH= LSP (运算最小频率高16bit HZ)
										MST |= #00000001B	//MST.0=1 关门方向
									}
								}
								DC_1:	   SJMP	  RUNL5       ;看着办????
							} else {
								if(COMD.bit1 == 1) { DO_0		//COMD.1=1? 不是强迫关门命令则跳DO_0
									if(COMD.bit2 == 1) { DC_0		//COMD.2=1? 不是 开门命令 则转DC_0
										if(COMD.bit3 == 1) { STOP_0		//COMD.3=1?非关门命令 则转STOP_0
											if(S <= #50) {			//比较S和 5%  (%都是放大1000倍的)
												if(MST.bit0 == 0) { STOP		//MST.0=0? 开门方向 则转STOP
													  JH	  STOP			;DOCPT > RDOPT,则转STOP???????
												
													AL1 = ESTT[0];		//AL1 = ESTT (F12 停止模式0:自由停止 1:减速停止)
													if(AL1.bit0 == 1) { STOP_A		//如果AL1.0=1 是减速停止 则转STOP_A
														ADCL1();			//频率跳变点预处理--反转加减速处理子程序
														DC_BK();			//直流制动处理子程序
													} else {
														RT &= #11111110B;		//自由停止则让 RT.0=1	
													}  //end AL1.bit0 == 1
													SJMP	  FGC00     ??????????????????????
												} else {
													RT |= #01000000B;         //RT.6=1
													
													if(DOCPT > RDOPT[0]) {	 //DOCPT 和RDOPT(开门推进时间参考)比较
														JH	  STOP			;DOCPT > RDOPT,则转STOP???????
													} else {
														FG = DCV3[0];        //FG = DCV3 (关门换速位置#3对应的速度 F40 Hz) DC PUSH SPEED
														SJMP	  FGC00  ?????????
													}
												}
											} else {
												if(S > #95) {			//比较S和9.5% (%都是放大1000倍的)
													if(MST.bit0 == 0) {STOP_4		//MST.0=0? 开门方向 则转STOP_4
														RT |= #00100000B		//RT.5=1 关门?
														
														if(DOCPT > RDOPT[0]) {	    	//DOCPT 和RDOPT(开门推进时间参考)比较
															 SJMP	  FGC00   ?????????
														} else {
															FG = DOV3[0];            //FG = DOV3 (开门换速位置#3对应的速度 F36 Hz) DO PUSH SPEED
															SJMP	  FGC00      ??????????
														}
													} else {
														SJMP	  STOP
													}
												} else {
													RT &= #10011111B         		//S <= 9.5% 则让RT,5,6=0
													SJMP	  STOP
												}
											}
										} else {  //是关门命令则
											if(MST.bit0 == 1) { DC_2		//MST.0=1? 关门方向 则转DC_2
												ADCL2();			//频率跳变点预处理--常规加减速处理子程序
												CLOSE();			//关门阶段位置对应速度计算子程序
											} else {
												ADCL1();			//频率跳变点预处理--反转加减速处理子程序
												FG = #LSP;			//FG = LSP (运算最小频率高16bit HZ)
												if(FOH <= #LSP) {		//FOH= LSP (运算最小频率高16bit HZ)
													MST |= #00000001B;	//MST.0=1 关门方向
												}
											}
											DC_1:	   SJMP	  RUNL5           ?????????????
										}
									} else {
											;如果是 开门命令 则继续下面处理
														;****自动开关门处理**************
										if(MST.bit0 == 0) { DO_2		//MST.0=0? 是开门 则转DO_2
											ADCL2();			//频率跳变点预处理--常规加减速处理子程序
											OPEN();			 	//开门阶段位置对应速度计算子程序
											SJMP	  DO_1
										} else {
											ADCL1();			//开门 频率跳变点预处理--反转加减速处理子程序
											FG = #LSP;			//FG = LSP (运算最小频率高16bit HZ)
											
											if(FOH >= #LSP) {		//FOH和LSP比较(运算最小频率高16bit HZ)
												MST &= #11111110B;	//MST.0=0 开门方向
											}
										}												
										 SJMP	  RUNL5 ??????????????
									}  //end COMD.bit2 == 1
								} else {
									if(MST.bit0 == 1) {ND_2		//是强迫关门命令, MST.0=1?(是关门方向?)是关门 则跳ND_2(GDV10没有强迫关门)理论不会执行，实际仿真能执行，奇怪
										ADCL2();			//频率跳变点预处理--常规加减速处理子程序
										CLOSE();			//关门阶段位置对应速度计算子程序
										if(FG <= EFG2[0]) {            //比较FG 和 EFG2 (F48 强迫关门速度Hz 出厂12Hz)? NRDING SPEED
											ADCL2();			//频率跳变点预处理--常规加减速处理子程序
										} else {
											ADCL1();			//如果FG > EFG2 频率跳变点预处理#1子程序
											FG = EFG2[0];		//FG = EFG2 (F48 强迫关门速度Hz
										}
										ND_1:	   SJMP	  DC_1 ?????????
									} else {
										ADCL1();				//开门则 频率跳变点预处理--反转加减速处理子程序
										FG = #LSP;				//FG = LSP(运算最小频率高16bit HZ)
										if(FOH <= #LSP) {		//FOH和LSP比较(运算最小频率高16bit HZ)
											MST |= #00000001B;	//MST.0=1  关门方向
										}
									}  //end MST.bit0 == 1
									DC_1:	   SJMP	  RUNL5       ???????????????
								}  //end COMD.bit1 == 1
							}  //end RT.bit7 == 1
						} else {
							STUDY();					//调用找门宽自学习处理子程序
							LJMP	  FGC00
						}  //end SD_FLG == #2
					}  //end COMD.bit0 == 0
				}  //end RT.bit0 == 0
			} else {
				if(RT.bit1 == 1) { XRUNL3		//RT.1=1? 力矩监控标志位有效 则跳转XRUNL3
					RT &= #01111111B;			//RT.7=0
		
					if(S <= #960) {		;比较S 和960 就是比较S 和 96% (%都是放大1000倍的)
						if(OBPT > #300) {		//OBPT <300? 力矩监控推进时间 
												//跳转AUTO_OC (自动开关门?)
							RT &= #11111101B;			//RT.1=0 力矩监控标志位无效
							OBPT = #0000H;				//OBPT=#0000H (力矩监控推进时间)
						} 
						
					} else {
						RT &= #11111101B;			//RT.1=0 力矩监控标志位无效
						OBPT = #0000H;				//OBPT=#0000H (力矩监控推进时间)
					}	
				
					if(MST.bit0 == 0) {DO_2		//MST.0=0? 是开门 则转DO_2
						ADCL2();				//频率跳变点预处理--常规加减速处理子程序
						OPEN();					//开门阶段位置对应速度计算子程序
					} else {
						ADCL1();				//开门 频率跳变点预处理--反转加减速处理子程序
						FG = #LSP;				//FG = LSP (运算最小频率高16bit HZ)
						
						if(FOH <= #LSP) {		//FOH和LSP比较(运算最小频率高16bit HZ)
							MST &= #11111110B;	//MST.0=0 开门方向
						}
					}
							
					SJMP	  RUNL5   ????????????????
				}	// end RT.bit1 == 1   这部分判断条件还有待改变？？？？？？？？
			} //end SD_FLG != 2 || (SD_FLG == 2 && RT.bit1 == 0)
		} else {
			if(RT.bit0 == 1) {		            		//RT.0=1 则跳转NORR1
				BOOST();								//调用低频力矩补偿子程序 -->VTRQ
				VANDF();								//调用V/F特性处理子程序  -->KV1
				FGC = EFG[0];							//把当前系统的设定(给定)运行频率(放大100倍 )赋予FGC
				LJMP	  RUNL			;跳转RUNL  要往上面跳，想什么办法跳过去？？？？？？？？？？？？？？
			} else {
				if(SD_FLG == #2) {						//核查自学习是否结束?
					FG = #LSP;							//FG = LSP(运算的最小频率HZ的高16bit)
					ADCL1();							//频率跳变点预处理--反转加减速处理子程序
					if(FOH > #LSP) {					//FOH= LSP(运算的最小频率HZ的高16bit)
						 LJMP	  RUNL5			;跳转RUNL5  怎么跳，看着办吧？？？？？？？？？
					}  //end FOH > #LSP		
				} else {
					SD_FLG = 0;							//如果自学习还没有结束，则强行设置为自学习开始阶段
				}  //end SD_FLG == #2
					
				FUN	= 0;								//功能码清零
				RT &= #00001110B;						//RT.1=RT.2=RT.3=1
				CONTROL = #00H;							//CONTROL=#00H
				STWAVE();								//禁止PWM波形CPU输出子程序
				STSPWM();								//停止SPWM波形发生子程序
				COMD = #0FFH;							//COMD=#0FFH 清楚所有当前的控制命令
				DELAY();                 				//延时24ms
				LJMP	  STAY0		;停止命令则 再次进入外围常规静态主循环程序!???????????????????????
			}  //end RT.bit0 == 1
		}  //end APPL.bit0 == 1		
	}  //end KEY.bit2 == 0 || (KEY.bit3 == 0 && KEY.bit2 == 0*/

	
}



interrupt void cpu_timer0_isr(void)
{
   	//CpuTimer0.InterruptCount++;
	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Toggle GPIO34 once per 500 milliseconds

  	//Acknowledge this interrupt to receive more interrupts from group 1
   	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}



void FORMAT()
{
	KEY = 0xFF;									//把当前键盘读取数值设为0FFH
	TERMAL = 0xFF;								//把当前TERMAL设置0FFH,这个TERMAL代表整机的端子输入信号状态标志 低四位有效
	CONTROL	= 0;								//CONTROL=#00H 控制标志寄存器 高有效
	DBUF[4] = 0xFF;	        					//把指示LED全部灭掉（其实都没有必要，因为OPEN为高，数码管板没有电）
}


/*****************************************************************
*           延时子程序
*标准DELAY为10倍基本延时，16MHZ下基本延时DELAY2为：最小状态周期和30x125nsx666=2.4ms
*IN  PARAMETER: NONE /DLY                DELAY为2.4x10=24ms
*MID PARAMETER: DLYS
*OUT PARAMETER: NONE
*16MHZ 状态周期是125ns
*12MHZ 状态周期是167ns
*8MHZ  状态周期是250ns
******************************************************************/
//这部分需要仔细计算DLEAY的时间问题
void DELAY()
{
	unsigned i,j;
	
	for(i = 0; i < 200; i++) {
		for(j = 0; j < 100; j++) ;
	}
}
/*
DELAY:	  LD   DLY,#10
DELAY1:   NOP
DELAY2:   LD   DLYS,#666                  ;4-10状态 1S延时
DELAY3:   DEC  DLYS                       ;3状态
	  CMP  DLYS,#0                    ;4-9状态
	  JNE  DELAY3                     ;4状态
	  DEC  DLY                        ;3状态
	  CMP  DLY,#0                     ;4-9状态
	  JNE  DELAY2                     ;8状态
	  RET*/


/******************************************************************
*            Translation待显示参量转化子程序 (16进制-10进制)
*IN  PARAMETER: AX1(需要转换的16bit参数)
*MID PARAMETER: BX1
*OUT PARAMETER: CL1,CH1,DL1,DH1,EL1
*******************************************************************/
/*void TRANS(Uint16 AX1)
{     
	Uint16 BX1;
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	CL1 = ADISP[BX1];          				//显示AX1的第一位
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	CH1 = ADISP[BX1];	  					//显示AX1的第二位
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	DL1 = ADISP[BX1];						//显示AX1的第三位
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	DH1 = ADISP[BX1];						//显示AX1的第四位
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	EL1 = ADISP[BX1];						//显示AX1的第五位
			//0   1	2   3	4   5	6   7
//ADISP:	 DCB   03H,9FH,25H,0DH,99H,49H,41H,1FH	;0-7
//	 DCB   01H,09H,11H,0C1H,63H,85H,61H,71H ;8-F
}*/


/************************************************************
*           FUNCTION PROCESSION 功能码筛选处理子程序
*IN  PARAMETER:  KEY(7EH)
*MID PARAMETER: FUN(5CH)
*OUT PARAMETER: FUN(5CH)
*F20=8080的时候FUNCTION是正常全部显示并按照约定的调整范围(不对用户开放)
*否则屏蔽所有的保留功能显示,FUN 01- 02, FUN 15-17, FUN 13-0F, FUN 0B-0D, FUN 38-3A 都不显示
*************************************************************/
/*void FUNS() {
    if(ESAV[0] == #8080) {              			//检查F20是否是8080,否则不显示保留功能码
		if(KEY.5 == 0) {							//不是UP键 则转FUNS02
			FUN += 1;
			if(FUN > #FUNM) {						//功能码循环加，大于最大功能码则置零
				FUN = 0;
			}
		}
		
		if(KEY.4 == 0) {							//不是DOWN键 则返回
			FUN -= 1;								//功能码自减，小于0时则置为最大功能码数
			IF(FUN > #FUNM) {
				FUN = #FUNM;
			}
		}
	} else {										//保留部分功能码,FUN 01- 02, FUN 15-17, FUN 13-0F, FUN 0B-0D, FUN 38-3A 都不显示
		if(KEY.5 == 0) {
			if(FUN == 0 || FUN == 0x06 || FUN == 0x0A || FUN == 0x0E \
				|| FUN == 0x14 || FUN == 0x2C || FUN == 0x2F || FUN == 0x37) {			
				
				FUN += 1;									
				if(FUN == 0x01) {					//屏蔽功能码FUN 01- 02
					FUN += 1;
				} else if(FUN == 0x0B || FUN == 0x15 || FUN == 0x38) {		//屏蔽功能码FUN 0B-0D, FUN 15-17, FUN 38-3A
					FUN += 2;
				} else if(FUN == 0x0F) {			//屏蔽功能码FUN 13-0F
					FUN += 4;
				}
			} else {
				FUN += 1;
				if(FUN > #FUNM) {					//功能码循环加，大于最大功能码则置零
					FUN = 0;
				}
			}
		}
		
		if(KEY.4 == 0) {
			if(FUN == 00) {							//屏蔽功能码 FUN 38-3A	
				FUN = 0x37;
			} else if(FUN == 0x31 || FUN == 0x2E || FUN == 0x18 || FUN == 0x14 \
				|| FUN == 0x0E || FUN == 0x08 || FUN == 0x03) {
				
				FUN -= 1;
				if(FUN == 0x17 || FUN == 0x0D) {		//屏蔽功能码 FUN 15-17   FUN 0B-0D
					FUN -= 2;
				} else if(FUN == 0x13) {				//屏蔽功能码 FUN 13-0F
					FUN -= 4;
				} else if(FUN == 0x02) {				//屏蔽功能码FUN 01- 02
					FUN -= 1;
				}
			}
		}
	}
}         */


/************************************************************
*            显示功能码子程序
*IN  PARAMETER: FUN
*MID PARAMETER: AX1,BX1  先显示引导的"b" "F"后+功能码
*OUT PARAMETER:
*************************************************************/
/*void DISFUN() 
{
	DBUF1 = #0C1H;           							//"b"                #71H F
	DBUF2 = #71H;										//"F"                0FFH
	AX1	= FUN;
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	DBUF4 = ADISP[BX1];									
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	DBUF3 = ADISP[BX1];
	DBUF5 |= #3FH;										//#0011 1111B 意味保持当前 DO DC LED的状态
}*/


/****************************************************************
*         查找到功能码对应设置参数处理子程序
*IN  PARAMETER: KEY,FUN,AX8(当前功能码的设置参数)
*MID PARAMETER: AX1(AL1,AH1),BX1,DX1
*OUT PARAMETER: AX8(当前功能码对应的合适的设置参数)
*****************************************************************/
/*void PARS()
{
	AX1 = 2*FUN;
	BX1 = AX8;
	
	if(KEY.bit5 == 0) {									//是UP键则装PARD1(UP键=#1101 1111B)
		DX1 = UPPAR[AX1];								//UP键 则把当前功能码的参数上限放入DX1中,处理程序PARU1
		
		if(FUN == 0) {
			BX1 = EFG[0];								//把F00 设定( 给定频率 )频率(Hz 放大100倍)存放的参数放入BX1中
			BX1 += 1;
			if(BX1 > FOM) {								//如果频率大于FOM最高运行频率，则将当前频率设为最高频率
				BX1 = FOM;
			} else {
				if(BX1 < FLO) {							//如果当前频率小于最小频率，则将当前频率设为最小频率
					BX1 = FLO;
				}
				EFG[0] = BX1;
				AX8 = BX1;
			}
		} else {
			BX1 += 1;
			
			if(BX1 > DX1) {								//如果当前参数大于最大频率，则把当前参数设为最大参数
				BX1 = DX1;
			}
			AX8 = BX1;
		}
	} else if{											//是DOWN 键,(DOWN键= #1110 1111B)
		DX1 = INPAR[AX1];								//读出当前功能码的下限参数到DX1
		if(FUN == 0) {									//把F00 设定( 给定频率 )频率(Hz 放大100倍)存放的参数放入BX1中
			BX1 = EFG[0];
			BX1 -= #1;
			if(BX1 < FLO) {								//防止设置的频率超出FLO - FOM之间
				BX1 = FLO;
			} else {
				if(BX1 > FOM) {
					BX1 = FOM;
				}
			}
			
			EFG[0] = BX1;
			AX8 = BX1;
		} else {
			BX1 -= #1;
			if(BX1 < DX1) {
				BX1 = DX1;
			}
	  
			if(BX1 > #65530) {							//自减溢出？ 为什么是65530？
				BX1 = DX1;
			}
			AX8	= BX1;
		}
	}
}*/
	  
/***************************************************
*            设置参数显示(带单位)子程序
*IN  PARAMETER: FUN,AX8
*OUT PARAMETER: NONE
****************************************************/
//DFREX:	  LJMP   DFRQCYB				//这句话是干什么的， 先不用管
/*
void DISPAR() 
{ 										//------------- FREQUENCY
	if(FUN != #0) {                		//看是不是F00? FG,FOUT
		if(FUN <= 3) {
			JNH    DFRQCYB
		} else {
			if(FUN < 33 || FUN > 40) {
				JLT    DAA1 
				if(FUN == 16) {
					JE     DFRQCYB
				} else if(FUN == 51) {
					JE     DFRQCYB 
				} else if(FUN == 19) {
					JE     DFRQCYC 
				}
			} else {
				SJMP   DFRQCYB
			}
		}
	} else {*/
	
	/*}
	  JE     DFRQCYA
	  CMP    FUN,#3                 ;看是不是F03? FLO,FOM,F1M
	  JNH    DFRQCYB
	  CMP    FUN,#33                ;如果>F03,看是不是F33?
	  JLT    DAA1                   ;小于F33,调转DAA1 ( F03< <F33)
	  CMP    FUN,#40
	  JGT    DAA1
	  SJMP   DFRQCYB
DAA1:	  CMP    FUN,#16                ;看是不是F16?       FBK
	  JE     DFRQCYB
	  CMP    FUN,#51		;看是不是F51?
	  JE     DFRQCYB                ;CLOSE BASE FREQ.
	  CMP    FUN,#19		;看是不是F19?
	  JE     DFRQCYC                ;FC
	  CMP    FUN,#46		;看是不是F46?
	  JLT    GGG1			;小于跳转GGG1
	  CMP    FUN,#49		;看是不是F49?
	  JGT    GGG1			;大于F49,跳转GGG1
	  SJMP   DFRQCYB
GGG1:	  LJMP   NEXT_RAG		;SECOND第二段显示参数子程序

DFRQCYA:  JBS    APPL,0,DDOOF           ;APPL.0=1上电显示最大脉冲数目模式，则转DDROOF F58 可调频率模式 (1:上电显示最大脉冲数目,0:上电显示最大频率数)
	  JBS    RT,0,DFRQCYA1
	  SJMP   DFG0

DDOOF:	  CMPB   SD_FLG,#2		;核查自学习是否结束?
	  JE     DFRQCYA1
	  LD     AX1,NUMBER		;AX1 = 当前获得的编码器脉冲数
	  LJMP   PEER1			;直接参数(无单位)显示子程序

DFRQCYA1: CMPB   DSCD,#0		;显示模式F45是不是0 显示设定频率?
	  JE     DFG0
	  CMPB   DSCD,#1		;显示模式F45是不是1 显示输出频率?
	  JE     DFOH0
	  CMPB   DSCD,#2		;显示模式F45是不是2 显示位置百分数?
	  JE     DWEIZ
	  LD     AX1,NUMBER		;显示模式F45是3 位置脉冲数?
	  LJMP   DNOUNIT		;带F63识别的参数(无单位)显示子程序

DWEIZ:	  LD     AX1,S			;2011-10 Johnson 显示位置百分数 把当前门宽% 给AX1
	  LJMP   DXX1X0			;无输入参数的脉冲百分数显示子程序

DFOH0:	  LD     AX1,FOH		;显示输出频率
	  CMP    AX1,#LSP		;AX1 和LSP比较(运算最小频率HZ的高16bit)
	  JH     DFOH1
	  CLR    AX1
DFOH1:	  LJMP   DISFRE			;频率参数显示子程序
DFG0:	  LD     AX1,FG		        ;2011-10 Johnson 显示设定( 给定 )频率  把当前最大频率赋予AX1 "0"HZ MODE
	  CMP    AX1,#LSP		;AX1 和LSP比较(运算最小频率HZ高16bit)
	  JH     DFG1
	  CLR    AX1
DFG1:	  LJMP   DISFRE			;频率参数显示子程序

DFRQCYB:  LD     AX1,AX8		;**.** HZ--***.*HZ
	  LJMP   DISFRE			;调涨频率显示，频率参数显示子程序
DFRQCYC:  LD     AX1,AX8		;**** HZ
	  LCALL  DNOUNIT		;带F63识别的参数(无单位)显示子程序
	  ORB	 DBUF5,#00011111B
	  ANDB	 DBUF5,#11011111B       ;"HZ"LED亮
	  RET
}

;****************************************
;   FREQUENCY DISPLAY 频率参数显示子程序
;IN  PARAMETER:
;MID PARAMETER:
;OUT PARAMETER: DBUF1,2,3,4,5
;****************************************
DISFRE:   LCALL  TRANS			;Translation待显示参量转化子程序 16进制转10进制
	  CMPB	 EL1,#03H
	  JNE	 DISFRE2
	  CMPB	 DH1,#03H
	  JNE	 DISFRE1
	  LDB	 DH1,#0FFH
DISFRE1:  LDB	 DBUF1,DH1	      	;**.** HZ  每次要判断首位是不是"0"字符，首位"0"不显示
	  ANDB	 DL1,#0FEH
	  LDB	 DBUF2,DL1              ;与#0FEH，让DL1带小数点
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  ORB	 DBUF5,#00011111B
	  ANDB	 DBUF5,#11011111B      	;HZ
	  RET
DISFRE2:  LDB	 DBUF1,EL1	        ;***.* HZ
	  LDB	 DBUF2,DH1
	  ANDB	 DL1,#0FEH
	  LDB	 DBUF3,DL1              ;与#0FEH，让DL1带小数点
	  LDB	 DBUF4,CH1
	  ORB	 DBUF5,#00011111B
	  ANDB	 DBUF5,#11011111B	;HZ
	  RET
;******************************************************
;         SECOND第二段显示参数子程序
;IN  PARAMETER: FUN
;MID PARAMETER:    好像是把功能码分成两组来区分处理
;OUT PARAMETER:
;******************************************************
NEXT_RAG: CMP    FUN,#5                	;看是不是F05?     ACL1
	  JLT    GGG3                  	;<F05,跳转GGG3
	  CMP    FUN,#10               	;看是不是F10?     DCL1
	  JGT    GGG3                  	;>F10,跳转GGG3
	  LJMP   SECONDX
GGG3:	  CMP    FUN,#17               	;看是不是F17?     TBK
	  JE     GGG4
	  CMP    FUN,#57               	;看是不是F57?
	  JE     GGG4
	  CMP    FUN,#42               	;看是不是F42?
	  JLT    GGG5                  	;<F42,跳转GGG5
	  CMP    FUN,#44               	;看是不是F44?
	  JGT    GGG5                  	;>F44,跳转GGG5
GGG4:	  LJMP   SECOND
	;-----------**.* %
GGG5:	  CMP    FUN,#4                 ;看是不是F04?
	  JE     GGG6
	  CMP    FUN,#52                ;看是不是F52?
	  JE     GGG6
	  CMP    FUN,#53                ;看是不是F53?
	  JE     GGG6
	  CMP    FUN,#54                ;看是不是F54?
	  JE     GGG6
	  CMP    FUN,#18                ;看是不是F18?
	  JE     GGG6
	  CMP    FUN,#14                ;看是不是F14?
	  JE     GGG6
	  CMP    FUN,#41                ;看是不是F41?
	  JE     GGG6
	  CMP    FUN,#25                ;看是不是F25?
	  JLT    GGG7                   ;<F25,跳转GGG7
	  CMP    FUN,#32                ;看是不是F32?
	  JGT    GGG7                   ;>F32,跳转GGG7
GGG6:	  LJMP   DXX1XA	              	;**.*% 位置脉冲百分数参数显示子程序
GGG7:	  LJMP   NOUNIT                 ;大于F32,小于F25处理, 调用带显示参数(无单位)入口的显示子程序

DPERCNT:  LD	 AX1,AX8              	;*** %
DPCTW:	  LCALL  TRANS		      	;Translation待显示参量转化子程序 16进制转10进制
	  CMPB	 DH1,#03H
	  JNE	 HIDAT
	  LDB	 DH1,#0FFH
	  CMPB	 DL1,#03H
	  JNE	 HIDAT
	  LDB	 DL1,#0FFH
	  CMPB	 CH1,#03H
	  JNE	 HIDAT
	  LDB	 CH1,#0FFH
HIDAT:	  LDB	 DBUF1,DH1
	  LDB	 DBUF2,DL1
	; ANDB	CH1,#11111110B	        ;DIP
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  ORB	 DBUF5,#00111011B
	  ANDB	 DBUF5,#11111011B       ;%
	  RET

SECOND:   LD	 AX1,AX8
	  LCALL  DNOUNIT		;带F63识别的参数(无单位)显示子程序
SECOND1:  ORB	 DBUF5,#00111110B
	  ANDB	 DBUF5,#11111110B       ;点亮"SEC" LED  2011-10 仿真感觉应该是对的，但是与DO DC LED控制不符合
	  RET
SECONDX:  LD	 AX1,AX8
	  LCALL  TRANS			;Translation待显示参量转化子程序 16进制转10进制
	  CMPB	 DH1,#03H
	  JNE	 ASS1
	  LDB	 DH1,#0FFH
	  CMPB	 DL1,#03H
	  JNE	 ASS1
	  LDB	 DL1,#0FFH
ASS1:	  LDB	 DBUF1,DH1
	  LDB	 DBUF2,DL1
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  ANDB	 DBUF3,#0FEH	         ;***.* SEC
	  SJMP	 SECOND1

;******************************************
;   带显示参数(无单位)入口的显示子程序
;******************************************
NOUNIT:   LD     AX1,AX8		;把当前功能码设置参数-->AX1   ****

;******************************************
;	带F63识别的参数(无单位)显示子程序
;******************************************
DNOUNIT:  CMP	 FUN,#63		;功能码是不是63? 本软件最大功能码是58 ，所以本句无效。
	  JNE	 PEER1
	  ;LD	AX1,TIMER1[0]
	  LD	 AX1,VDC		;如果F63 则准备显示VDC参数 AX1 = 当前直流母线电压
;******************************************
;	直接参数(无单位)显示子程序
;******************************************
PEER1:	  LCALL  TRANS			;Translation待显示参量转化子程序 16进制转10进制(入口AX1，出口是EL1,DL1,CH1,CL1)
	  CMPB	 EL1,#03H               ;ADISP[03]="0" ，EL是转化后的首位，也是判断大数和小数的标志，EL不是"0"字符就是大数
	  JNE	 HINUT
	  CMPB	 DH1,#03H
	  JNE	 LONUT
	  LDB	 DH1,#0FFH              ;TRANS输出顺序是EL1,DH1,DL1,CH1,CL1, 首位'0"字符不显示
	  CMPB	 DL1,#03H               ;比较下一位是不是"0"字符? 如果是则需要再次不显示
	  JNE	 LONUT
	  LDB	 DL1,#0FFH
	  CMPB	 CH1,#03H               ;比较下一位是不是"0"字符? 如果是则需要再次不显示
	  JNE	 LONUT
	  LDB	 CH1,#0FFH
LONUT:	  LDB	 DBUF1,DH1
	  LDB	 DBUF2,DL1
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  RET

HINUT:	  ANDB	 DH1,#0FEH               ;**.** 与0FEH，让第二位带小数点
	  LDB	 DBUF1,EL1
	  LDB	 DBUF2,DH1
	  LDB	 DBUF3,DL1
	  LDB	 DBUF4,CH1
	  RET*/

/******************************************
*        位置脉冲百分数参数显示子程序
*IN  PARAMETER: AX8(要显示的参数)
*MID PARAMETER:
*OUT PARAMETER:
*******************************************/
/*
DXX1XA:   LD	 AX1,AX8		;AX1 = 当前要显示的参数

;****无输入参数的脉冲百分数显示子程序****

DXX1X0:   LCALL  TRANS	                ;Translation待显示参量转化子程序 16进制转10进制 **.*%
	  CMPB	 DH1,#03H               ;DH-DL-CH-CL
	  JNE	 DXX2X0
	  LDB	 DH1,#0FFH              ;首位如果遇到"0"字符要不显示
	  CMPB	 DL1,#03H
	  JNE	 DXX2X0
	  LDB	 DL1,#0FFH
DXX2X0:   LDB	 DBUF1,DH1
	  LDB	 DBUF2,DL1
	  ANDB	 CH1,#11111110B	        ;DIP 与0FEH,让CL1带小数点
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  ORB	 DBUF5,#00111011B	;HAD LED 顺序: DO  DC  Hz % SEC MIN MIC MIS (其中MIN MIC MIS HAD备用,GDV10不支持SEC MIN MIC MIS)
	  ANDB	 DBUF5,#11111011B       ;为什么会出现"%"亮?  仿真测试的确亮了！
	  RET*/

/******************************************************************
*	    当前功能码设置参数写入对应EEPROM 功能码单元子程序
*IN  PARAMETER: KEY,FUN,AX8(要写入的设置参数)
*MID PARAMETER:
*OUT PARAMETER: AX8(经过写入EERPOM后再次读回的设置参数)
*******************************************************************/
/*void WRS()
{
	AX1 = FUN + FUN;									//AX1 = FUN + FUN 根据功能码代号算出对应EEPROM物理地址
	if(KEY.bit7 == 1) {									//是写入键(#0111 1111B)则转WRS1
		if(FUN != #20) {								//当前功能码是F20吗? 主要是为了区分非F20功能码是不是当前支持擦写418或显示所有功能码8080
			BX1 = ESAV[0];								//如果不是F20，需要把内存ESAV 数据保存参数给BX1
			if(BX1 == #8080 || BX1 == #418) {			//是8080?
														//F20=8080 则要全部显示功能码	
				AX9 = FUN + FUN;
				AX1 = AX8;
				EWRE();		     						//调用EEPROM写入程序
				DELAY2(2);								//Add for AT93C66
				ERDE();									//写完后读出来

				E2M[AX9] = AX1;							//再次保存到该功能码对应的RAM单元
				AX8	= AX1;								//AX8 = 写入EEPROM后读回的设置参数

				DBUF1 = #61H;         					//E
				DBUF2 = #99H;           				//4 RESTORE FACETORY OK LABLE->E200 #25H
				DBUF3 = #0FDH;          				//-
				DBUF4 = #0FDH;          				//-
				DBUF5 |= #3FH;           				//保持当前LED约定方式
				BL1 = #50;
				while(BL1 != 0) {
					DISPLY();							//间接(RT.2=1判断有效)数码管数据发送显示处理子程序
					DELAY();
					BL1 --;
				}
			} else {
				AX1 = FUN + FUN;						//如果ESAV不是8080 也不是 418，则在保持屏蔽功能码条件下,不支持写入
				BX1 = E2M[AX1];							//把对应功能码的RAM单元参数给BX1
				AX8	= BX1;								//AX8 = 当前功能码的设置参数，但是不支持写入eeprom，并显示"-Err"故障
				DBUF1 = #0FFH;							//-
				DBUF2 = #61H;							//E
				DBUF3 = #0F5H;							//r
				DBUF4 = #0F5H;							//r
				DBUF5 |= #3FH;							//保持当前LED约定方式

				BL1 = #50;
				while(BL1 != 0) {
					DISPLY();							//间接(RT.2=1判断有效)数码管数据发送显示处理子程序
					DELAY();
					--BL1;
				}
			}
		} else {										//F20=8080 则要全部显示功能码
			AX9 = FUN + FUN;
			AX1 = AX8;
			EWRE();		     						//调用EEPROM写入程序
			DELAY2(2);								//Add for AT93C66
			ERDE();									//写完后读出来

			E2M[AX9] = AX1;							//再次保存到该功能码对应的RAM单元
			AX8	= AX1;								//AX8 = 写入EEPROM后读回的设置参数

			DBUF1 = #61H;         					//E
			DBUF2 = #99H;           				//4 RESTORE FACETORY OK LABLE->E200 #25H
			DBUF3 = #0FDH;          				//-
			DBUF4 = #0FDH;          				//-
			DBUF5 |= #3FH;           				//保持当前LED约定方式
			BL1 = #50;
			while(BL1 != 0) {
				DISPLY();							//间接(RT.2=1判断有效)数码管数据发送显示处理子程序
				DELAY();
				BL1 --;
			}
		}
	}
}*/


/*************************************************************
*         读当前键盘状态子程序
*IN  PARAMETER: KEY(上次主循环读到键值)
*MID PARAMETER: AL1,CL1
*OUT PARAMETER: KEY(本次子程序读到键值),RT,TERMAL
**************************************************************/
void KEYB()
{
	AL1 = read_key();								//读P0口,P0.0/0.1/0.2/0.3代表键盘输入
	KEY138();                  						//AL1 为本地读到的正确键值低四位有效 1111XXXXB 键盘数值分析转译子程序
	
	if(AL1 != KEY) {								//比较本次读到的键盘数值和上次的是否一样？
		DELAY();									//如果不一样，则延时后再读，再比较，相当于键盘的软件滤波
		AL1 = read_key();

		KEY138();
		if(AL1 != KEY) {
			KEY = AL1;								//如果经过滤波的确是新的键盘数值，则更新KEY
		}
	}
	
	if(APPL & 0x01 == 0) {							//检查当前的显示是最大脉冲数还是最大频率数? APPL.bit0 == 0 1->上电显示最大脉冲数目,0->上电显示最大频率值
		if(KEY & 0x08 == 0) {						//Key.3=1就代表当前不是软复位和停止键	KEY.bit3 == 0
			RT &= 0xFE;								//如果是软复位或停止键，则让RT=#1111 1110B
		}
	} else {
		AL1 = read_key();							//AL1 = P0_PIN[0]
		CL1 = E2M[EEXCM];							//CL1 = EEXCM (F15外部通信模式 固定为1)
		if(CL1 & 0x01 == 0x01) {             		//EEXCM=0 则转EXCM1  CL1.bit0 == 1
			AL1 = ~AL1;								//EEXCM=1,  AL1取反，由于IN4，IN3是通过反相器输入到CPU的，故而取反后代表真实输入 15/12-1999
		}
		
		AL1 = AL1 >> 4;                  			//右移4位，相当于把端子控制输入状态移到AL1的低4位
        AL1 |= 0xF3;          						//取IN4，IN3		AL1 |= #11110011B; 
		if(AL1 != TERMAL) {							//比较本次结果和上次TERMAL保存是否相等?
			DELAY();								//不相等，则延时24ms
			AL1 = read_key();						//再次读  AL1 = P0_PIN[0]  相当于键盘的软件滤波
			
			if(CL1 & 0x01 == 0x01) {             	//EEXCM=0 则转EXCM2		CL1.bit0 == 1
				AL1 = ~AL1;                    		//EEXCM=1,  AL1取反，由于IN4，IN3是通过反相器输入到CPU的，故而取反后代表真实输入 15/12-1999
			}
			
			AL1 = AL1 >> 4;                  		//右移4位，相当于把端子控制输入状态移到AL1的低4位
			AL1 |= 0xF3;		          			//取IN4，IN3	AL1 |= #11110011B;
			
			if(AL1 != TERMAL) {              		//比较本次结果和上次TERMAL保存是否相等?
				TERMAL = AL1;              			//把移位后的外部端子输入给TERMAL标志
			}
		}
	}
	
}

/*读入键值，并将键值放入变量temp返回*/
unsigned char read_key()
{
	unsigned char temp = 0xFF; 
	
   if(GpioDataRegs.GPADAT.bit.GPIO31 == 1)  		//当有键按下时，就把键值放在temp的低位
   		temp &= 0xFE;
   if(GpioDataRegs.GPBDAT.bit.GPIO32 == 1)  		//当有键按下时，就把键值放在temp的第二位
   		temp &= 0xFD;
   if(GpioDataRegs.GPBDAT.bit.GPIO33 == 1)  		//当有键按下时，就把键值放在temp的第三位
   		temp &= 0xFB;
   if(GpioDataRegs.GPBDAT.bit.GPIO34 == 1)  		//当有键按下时，就把键值放在temp的第四位
   		temp &= 0xF7;
   	
   	return temp; 									//返回值
}


/**********************************************************
*          自动显示子程序
*IN  PARAMETER: KEY CONTROL,DYCT
*OUT PARAMETER: CONTROL
***********************************************************/
/*void AUTODY()
{
	if(KEY.bit5 == 1 && KEY.bit4 == 1) {							//是UP键则跳转AU1处理程序
		CONTROL &= #11111101B;										//如果不是UP也不是DOWN键盘则把CONTROL.1=0
		DYCT = #0;													//DYCT赋于0；让DYCT重新计数
	} else {
		if(CONTROL.bit1 == 1) {           							//CONTROL.1=0 则转AU4
			if(DYCT <= #160) {               						//DYCT在TIMER2中加1，则算出160对应 160 x 16ms = 2.56秒
				DELAY2(8);
			}
		} else {
			DELAY2(200);
			CONTROL &= #00000010B;								//将CONTROL与#02H字节逻辑或
		}
	}
}*/


/***************************************************
*                显示 "DCB-"子程序
*IN  PARAMETER:
*OUT PARAMETER: DBUF1,DBUF2,DBUF3,DBUF4
****************************************************/
/*void JTDC() 
{
	DBUF1 = #85H;					//"B"
	DBUF2 = #63H;					//"C"
	DBUF3 = #0C1H;					//"D"
	if(FRSH.bit4 ==1) {            	//目的是让最后一位出现闪烁
		DBUF4 = #0FFH;				//NONE
	}

	DBUF4 = #0FDH;					//"-"
}*/
	  
/****************************************************
*               SPWM_WG 波形运算中断服务子程序
*IN  PARAMETER:
*MID PARAMETER: 在本程序初始化中是中心对称方式，向上计数
*OUT PARAMETER: 每个载波周期结束发起一个中断，所以只有发
*       生载波周期改变，否则每次都是一个载波周期进入一次
*****************************************************/
/*
SPWM:	  PUSHF                     	;6
	  EI			    	;2
	  XORB	 NEWVECT,#1		;	NEWVECT 异或00000001B,每次进来分别进入SPWM0 和 SPWM1
					;       进来算出当前矢量角度和Vrms，下次进来算出Ta,Tb,Tc推动PWM占空比变化
	  JBC	 NEWVECT,0,SPWM0	;	NEWVECT=0 则跳转SPWM0
	  SJMP	 SPWM1			;	否则跳转SPWM1

SPWM0:	  SUB	 AX,FGC,FOH	     	;5	AX = FGC - FOH (当前运行频率 - 上次运行频率)
	  JH	 INC11		    	;8	>0,则跳转INC11
	  NEG	 AX		    	;3	如果FGC-FOH<0, 则AX按为取反，符号改变，绝对数值不变
	  CMP	 AX,DECFOH	    	;4	比较AX 和DECFOH
	  JH	 DEC21			;	如果AX > DECFOH, 则跳转DEC21
	  LD	 FOH,FGC		;	如果AX < DECFOH, 则FOH = FGC
	  SJMP	 ANGAL1		    	;SUM=28 跳转ANGAL1

DEC21:	  SUB	 FOL,DECFOL          	;4+8(JH)FOL = FOL - DECFOL ?
	  SUBC	 FOH,DECFOH          	;4	FOH = FOH - DECFOH - 进位为0继续减1
	  SJMP	 ANGAL1	            	;7	跳转ANGAL1
INC11:	  CMP	 AX,INCFOH		;	比较AX和INCFOH
	  JH	 INC21			;	如果AX > INCFOH,则跳转INC21
	  LD	 FOH,FGC		;	如果AX <=INCFOH,则FOH = FOH + FGC
	  SJMP	 ANGAL1			;	跳转ANGAL1
INC21:
	  ADD	 FOL,INCFOL	    	;4	FOL = FOL + INCFOL
	  ADDC	 FOH,INCFOH	    	;4	FOH = FOH + INCFOH + 进位标志

ANGAL1:   MULU	 CX,KV1,FOH	    	;14	CX  = KV1 * FOH  (KV1是根据V-F曲线算出来的V数值)  CX=乘法的低16bit DX=乘法的高16bit
	  SHLL	 CX,#2		    	;7+2	CX 左移2位，CX = CX * 4  相当于(KV1 * FOH ) *4    双字移位，CX，DX同时左移
	  ADD	 AX,DX,VTRQ	    	;4	AX  = DX + VTRQ (VTRQ是算出来的当前力矩补偿电压数值)
					;	AX  = ((KV1 * FOH)*4 / 65536 ) + VTRQ   [DX 是 KV1 * FOH 的高16bit, 就好比是KV1 * FOH 右移16位 就是DX]

	  LD	 DX,AX			;	DX  = AX = ((KV1 * FOH)*4 / 65536 ) + VTRQ

	  CLR	 CX			;	CX =#0000H
	  SHRL	 CX,#7			;	双字右移7次，相当于把DXCX同时右移7次，CX = DX / 128

	  DIVU	 CX,VDC			;	CX = (DX/128) / VDC 的商，DX = (DX/128) / VDC 的余数 (VDC 是当前直流母线电压采样数值)
					;       CX = (((KV1 * FOH)*4 / 65536 ) + VTRQ ) /(128 * VDC) 的商 (s)


					;	VDC=(1023*真实采样母线电压数值)/5 (V) (为5V 采样参考电压 10bitAD )

	  LD	 VRMS,CX		;	Vrms =(((KV1 * FOH)*4 / 65536 ) + VTRQ ) /(128 * VDC) 的商  (Vrms 是电压的均方数值)

	  CMP	 VRMS,TZ	    	;4	比较Vrms和TZ大小 为什么?
					;	Vrms=(Uref/Vdc)TZ=mTZ 就是比较m(调制深度)是否>1 ?

	  JNH	 JJX1		    	;4	如果Vrms<=TZ 则跳转JJX1
	  LD	 VRMS,TZ	    	;4 SUM=70 如果调制深度>1 ,则让调制深度=1，就是Rrms=TZ

JJX1:	  MULU	 AX,FOH,DFINR1	    	;14	AX  = FOH * DFINR1  AX1=乘法的低16bit BX1=乘法的高16bit
	  SHLL	 AX,#3 ;SPWM0 MODE  	;7+5	AX  = (BX AX ) * 8 (左移3次相当于* 8) 的低16位
					;	SPWM0 MODE 就是中心对准PWM模式，这种模式得到的PWM波产生的谐波小 AC电机常用这个方式
	  ADD	 BX,BX			;	BX  = BX * 2 = ((FOH * DFINR1 *8 )/65536) * 2   BX是 FOH * DFINR1 的高16位，相当于FOH * DFINR1右移16位

	  ADD	 FIA,BX 	    	;4	FIA = FIA + ((FOH * DFINR1 * 8)/65536) * 2  (FIA 每次WG中断基本都增加BX,所以一定会出现有进位的现象)
					;	把DFINR1公式带入: FIA = FIA + ( FOH * TZ * 10 )/5086  应该40960的目的知道了，可以被约掉
				        ;	FIA = 当前的矢量旋转角度
					;       ZDK把((FOH * DFINR1 * 8)/65536) * 2 【*2 是因为是再次进入WG中断SPWM0中间间隔一次，所以是2次角度增加】
					;	【 / 65536 是因为ZDK把60度对应成65536 ，所以有整数倍进位就要移动SECTOR】

	  JNC	 THISSEC	    	;4	如果没有进位标志，则跳转THISSEC
					;	如果有进位标志意味FIA超过了65536(2^16)，就是超越了一个60读扇区

	  SHLB	 SECTOR,#1	    	;6+1	如果超越了一个扇区，则让SECTOR 左移1位， SECTOR的1位代表当前是哪个扇区
	  JBC	 SECTOR,6,THISSEC   	;5	SECTOR.6=0? 如果是0，意味当前还是在扇区1-6之间 则跳转THISSEC
	  LDB	 SECTOR,#1	    	;4 SUM=50 如果SECTOR.6=1意味是第7个扇区，就强行让扇区号SECTOR.0=1从第一个扇区开始

THISSEC:  LD	 FIX,FIA	    	;4	FIX = FIX + FIA
	  SHR	 FIX,#5 	    	;6+5	FIX = FIX / 32 (右移5位，就是/32 剩余11bit数据 MAX to 2048) 表示角度变化只能在2048个字节内(1024个点间)
	  AND	 FIX,#0FFFEH	    	;5	FIX = FIX 与 #0FFFEH (目的是取偶数，SINE表是字存储，查表要偶数地址)
	  SUB	 FIY,TAB,FIX	     	;5	FIY = TAB - FIX (TAB 是2048, 相当于2048个字节，60度)
	  POPF				;       FIY = 60Degree - FIX
	  RET

SPWM1:	  MULU	 T1,VRMS,SINTA[FIX]	;15	T1  = Vrms * SINTA[FIX]  T1=乘积低16bit T2=乘积高16bit= (Vrms * SINTA[FIX]) / 65536
					;	理论公式t2 =mT*sin(theta)
					;
	  MULU	 CX,VRMS,SINTA[FIY]    	;15	CX  = Vrms * SINTA[FIY] = Vrms * SINTA( 60 - FIX)  CX=乘积低16bit DX=乘积高16bit
					
	  LD	 T1,DX		       	;4	T1  = DX = (Vrms * SINTA[FIY]) / 65536   SINTABLE[] = 65536 * SIN ( 60N/1024)
					;       理论公式t1 =mT*sin(60-theta)
					;  
       ;  JBS	NEWVECT,7,DCBAKE1   	;5 SUM=64 制动?
	  JBC	 CONTROL,2,OVERM	;	CONTROL.2=0 (直流制动无效?) 则转OVERM (检查是否过调制?)

DCBAKE1:  LD	 T1,TDC			;	T1  = TDC
	  LD	 T2,TDC			;	T2  = TDC
	  LDB	 SECTOR,#1		;       SECTOR = 1
ENDSPWM1:

;*********过调制?******************
OVERM:	  ADD	 CX,T1,T2           	;9+5	CX =  T1 + T2
	  SUB	 DX,TZ,CX           	;5	DX =  TZ -CX  (DX = TZ - T1 - T2)  计算SVPWM中的无效矢量时间t0=Tpwm-t1-T2
	  JGT	 FULLPWM1           	;4	如果t0 (TZ - T1 -T2)>0, 则跳转FULLPWM1
	  CLR	 DX	            	;3	t0( TZ - T1 - T2 ) <=0  则清DX, 意味让t0 = 0
	  SUB	 T1,TZ,T2           	;5	T1 = TZ - T2

FULLPWM1:
	 ;SHR	 DX,#1		    	;6+1 	SUM=38
	  CLR	 DX			;	清DX无效矢量时间=0   实际仿真不是0，奇怪奇怪
SPLIT21:  JBS	 SECTOR,0,SEC11	    	;5	SECTOR.0=1? 则跳SEC11
	  JBS	 SECTOR,1,SEC21	    	;5	SECTOR.1=1? 则跳SEC21
	  JBS	 SECTOR,2,SEC31	    	;5	SECTOR.2=1? 则跳SEC31
	  JBS	 SECTOR,3,SEC41	    	;5	SECTOR.3=1? 则跳SEC41
	  JBS	 SECTOR,4,SEC51	    	;5	SECTOR.4=1? 则跳SEC51
					;       DX = 无效矢量时间=0

SEC61:	  LD	 AX,DX		    	;4	意味这段只能是第6个扇区了  AX =  DX
	  ADD	 BX,AX,T2	    	;5	BX = AX + T2
	  ADD	 CX,BX,T1	    	;5	CX = BX + T1
	  SJMP	 WOUT1		    	;7 SUM=46 跳转WOUT1(输出PWM)

SEC51:	  LD	 BX,DX			;	BX = DX
	  ADD	 AX,BX,T1		;       AX = BX + T1
	  ADD	 CX,AX,T2		;       CX = AX + T2
	  SJMP	 WOUT1			;       跳转WOUT1
SEC41:	  LD	 BX,DX			;	BX = DX
	  ADD	 CX,BX,T2		;	CX = BX + T2
	  ADD	 AX,CX,T1		;       AX = CX + T1
	  SJMP	 WOUT1			;	跳转WOUT1

SEC31:	  LD	 CX,DX			;	CX = DX
	  ADD	 BX,CX,T1		;	BX = CX + T1
	  ADD	 AX,BX,T2		;       AX = BX + T2
	  SJMP	 WOUT1			;	跳转WOUT1

SEC21:	  LD	 CX,DX			;	CX = DX
	  ADD	 AX,CX,T2		;	AX = CX + T2
	  ADD	 BX,AX,T1		;	BX = AX + T1
	  SJMP	 WOUT1			;	跳转WOUT1

SEC11:	  LD	 AX,DX			;	AX = DX
	  ADD	 CX,AX,T1		;	CX = AX + T1
	  ADD	 BX,CX,T2		;	BX = CX + T2

WOUT1:	  JBS	 MST,0,RUNREV1		;	MST.0=1? 开门 则跳转RUNREV1 ( 反向开门)

;**************正向关门?*****************

RUNFWD1:  ST	 AX,WG_COMP1[0]		;	赋新数给比较计数器，改变占空比就是改变电机运行频率
	  ST	 BX,WG_COMP2[0]		;	DUTY = ( WG_COMPx / WG_RELOAD ) * 100%
	  ST	 CX,WG_COMP3[0]
POPA6:	  POPF
	  RET
;*******电机正转和反转只是换了UV相顺序***

;**************反向开门?*****************

RUNREV1:  ST	 BX,WG_COMP1[0]       	;8+9	赋新数给比较计数器，改变占空比就是改变电机运行频率
	  ST	 AX,WG_COMP2[0]       	;8	DUTY = ( WG_COMPx / WG_RELOAD ) * 100%
	  ST	 CX,WG_COMP3[0]       	;8
POPA7:	  POPF			      	;7
	  RET			      	;11     SUM=51
				      	;TOTAL  346 -> 86.5 US (346 x 250ns=86.5us by Johnson) */

						
/************************************************
*        定时器 TIMER2溢出中断服务程序
*
*IN  PARAMETER:
*OUT PARAMETER:
*REMARK: 实际只是TIMER2的定时器，TIMER1为POLLING
*定时器进入时间间隔应该是16ms
*TIMER2(WORD)=65535 TIMER2的分辨率是250ns
*所以间隔=65536 x 250ns=16384000ns = 16.4ms
*所以程序中6024(次)= 6024 x 16.4=98.79秒=99秒
*          6000(次)= 6000 x 16.4=98.4秒
*************************************************/
/*TIMOV:	  PUSHF
	  EI
	  INCB  FRSH			;FRSH标志+1
	  ORB   RT,#00000100B		;每次进入定时中断都让RT.2=1不变
	  JBC   CONTROL,1,TOV1		;CONTROL.1=0，则跳转TOV1
	  INCB  DYCT			;DYCT标志+1
	  CMPB  DYCT,#200		;DYCT是否到200? 200* 16ms = 3.2秒
	  JNH   TOV2			;如果DYCT<=200 则调转TOV2
	  LDB   DYCT,#200		;如果DYCT>200，则DYCT=200
	  SJMP  TOV2
TOV1:	  CLRB  DYCT			;DYCT=00H
TOV2:	  JBC   RT,4,TOV3              	;RT.4=0则跳转TOV3 自学习推进开始? STUDY PUSH BEGINS
	  INC   SPT			;SPT标志+1
	  CMP   SPT,#6000              	;6024==99SEC
	  JNH   TOV31			;SPT<=6000则跳转TOV31
	  LD    SPT,#6000	       	;6024==99秒
	  SJMP  TOV31
TOV3:	  CLR   SPT			;SPT=00H
TOV31:	  JBS   RT,5,TOV4              	;RT.5=1 则跳转TOV4	ANY OF DC or OC BEGIN TO COUNT
	  JBS   RT,6,TOV4		;RT.6=1 则跳转TOV4
	  SJMP  TOV41

TOV4:	  INC   DOCPT			;DOCPT+1
	  CMP   DOCPT,#6000		;
	  JNH   TOV42			;DOCPT<=6000则跳转TOV42
	  LD    DOCPT,#6000		;如果DOCPT>6000 则让DOCPT=6000
	  SJMP  TOV42

TOV41:	  CLR   DOCPT			;DOCPT=00H
TOV42:	  JBC   CONTROL,2,TOV43		;CONTROL.2=0 则跳转TOV43
	  INC   DCCT			;DCCT+1
	  CMP   DCCT,#6000
	  JNH   TOV5			;DCCT<=6000 则跳转TOV5
	  LD    DCCT,#6000		;如果DCCT>6000 则让DCCT=6000
	  SJMP  TOV5

TOV43:	  CLR   DCCT			;DCCT=00H
TOV5:	  JBC   RT,1,TOV6		;RT.1=0 则跳装TOV6
	  INC   OBPT			;OBPT+1 力矩监控推进时间
	  CMP   OBPT,#6000		;力矩监控推进时间
	  JNH   TOV61			;OBPT<=6000 则跳转TOV61
	  LD    OBPT,#6000		;如果OBPT>6000 则让OBPT=6000
	  SJMP  TOV61

TOV6:	  CLR   OBPT			;OBPT=00H  力矩监控推进时间
TOV61:	  ADDB  SAMP_T,#32		;SAMP_T=SAMPT+32

	  CMPB  SAMP_T,#0		;SAMP_T是否等于0?  SAMP_T + 8次32能到零，意味进过8*16ms=128ms

	  JNE   PPPP			;如果SAMP_T不等于0，则跳转PPPP

	  JBC   FRSH,3,PPPP          	;FRSH.3=0 则跳转PPPP  SAMPLE TIME PER 4-TIMOV

	  SUB   DTS,AX3,NUMBER		;FRSH.3=1 (1000B *16ms=128ms) DTS = AX3 - NUMBER 核查是否遇到障碍物?
	  LD    AX3,NUMBER		;AX3=NUMBER
PPPP:	  JBC   RT,3,PPPP2		;RT.3=0 则跳转PPPP2
	  INC   JDLY			;JDLY+1
	  CMP   JDLY,#6000		;JDLY和6000比较
	  JNH   PPPP1			;JDLY<=6000 则跳转PPPP1
	  LD    JDLY,#6000		;JDLY>6000 则让JDLY=6000
PPPP1:	  SJMP  POPA9			;直接返回

PPPP2:	  CLR   JDLY			;JDLY=00H
POPA9:	  POPF
	  RET				;返回*/

/***************************************************
*          EXTINT外部中断保护服务程序
*IN  PARAMETER: NONE
*OUT PARAMETER:
***************************************************/
/*
EXINT:	  DI			        ;关闭所有中断
	  LDB   AL,P2_REG[0]
	  ORB   AL,#00010000B		;意味ZDK把PWM输出的245关闭,对我的板没有作用,但是IR2130S实际当前已经关闭PWM输入了.但是危险的是IR2130S若干时间后会自动打开PWM输入!!
	  STB   AL,P2_REG[0]

	  LDB   AL,P1_PIN[0]		;检查P1.0是否过压?  过压OU 低电平有效!
	  JBC   AL,0,EXINT1		;如果过压，转EXINT1
	  ANDB  AL0,#11111110B	        ;不是过压就是过流 SC-ACH9-P1.1
	  SJMP  TEST
EXINT1:   ANDB  AL0,#11111101B	        ;OU-ACH8-P1.0
TEST:	  DI				;关闭所有中断，有点重复

	  LDB   AL,P2_REG[0]		;意味ZDK把PWM输出的245关闭,对我的板没有作用,但是IR2130S实际当前已经关闭PWM输入了.
	  ORB   AL,#00010000B		;感觉有点重复
	  STB   AL,P2_REG[0]

	  CLRB  AL			;字节清AL寄存器
	  STB   AL,PI_PEND[0]
	  STB   AL,PI_MASK[0]		;屏蔽所有PI中断源

	  LDB   AL,WG_PROTECT[0]
	  ANDB  AL,#0FEH		;字节与#11111110B,意味让WG_PROTECT.0(EO)=0,禁止输出PWM
	  STB   AL,WG_PROTECT[0]

	  CLRB  INT_PEND
	  CLRB  INT_PEND1
	  CLRB  INT_MASK
	  CLRB  INT_MASK1
	  LDB   AL1,PI_MASK[0]
	  ANDB  AL1,#11101111B		;屏蔽外设的WG中断
	  STB   AL1,PI_MASK[0]

TEST1:	  LDB   AL1,#0
	  STB   AL1,PWM0[0]		;清楚PWM0 占空比控制寄存器(MC186.inc)，PWM0->P6.6=0 E2CK/ZDK SCH,EECK/JJ,估计是用这个脚来做试验验证故障发生

OCX:	  JBS   AL0,0,OUX		;测试位为1则转，判断是否当前故障是否过压AL0.0?
	  LD    AX,#4			;如果不是过压，必定是过流
	  SJMP  FLUR

OUX:	  JBS   AL0,1,ERRX		;判断AL0.1是否1?是否同时也过流? 如果过压OU，过流OC同时发生则转ERRX
	  LD    AX,#12
	  SJMP  FLUR
ERRX:	  LD    AX,#16			;如果过压过流同时发生，则显示组合故障指示
FLUR:	  LD    DX,AX
	  LDB   DBUF1,ERROR[DX]		;显示故障表 oc 或ou 或同时
	  INC   DX
	  LDB   DBUF2,ERROR[DX]		;
	  INC   DX
	  LDB   DBUF3,ERROR[DX]		;
	  INC   DX
	  LDB   DBUF4,ERROR[DX]
	  LDB   DBUF5,#0FFH             ;关闭所有的LED指示

	  LCALL DISPLY1			;显示"err" 直接(强制RT.2=0)数码管数据发送显示处理子程序
	  LD    DLY,#50                 ;延时50个基本单位时间,2.4msx50=100ms
	  LCALL DELAY2

NOMEM:	  LDB   AL,P2_REG[0]		;意味ZDK把PWM输出的245关闭,对我的板没有作用,但是IR2130S实际当前已经关闭PWM输入了.
	  ORB   AL,#00010000B		;感觉再次重复
	  STB   AL,P2_REG[0]
	  CLRB  AL
	  STB   AL,PI_MASK[0]		;屏蔽所有PI的外设中断源
	  STB   AL,PI_PEND[0]
	  CLRB  INT_PEND
	  CLRB  INT_PEND1
	  CLRB  INT_MASK
	  CLRB  INT_MASK1

FLOOP:	  LDB   AL,ERST[0]		;读F55 当前遇到故障的恢复模式设置 ERST=0键盘处理故障模式 ERST=1 键盘和端子处理故障模式
	  JBS   AL,0,TORST
	  LCALL KEYB			;读当前键盘状态子程序
	  JBC   KEY,3,NEXT1		;如果是停止键，则直接复位系统
	  SJMP  FLOOP			;要么读到键盘的停止键，要么修改当前故障恢复模式数值，否则一直在等待

TORST:	  LCALL KEYB			;读当前键盘状态子程序
	  JBC   KEY,3,NEXT1		;如果是停止键 则直接复位系统
	  JBC   TERMAL,3,NEXT1		;端子开门，则直接复位系统
	  JBC   TERMAL,2,NEXT1		;端子关门，则直接复位系统
	  SJMP  FLOOP                   ;继续循环等待
NEXT1:	  RST				;系统复位 PSW/PC清零,相当于程序重新启动
*/


/***********************************************
*          禁止PWM波形CPU输出子程序
*IN  PARAMETER:
*OUT PARAMETER:
************************************************/
void STWAVE()
{

}
/*STWAVE:   LDB   AL1,WG_PROTECT[0]
	  ANDB  AL1,#0FEH
	  STB   AL1,WG_PROTECT[0]
	  RET*/
	  
	  
/***********************************************
*          允许PWM波形CPU输出子程序
*IN  PARAMETER:
*OUT PARAMETER:
************************************************/
void OPWAVE()
{

}
/*OPWAVE:   LDB   AL1,WG_PROTECT[0]
	  ORB   AL1,#1H
	  STB   AL1,WG_PROTECT[0]
	  RET*/

/************************************************
*          启动SPWM波形发生子程序
*IN  PARAMETER:
*OUT PARAMETER:
*************************************************/
void OPSPWM()
{
	
}
/*OPSPWM:   LD    AX1,TZ			;AX = TZ (载波周期) (s)
	  SHR   AX1,#1			;右移一位，代表/2
	  ST    AX1,WG_COMP1[0]		;U Phase 比较技术器为载波周期的一半 占空比50%
	  ST    AX1,WG_COMP2[0]		;V Phase 比较技术器为载波周期的一半 占空比50%
	  ST    AX1,WG_COMP3[0]		;W Phase 比较技术器为载波周期的一半 占空比50%
					;根据Intel 97C196MC 规格占空比为50% Duty=WG_COMPx/WG_RELOAD
	  LDB	AL1,PI_PEND[0]	        ;CLRB  PI_PEND[0]

	  LDB	AL1,PI_MASK[0]
	  ORB	AL1,#00010000B
	  STB	AL1,PI_MASK[0]		;打开WG 中断
	  ORB	INT_MASK1,#01100000B	;打开EXTINT和PI中断
	  ST	TZ,WG_RELOAD[0]		;置本机当前的载波周期
	  LD	AX1,WG_CON[0]		;取当前的WG_CON状态
	  OR	AX1,#0000010000000000B	;首先确保WG_COUT.10=1 使能(start counter)
	  AND	AX1,#1100111111111111B 	;SPWM0 MODE 2011-10 Johnson 模式0 或 1 SPWM方式 中心对准，只更新1次或2次寄存器，增加计数，使能开始计数器
					;这个设置只是确保M1=M0=0 只能意味是SPWM0 或 SPWM1模式
	  ST	AX1,WG_CON[0]
	  RET*/
	  
	  
/************************************************
*         停止SPWM波形发生子程序
*IN  PARAMETER:
*OUT PARAMETER:
*************************************************/
void STSPWM()
{
	
}
/*
STSPWM:   LDB	AL1,PI_PEND[0]
	  LDB	AL1,PI_MASK[0]
	  ANDB	AL1,#11101111B		;屏蔽WG中断
	  STB	AL1,PI_MASK[0]
	  ANDB	INT_MASK1,#11011111B	;屏蔽PI中断，意味全部停止SPWM发生
	  CLR	AX1
	  ST	AX1,WG_RELOAD[0]	;载波周期清零,意味停止比较比较
	  ST	AX1,WG_COMP1[0]
	  ST	AX1,WG_COMP2[0]
	  ST	AX1,WG_COMP3[0]
	  RET*/
	  
/***********************************************
*           数码显示清屏处理程序
* DBUF1,2,3,4,5=0FFH
************************************************/
void BLACK()
{
	DBUF[0] = 0xFF;
	DBUF[1] = 0xFF;
	DBUF[2] = 0xFF;
	DBUF[3] = 0xFF;
	DBUF[4] = 0xFF;
}


/*****************************************
*        运行初始化INICIAL子程序
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER:
******************************************/
/*void INICIAL()
{	
	CONTROL |= 0x01;							//CONTROL.0=1
	AX1 = 4000;									//AX1 = 4000
	AX1 = AX1 * 1000 / EFC[0];					//AX1 = AX1 * 1000  ( 4000 * 1000 ) 字乘法 AX1=乘法的低16bit BX1=乘法的高16bit
												//AX1 = AX1 / EFC载波频率(F19 Hz) 商放在AX1  余数放在BX1   AX1=011DH(285)
	TZ = AX1;									//感觉TZ 应该是载波的周期，但是为什么是4000 000 /14000(14Khz)?
												//难道是载波周期 放大1000 * 4000 倍?
												//TZ载波周期S(秒),实际是WG_RELOAD计数器的数值，根据Intel 87C196MC规格书公式的确算出来是载波周期

	BOOST();									//调用低频力矩补偿子程序 -->VTRQ
												//VTRQ = [(转矩补偿百份数 放大1000倍的)/ 1000 ] * [4000 *1000 / 载波频率14000]

	FOM = EFOM[0];								//字装载,把EEPROM F02对应的本机设置的最高运行频率数值存放到FOM中，放大100倍 5000 = 50HZ

       // ---------波形运算用参量--------

	BX1 = ETBK[0];								//BX1 = F17_ETBK  ( F17 直流制动时间  出厂2秒 固定为2秒)
	CX1 = (BX1 * 10000) / 163;					//CX1 = BX1 * 10000  CX1=乘法的低16bit DX1=乘法的高16bit
												//2 * 10000 =20000毫秒?
												//CX1 = CX1 / 163 why?  CX1 = 除法的商(122) DX1 = 除法的余数(114)
	RDCCT[0] = CX1;								//RDCCT = CX1  (直流制动时间参考)
												//RDCCT = ( F17_ETBK(2) * 10000 ) / 163 的商

	BX1 = ESPT[0];								//BX1 = ESPT F44 自学习推进时间 出厂2秒 [0 - 10]
	CX1 = (BX1 * 10000) / 163;					//CX1 = BX1 * 10000  放大10000倍  CX1=乘法的低16bit DX1=乘法的高16bit
												//CX1 = CX1 / 163  CX1 = 除法的商(122) DX1 = 除法的余数(114)
	RSPT[0] = CX1;								//RSPT = CX1 (自学习时间参考)
												//RSPT = ( F44_ESPT(2) * 10000 ) / 163 的商

	BX1 = EJDLY[0];								//BX1 = EJDLY F57 强迫关门延时发信号时间 出厂2秒 [0 - 10]
	CX1 = (BX1 * 10000) / 163;					//放大10000倍  CX1=乘法的低16bit DX1=乘法的高16bit
												//CX1 = CX1 / 163  CX1 = 除法的商(122) DX1 = 除法的余数(114)
	RJDLY[0] = CX1;								//RJDLY = CX1 (强迫关门继电器延时发信号时间参考)
												//RJDLY = ( F57_EJDLY(2) * 10000 ) /163 的商


	BX1 = EDOPT[0];								//BX1 = EDOPT F42 关门推进时间(关到位夹紧力保持时间，门关齐后的持续推进时间)出厂100秒 (代表永远推进) [0 - 100] ZDK出厂90秒
	CX1 = (BX1 * 10000) / 163;					//放大10000倍  CX1=乘法的低16bit DX1=乘法的高16bit
												//CX1 = CX1 / 163  CX1 = 除法的商(104) DX1 = 除法的余数(8)
	RDOPT[0] = CX1;								//RDOPT = CX1 (关门推进时间参考) 【ZDK 好像有意把变量名字搞颠倒】
												//RDOPT = (F42_EDOPT(100) * 10000 ) /163 的商

	BX1 = EDCPT[0];								//BX1 = EDCPT F43 开门推进时间(开到位阻止力保持时间，
												//门开齐后的持续推进时间)出厂100秒 (代表永远推进) [0 - 100] ZDK出厂100秒
	CX1 = BX1 * 10000 / 163;					//放大10000倍  CX1=乘法的低16bit DX1=乘法的高16bit
												//CX1 = CX1 / 163  CX1 = 除法的商(104) DX1 = 除法的余数(8)
	RDCPT[0] = CX1;								//RDCPT = CX1 (开门推进时间参考) 【ZDK 好像有意把变量名字搞颠倒】
												//RDCPT = ( F43_EDCPT(100) * 10000 ) /163 的商


	AX1 = EVBK[0];								//AX1 = F18_EVBK (F18 制动强度  出厂2Hz 放大10倍 20 (固定数值))
	AX1 = AX1 * TZ / 2000;						//AX1 = AX1 * TZ  有点问题，TZ好像是周期呀 AX1=乘法的低16bit BX1=乘法的高16bit
												//AX1 = AX1 / 2000  why?  AX1 = 除法的商 BX1 = 除法的余数
	TDC = AX1;									//TDC = AX1 (直流制动SVPWM的T1，T2时间数值 1%的TZ)
												//TDC = ( F18_EVBK(20) * TZ ) /2000 的商


	AX1 = TZ * 40960 / 5086;					//AX1 = TZ * 40960 why?  AX1=乘法的低16bit BX1=乘法的高16bit
												//AX1 = AX1 / 5086 why?  AX1 = 除法的商 BX1 = 除法的余数
	DFINR1 = AX1;								//DFINR1 = AX1 DFINR1是什么?
												//DFINR1 = ( 40960 * TZ ) / 5086 的商

	VANDF(); 									//调用V/F特性处理子程序 -->KV1

	/*********计算开门的DOS01数值***
	* 在第0个换速位置和第1个换速位置之间的点 (%) 确保S曲线正确润滑
	********************************
	AX1 = DOS1[0];								//AX1 = 开门换速位置#1 DOS1(%)
	AX1 = AX1 - DOS0[0];						//AX1 = AX1 - 开门换速位置#0 DOS0 (%)
	DOSA[0] = AX1;								//DOSA = AX1 (有点意思，DOSA=DOS1-DOS0)
	AX1 = AX1 >> 1;								//AX1 = AX1 /2 (DOS1-DOS0)/2 好像是计算第一和第二个换速位置的中点
	AX1 = AX1 + DOS0[0];						//AX1 = AX1 + DOS0 (是开门开始到第0和第1换速位置中心的长度换成%)
	DOS01[0] = AX1;								//搞定DOS01 就是第一和第二换速点的中心%
												//DOS01 (%) = DOS0 + ( DOS1 - DOS0) /2

	/*********计算开门的DOS23数值***
	* 在第2个换速位置和第3个换速位置之间的点 (%) 确保S曲线正确润滑
	********************************
	AX1 = DOS3[0];								//AX1 = 开门换速位置#3 DOS3(%)
	AX1 = AX1 - DOS2[0];						//AX1 = AX1 - 开门换速位置#2 DOS2
	DOSB[0] = AX1;								//DOSB = AX1 (DOSB=DOS3-DOS2)
	AX1 = AX1 >> 1;								//AX1 = AX1/2  (DOS3-DOS2)/2 算第2和第3个换速位置的中心
	AX1 = AX1 + DOS2[0];						//AX1 = AX1 + DOS2
	DOS23[0] = AX1;								//搞定DOS23 就是第二和第三换速点的中心%
												//DOS23 (%) = DOS2 + ( DOS3 - DOS2 )/2

	/*********计算关门的DCS01数值***
	* 在第0个换速位置和第1个换速位置之间的点 (%) 确保S曲线正确润滑
	********************************
	AX1 = DCS0[0];
	AX1 = AX1 - DCS1[0];
	DCSA[0] = AX1;
	AX1 = AX1 >> 1;
	AX1 = AX1 + DCS1[0];
	DCS01[0] = AX1;								//搞定DCS01 就是关门第0和第1换速点的中心%
												//DCS01 (%) = DCS1 + ( DCS0 - DCS1 )/2
												//注意关门点计算公式和开门不同

	/*********计算关门的DCS23数值***
	* 在第2个换速位置和第3个换速位置之间的点 (%) 确保S曲线正确润滑
	********************************
	AX1 = DCS2[0];
	AX1 = AX1 - DCS3[0];
	DCSB[0] = AX1;
	AX1 = AX1 >> 1;
	AX1 = AX1 + DCS3[0];
	DCS23[0] = AX1;								//搞定DCS23 就是关门第二和第三换速点的中心%
												//DCS23 (%) = DCS3 + ( DCS2 - DCS3 )/2
												//注意关门点计算公式和开门不同

	/***计算开门DOVA,DOVB数值***
	* 开门速度平方特性
	********************************
	AX1 = DOV1[0];								//AX1 = DOV1 (开门换速位置#1对应的速度Hz)
	AX1 = AX1 - DOV0[0];						//AX1 = DOV1 -DOV0(计算开门换速位置#0，#1对应位置的速度差 Hz)
	AX1 = AX1 << 1;								//AX1 = AX1 *2 (DOV1-DOV0)*2
	DOVA[0] = AX1;								//开门平方特性参数DOVA =  (DOV1-DOV0)*2
												//意味 DOV1 > DOV0 (Hz)

	AX1 = DOV1[0];
	AX1 = AX1 - DOV2[0];
	AX1 = AX1 << 1;
	DOVB[0] = AX1;								//开门平方特性参数DOVB =  (DOV1-DOV2)*2
												//意味 DOV1 > DOV2 (Hz)

	/***计算关门DCVA,DCVB数值***
	* 关门速度平方特性
	********************************
	AX1 = DCV1[0];
	AX1 = AX1 - DCV0[0];
	AX1 = AX1 << 1;
	DCVA[0] = AX1;								//关门平方特性参数DCVA =  (DCV1-DCV0)*2
												//意味 DOV1 > DOV0 (Hz)

	AX1 = DCV1[0];
	AX1 = AX1 - DCV2[0];
	AX1 = AX1 << 1;
	DCVB[0]	= AX1;								//关门平方特性参数DCVB =  (DCV1-DCV2)*2
												//意味 DOV1 > DOV2 (Hz)

	FIA = 0;									//FIA=#0000H
	NEWVECT = 0;								//NEWVECT=00H
	SECTOR = 1;									//扇区号SECTOR=1
	FGC = #LSP0;								//FGC = LSP0 (运算的最小频率低16bit) Hz
	FOH = #LSP0;								//FOH = LSP0 (运算的最小频率高16bit) Hz
	FG = #LSP0;									//FG  = LSP0 (运算的最小频率高16bit) Hz
	FOL = 0;									//FOL =#0000H
	TAB = 2048;									//TAB = 2048 (SINE表1024个点字数据，按字节就是2048)

	DTS	= 0;									//DTS = #0000H
	AX3 = NUMBER;								//AX3 = NUMBER (30000) AX3 缓存本次获得的编码器脉冲数

	CONTROL = 0;								//CONTROL=#00H
	FUN = 0;									//FUN=00

	AX1 = EDEADT[0];							//2011-10 Johnson EDEADT (F23)死区时间出厂为2 实际对应80C196MC为1us 没有算外部IR2130S自带的
	AX1 = AX1 << 2;								//死区时间减小一倍,根据80C196的规格书Tdead=DT/8 ZDK把F23的数值放大4倍就是 F23/2=当前的死区时间(us)
	AX1 &= 0x03FF;								//取低10bit影响死区时间 AX1 &= #0000001111111111B;
	BX1 = WG_CON[0];
	BX1 &= 0xFC00;								//保留当前WG_CON的高五位状态，重点是叠加死区新数据进去 BX1 &= #1111110000000000B;	
	BX1 |= AX1;
	WG_CON[0] = BX1;
}*/

/*********************************************************
*             辅助初始化子程序
*IN  PARAMETER: NONE
*OUT PARAMETER: NONE
**********************************************************/
void BEGIN() 
{
	TERMAL = 0xFF;									//2011-10 Johsnon感觉重复，TERMAL通信参数标志设置为#FFH，表示当前没有按键输入，也没有外部控制(AUTO归结到外部控制）
	DBUF[4] = 0xFF;
	//P2_REG[0] = 0x1C;								//2011-10 Johnson 同时把其他的设置为0，只有SCT/SIN/THR置高(初始化LED移位寄存器并关闭PWM输出) (本程序暂时不支持THR)
													
													//F00设置(给定)频率->AX1  default EFG =5000 50HZ
													//F02最大运行频率->BX1  default EFOM=5000 50HZ  
	if(E2M[EFG] > E2M[EFOM]) {						//当前设定频率F00不高于最大频率则F02转BEGIN1
		E2M[EFG] = E2M[EFOM];						//如果F00 设定频率 <= F02 最大运行频率 则让 设定频率F00=最大运行频率
	}
	
	FUN = 0;										//功能码置为00H
	RT &= 0x00;										//程序一开始RT标志为00H, 1有效

	CONTROL = 0;									//CONTROl=00H
	FOH = 0;										//FOH=0000H
	FOL = 0;										//FOL=0000H
	DCCT = 0;										//DCCT=0000H 可能是直流制动时间

	DSCD = E2M[EDSCD];								//把当前显示模式F45 EDSCD存入DSCD 0=显示设定频率，1=显示输出频率，2=显示位置百分数，3=显示位置脉冲数
	AL0 = 0xFF;										//AL0标志置为FFH
	SMT = 0;										//清SMT标志
}

/************************************************************
*        频率跳变点预处理--自学习常规加减速处理子程序
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: INCFOL, INCFOH, DECFOL, DECFOH
*      感觉应该是速度，可是为什么是时间?
*************************************************************/
/*void ADCL0()
{
	CX1 = EACL0[0];									//CX1 = EACL0 (F05 自学习常规加速时间s) 出厂20s 放大10倍 200
	BX1 = 0;
	AX1 = 4096;									//为什么要用4096?
	AX1 = AX1 / CX1;								//双字除法  AX1 = AX1 / CX1 ( 4096 / 200 = 20.余数96 ) 商在AX1(20) 余数在BX1(96)

	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  =20 * TZ(285)= 5700(1644H) BX1(0000H)  AX1=乘法的低16bit BX1=乘法的高16bit
					
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 左移2位放大4倍 5700*4=22800(5910H) 18/1-2000
													//AX1 = [((4096 / F05_EACL0) * TZ )* 4] 低16bit
													//BX1 = [((4096 / F05_EACL0) * TZ )* 4] / 65536
	INCFOL = AX1;									//INCFOL = AX1  (5910H)(22800)
	INCFOH = BX1;									//INCFOH = BX1  (0000H)
   /***2011-10明白为什么是H和L?**********
   *原因L代表32位乘法的低16位结果
   *    H代表32位乘法的高16为结果
   *F05_EACL0 =[1 - 999]仿真发现无论如何INCFOH=0000H
   *INC(DEC) FOL(FOH)到底是什么意思 ?
   **************************************
	CX1 = EDCL0[0];									//CX1 = EDCL0 (F06 自学习常规减速时间s) 出场10s 放大10倍 100
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 (4096 / 100=4. 余数96) 商在AX1(4) 余数在BX1(96)
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ = 4 * 285 = 11400(2C88H),BX1放乘积高16位(当下是0) AX1=乘法的低16bit BX1=乘法的高16bit
	AX1 = AX1 << 2;								//AX1 = AX1 *4  左移2位放大4倍  18/1-2000
	DECFOL = AX1;									//DECFOL = AX1 = [((4096 / F06_EDCL0) * TZ ) * 4] 低16bit  (0B220H) 45600
	DECFOH = BX1;									//DECFOH = BX1 = [((4096 / F06_EACL0) * TZ )* 4] / 65536   (0000H)
}*/

/************************************************************
*	频率跳变点预处理--反转加减速处理子程序
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: INCFOL, INCFOH, DECFOL, DECFOH
*    感觉应该是速度，可是为什么是时间?
*************************************************************/
/*void ADCL1()
{
	CX1 = EACL1[0];									//CX1 = EACL1 (F07 反向加速时间s) 出厂30s 放大10倍 300
	BX1 = 0;
	AX1 = 4096;									//为什么要用4096?
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 300 = 13 余196) AX1 = 除法的商 BX1 = 除法的余数
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=乘法的低16bit BX1=乘法的高16bit
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 左移2位放大4倍  18/1-2000
	INCFOL = AX1;									//INCFOL = AX1 = [((4096 / F07_EACL1) * TZ ) * 4] 低16bit  (39E4H) 14820
	INCFOH = BX1;									//INCFOH = BX1 = [((4096 / F07_EACL1) * TZ )* 4] / 65536   (0000H)

	CX1 = EDCL1[0];									//CX1 = EDCL1 (F08 反向减速时间s) 出厂10s 放大10倍 100
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 100 = 4 余96) AX1 = 除法的商 BX1 = 除法的余数
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=乘法的低16bit BX1=乘法的高16bit
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 左移2位放大4倍 18/1-2000
	DECFOL = AX1;									//DECFOL = AX1 = [((4096 / F08_EDCL1) * TZ ) * 4] 低16bit   (0B220H) (45600)
	DECFOH = BX1;									//DECFOH = BX1 = [((4096 / F08_EDCL1) * TZ )* 4] / 65536    (0000H)
}*/

/************************************************************
*	频率跳变点预处理--常规加减速处理子程序
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: INCFOL, INCFOH, DECFOL, DECFOH
*   感觉应该是速度，可是为什么是时间?
*************************************************************/
/*void ADCL2()
{
	CX1 = EACL2[0];									//CX1 = EACL2 (F09 常规加速时间s ) 出厂10s 放大10倍 100
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 100 = 4 余96 ) AX1 = 除法的商 BX1 = 除法的余数
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=乘法的低16bit BX1=乘法的高16bit
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 左移2位放大4倍 18/1-2000
	INCFOL = AX1;									//INCFOL = AX1 = [((4096 / F09_EACL2) * TZ ) * 4] 低16bit (0B220H) 45600
	INCFOH = BX1;									//INCFOH = BX1 = [((4096 / F09_EACL1) * TZ )* 4] / 65536   (0000H)

	CX1 = EDCL2[0];									//CX1 = EACL2 (F10 常规加速时间s ) 出厂15s 放大10倍 150
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 150 = 27余46 )  AX1 = 除法的商 BX1 = 除法的余数
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=乘法的低16bit BX1=乘法的高16bit
	AX1 = AX1 << 2;                 				//AX1 = AX1 * 4 左移2位放大4倍 18/1-2000
	DECFOL = AX1;									//DECFOL = AX1 = [((4096 / F10_EDCL2) * TZ ) * 4] 低16bit   (783CH) 30780
	DECFOH = BX1;									//DECFOH = BX1 = [((4096 / F10_EDCL2) * TZ )* 4] / 65536     (0000H)
}*/

/******************************************
*         键盘数值分析转译子程序
*IN  PARAMETER: AL1
*MID PARAMETER:
*OUT PARAMETER: AL1
*2011-10 Johnson 本软件只支持8个键
*******************************************/
void KEY138()
{
	AL1 |= 0xF0;         						//只取低四位 ENABLE B0-B3
	
	if(AL1 == 0xFF) {							//有没有键按下?
		AL1 = 0xFF;								//把实际键值转译成#1111 1111B 无键按下
	} else if(AL1 == 0xFE) {					//开门键?
		AL1 = 0xFE;								//把实际键值转译成#1111 1110B 开门键
	} else if(AL1 == 0xFD) {					//关门键?
		AL1 = 0xFD;								//把实际键值转译成#1111 1101B 关门键
	} else if(AL1 == 0xFC) {					//强迫关键? 本机器不支持这个键
		AL1 = 0xFB;								//把实际键值转译成#1111 1011B 强迫关键? 本机器不支持这个键
	} else if(AL1 == 0xFB) {					//停止键?
		AL1 = 0xF7;								//把实际键值转译成#1111 0111B 停止键
	} else if(AL1 == 0xFA) {					//DOWN键?
		AL1 = 0xEF;								//把实际键值转译成#1110 1111B DOWM键
	} else if(AL1 == 0xF9) {					//UP键?
		AL1 = 0xDF;								//把实际键值转译成#1101 1111B UP键
	} else if(AL1 == 0xF8) {					//功能键?
		AL1 = 0xBF;								//把实际键值转译成#1011 1111B 功能键
	} else if(AL1 == 0xF7) {					//写入键?
		AL1 = 0x7F;								//把实际键值转译成#0111 1111B 写入键
	} else if(AL1 == 0xF6) {					//软件复位键?
		AL1 = 0XF3;								//把实际键值转译成#1111 0011B 软件复位键
	} else if(AL1 == 0xF5) {					//停止+强迫关门+开门+关门键?本机器不支持这个键
		AL1 = 0xF0;								//把实际键值转译成#1111 0000B 停止+强迫关门+开门+关门键?本机器不支持这个键
	}
}



void delay_loop(unsigned int ude)
{
    unsigned int  i, j;
    
    for (i = 0; i < 10; i++) {
    	for(j = 0; j < ude; j++) {}
    }
}


/************************************************
 * 			set_bit_data(unsigned int udat)
 * 	set output GPIO16 bit by bit 
 ***********************************************/
void set_bit_data(unsigned int udat)
{
	delay_loop(50);
	if(udat)
		GpioDataRegs.GPASET.bit.GPIO16 = 1;			// set DO = 1
	else
		GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;		// set DO = 0
	
	delay_loop(50);
	GpioDataRegs.GPASET.bit.GPIO18 = 1;				// set SCLK = 1
	delay_loop(100);
	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;			// set SCLK = 0	
}

/************************************************
 * 			read_bit_data()
 *	read the output one bit by bit on GPIO18
 ***********************************************/
unsigned int read_bit_data()
{
	unsigned int rx_tmp = 0x0;
	
	delay_loop(100);
	GpioDataRegs.GPASET.bit.GPIO18 = 1;				// set SCLK = 1
	delay_loop(50);
	rx_tmp = GpioDataRegs.GPADAT.bit.GPIO17;
	delay_loop(50);

	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;			// set SCLK = 0
	
	return !(!(rx_tmp));
}


/*------[  BEGIN FROM '1' 起始状态设置]--------*/
void EBOOST()
{
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;		//set CS = 0
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;		//set DO = 0
	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;		//set SCLK = 0
	delay_loop(100);
	
	GpioDataRegs.GPASET.bit.GPIO19 = 1;			// set CS = 1
	delay_loop(1);
	GpioDataRegs.GPASET.bit.GPIO16 = 1;			// set DO = 1
	delay_loop(1);
	GpioDataRegs.GPASET.bit.GPIO18 = 1;			// set SCLK = 1
	delay_loop(100);
	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;			// set SCLK = 0
}

/**************************************************************************
 * 	send_data(unsigned int udat, unsigned char ubit, unsigned char umode)
 * udat: write into the E2PROM 
 * ubit: write ubit from udat 
 * umode: 16bit or 8bit
 *************************************************************************/ 	
void send_data(unsigned int udat, unsigned char ubit, unsigned char umode)
{
	unsigned int i;
	
	if(ubit > 16 || ubit > umode)
		return ;
	
	for(i = 0; i < ubit; i ++) {
		set_bit_data( !(!(udat & (1 << (umode - i - 1)))) );		// umode = 8 or 16
	}	
}


/************************************************
 *  		gpio_set_e2prom_EWEN()
 *  set 93c66 EWEN 
 * *********************************************/
void gpio_set_e2prom_EWEN()
{
	EBOOST();	
	
	send_data(uEWEN, 10, 16);					//uEWEN  0x3000    = 0011 0000 0000 0000 
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;		//set CS = 0
	delay_loop(100);
	GpioDataRegs.GPASET.bit.GPIO19 = 1;			//set CS = 1
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;		//set DO = 0
	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;		//set SCLK = 0
}

/************************************************
 * 		void gpio_set_e2prom_EWDS()
 * set 93c66 EWDS 
 ***********************************************/
void gpio_set_e2prom_EWDS()
{	
	EBOOST();
	
	send_data(uEWDS, 10, 16);					
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;		//set CS = 0
	delay_loop(100);
	GpioDataRegs.GPASET.bit.GPIO19 = 1;			//set CS = 1
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;
}


/*--------[ Write AX1 to AL9 (0-256)x16]-------------*/
/*********************************************************
*           串行EEPROM写入子程序
*IN  PARAMETER: AX9(地址),AL1(数据)
*OUT PARAMETER: NONE
**********************************************************/
void EWRE(unsigned int udata, unsigned int uaddr)
{
	gpio_set_e2prom_EWEN();			//set the EWEN before read the data
	
	EBOOST();
	
	send_data(uWRITE, 2, 8);		//send the read ills  uWRITE = 0x40 =  0100 0000B
 	send_data(uaddr, 8, 8);			//set the address 
 	send_data(udata, 16, 16);		//send 16 bit data write to EEPROM
	
	gpio_set_e2prom_EWDS();	
}


/*------[ Read 'AL9' to AX1 (16bit)]------
* *****  READ(0110XXXXXXXX)************
*入口参数:AL9(8bit地址)
*出口参数:AX1(16bit)
*----------------------------------------*/
Uint16 ERDE(unsigned int uaddr)
{
	unsigned int i, rx_data = 0 ;
	
	gpio_set_e2prom_EWEN();			   		// set the EWEN before read the data	
	delay_loop(1);
	
	EBOOST();
	send_data(uREAD, 2, 8);  				// uREAD  0x80    0x1000 0000
	send_data(uaddr, 8, 8);				 	// 8 bit address  
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
	
	i = 16;
	while(i > 0) {
		rx_data |= read_bit_data() << (i - 1);
		i --;		
	}
	
	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;		// set CS = 0
	delay_loop(100);
	GpioDataRegs.GPASET.bit.GPIO19 = 1;			// set CS = 1
	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;
	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;
	gpio_set_e2prom_EWDS();	
	return rx_data;
}


		 
/*===============================================
*   首次运行需要把功能码参数写入空EEPROM子程序
*   函数功能：判断是否要恢复出厂设置，其次判断是否有授权密码
*   无授权密码则等待输入写入键后进行写入出厂操作
*	存在疑问：如果有授权密码，则无处理程序，无法对E2PROM进行操作
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER:
*===============================================*/
void INE2M()
{
	AX9 = XSAV;                       		//字装载, XSAV=028H,让AX9=028H (开始读取F20的数值参数)
	AX1 = ERDE(AX9);						//调用EEPROM读程序,读取的数值存放在AX1中
	
	if(AX1 != 888) {                        //888为恢复出厂设置标志  6080 as DEFAULT PARAMETER RESTORE LABEL
		AX9 = XPWD;							//XPWD=C8H(200),AT93C66的密码   看EEPROM C8H里面的数值是不是3FH?
		AX1 = ERDE(AX9);
		if(AX1 != PASS) {					//读到的数值放在AX1中,其中密码的低字节-->(AL1)
		//PASS=3FH(63),把AL1的内容与3FH按照字节比较看是不是相等(检查EEPROM是否有授权码)，不相等则跳转
		//如果没有发现授权密码,则转EEPROM出厂参数写入处理程序
			KEYB();				//
			while(KEY != 0x7F) {			//读当前键盘状态子程序
				KEYB();						//如果不是就继续读键值,意味如果EEPROM没有授权密码,用户不输入写入键,则一直循环等待!	
			}
		} else {
			return ;
		}	
	}
	
	CX1 = 0;						//如果是写入键,则开始处理写入EEPROM出厂参数工作
	DX1 = 0;
	BX1 = 0;
	DBUF[0] = 0x61;
	DBUF[1] = 0x99;					//E
	DBUF[2] = 0x03;					//4  RETORE FACTORY CODE OK LABEL->E200 #25H
	DBUF[3] = 0x03;					//0 
	DBUF[4] = 0xFF;					//0
	

	DISPLY1();						//显示E400(恢复出厂参数标志) 直接(强制RT.2=0)数码管数据发送显示处理子程?
	BX1 = 100;						//显示持续约定时间 100xDelay TIME
	
	while(BX1 -- != 0) {			//显示E400(恢复出厂参数标志)
		DELAY();
		DISPLY1();					//直接(强制RT.2=0)数码管数据发送显示处理子程序
	}
	
	AX9 = 0;                      	//清AX9,准备恢复出厂设置参数信息
	
	while(AX9 != 159) {				//159?(160个) 双字节就是80个参数 (实际看X8401的有效数目的确是160) 把部分SINE表也写进去了
		AX1 = FACTRY[AX9];              //开始恢复出厂信息, 把出厂参数装入AX1中(偏移地址是AX9)
		E2M[AX9] = AX1;                 //首先把RAM对应的设置数值也同时恢复成出厂参数数值
	
		EWRE(AX1, AX9);					//调用EEPROM写入程?
	
		//DLY = 2;                     	//Add for AT93C66
		DELAY();                       	//能有效复位成出厂参数
		AX9 ++;				 			//AX9=AX9+1 =下一个单元的地址
										//2011-10 Johnson 感觉再次加1就多余，这也是实际EERPOM数据出现地址间隔的原因，ZDK HD508.ASM没有这句，可以考虑去掉这句
	}

	AX1 = PASS;                         //最后把3FH(63)授权码写到EERPOM的(C8H)地址
	AX9 = XPWD;                        	//调用写入程序
	EWRE(AX1, AX9);				 		//调用EEPROM写入程序
	//DLY = 2;                         	//Add for AT93C66
	DELAY();                           //实际编程器读取AT93C66的真实数据发现地址列表和本程序有差别，不影响使用
	
	return ;
}



/********************************************************
*         从EEPROM读取功能码数值保存到对应RAM中
*IN  PARAMETER: FUN
*MID PARAMETER: AX8
*OUT PARAMETER: NONE
*********************************************************/
void INRAM()
{
	AX1 = FUN;
	AX8	= E2M[AX1];
	AX9 = 0;
	for(AX9 = 0; AX9 < 119; AX9 ++) {
		AX1 = ERDE(AX9);
		E2M[AX9] = AX1;
	}
}



/******************************************************
*  把指定功能码参数从EEPROM 取数装入对应RAM单元子程序
*IN  PARAMETER: FUN,AX9(当前功能码)
*MID PARAMETER：AX9,AH1,AL1
*OUT PARAMETER: AX8(对应功能码的EEPROM设置参数)
*******************************************************/
void GETS()
{

}
/*GETS:	 CMP    FUN,#0
	 JNE    GETS1				;如果功能码是F00则返回
	 RET
GETS1:	 ADD    AX9,FUN,FUN			;AX9 = 当前功能对应的EEPROM物理地址
	 INC    AX9
	 LCALL  ERDE
	 LDB    AH1,AL1
	 DEC    AX9
	 LCALL  ERDE
	 ST     AX1,E2M[AX9]			;读取EEPROM里面当前功能码的设置参数，保存到内存相关单元里面
	 ST     AX1,AX8
	 RET*/

/******************************************************************
*           获得位置脉冲数子程序 (位置测量)
*IN  PARAMETER:	TIMER1	应该是获得TIMER1的数值
*OUT PARAMETER: TIMER1
*******************************************************************/
/*void GETPOS()
{
	NUMBER = (unsigned int)EQep1Regs.QPOSCNT / 4;		//获取当前编码器脉冲数
	if(NUMBER <= 0) {
		NUMBER = 0;
		//将编码器脉冲数置0
	}
	
	if(SD_FLG != 2) {			//自学习没有结束
		if(NUMBER > NMAX) {
			NUMBER = NMAX; 		//把前编码器脉冲置为最大脉冲数
			//把编码器脉冲计数器置为最大脉冲数
		}
	} else {					//自学习结束
		if(NUMBER >= EPULSE[0]) {
			NUMBER = EPULSE[0];
			///将编码器置为EPULSE[0]
		}
	}
}*/


/*************************************************************
*    (只处理上电最大脉冲显示模式)命令处理子程序 COMMAND ROUTINE
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: COMD
*		TERMAL =# 1111 IN4 IN3 AUTO(IN2) GND(IN1)
*         高有效!       IN4:	       关门
*			IN3:           开门
*			IN2(换为AUTO): 轻推 GBV10硬件换成AUTO了
*			IN1(换为GND):  停止 GBV10硬件换成GND了，永远失效
*         TERMAL的低四位是经过程序取反的，高无效！低有效
*	COMD标志不区分当前是键盘还是点动还是外部端子，统一到COMD的低四位
**************************************************************/
/*void COMMAND()
{
	AL1 = APPL[0];									//把F58给AL1  (可调频率模式 1->上电显示最大脉冲数目,0->上电显示最大频率值)
	if(AL1 & 0x01 == 1) {							//如果是 上电显示最大脉冲数目 模式 则跳转COMMD1  AL1.bit0 == 1
		AL1 = ECMD[0];								//AL1 = F11 (控制模式) 0->键盘控制 1-> 点动  2->外部端子
		if(AL1 == 2) {								//控制模式是2 外部端子控制模式 
			if(TERMAL & 0x02 == 0) {                //外部端子控制模式下TERMAL.1=1? (HAD 是轻推，实际换成AUTO了)  不是IN1(轻推 不支持) 则跳ERLY2    NRDING
				COMD = 0xFD;						//COMD = # 1111 1101B 端子强迫关门
			} else if(TERMAL & 0x04 == 0) {         //不是开门IN3 则跳ERLY33   OPEN
				COMD = 0xFB;						//COMD = # 1111 1011B 端子开门
			} else if(TERMAL & 0x01 == 0) {       	//不是端子停止(GDV10不支持)则跳ERLY3  STOP
				COMD = 0xFE;						//COMD = # 1111 1110B 端子停止
			} else if(TERMAL & 0x08 == 0) {         //不是端子关门则跳ERLY4 CLOSE
				COMD = 0xF7;						//COMD = # 1111 0111B 端子关门
			} else {
				COMD = 0xFF;						//没有端子信号，则让COMD = # 1111 1111B
			}
		} else if(AL1 == 1){						//控制模式是1 键盘点动，本软件不支持
			if(KEY & 0x01 == 0) {					//不是键盘开门键则跳KJOG1
				COMD = 0xFB;						//COMD = # 1111 1011B 键盘开门
			} else if(KEY & 0x02 == 0) {			//不是键盘关门键则跳KJOG2
				COMD = 0xF7;						//COMD = # 1111 0111B 键盘关门
			} else if(KEY & 0x04 == 0) {			//不是键盘强迫关门则跳KJOG3
				COMD = 0xFD;						//COMD = # 1111 1101B 键盘强迫关门 (GDV10没有这个键)
			} else if(KEY & 0x08 == 0) {			//不是键盘停止键则跳KJOG4
				COMD = 0xFE;						//COMD = # 1111 1110B 键盘停止
			} else {
				COMD = 0xFF;						//没有相关按键则让COMD = # 1111 1111B
			}
		} else if(AL1 == 0) {						//控制模式是0 键盘模式
			if(KEY & 0x04 == 0) {                   //不是键盘强迫关门键则跳KBTM2	NRDING
				COMD = 0xFD;						//COMD = # 1111 1101B 键盘强迫关门 (GDV10没有这个键)
			} else if(KEY & 0x01 == 0) {            //OPEN
				COMD = 0xFB;						//COMD = # 1111 1011B 键盘开门
			} else if(KEY & 0x08 == 0) {            //STOP
				COMD = 0xFE;						//COMD = # 1111 1110B 键盘停止
			} else if(KEY & 0x02 == 0) {            //CLOSE
				COMD = 0xF7;					//COMD = # 1111 1110B 键盘关门
			}
		}
	}// RET					;如果是 上电显示最大频率数值 模式 则返回
}*/


/****************************************************
*           直流制动处理子程序
*IN  PARAMETER: F16(EEBK) LSP
*MID PARAMETER:
*OUT PARAMETER: RT
*****************************************************/
/*void DC_BK()
{
	AX1 = EFBK[0];								//AX1 = F16_EFBK (F16 直流制动启始频率 放大100倍，出厂45=0.45Hz)
	if(AX1 > #LSP) {
		FG = EFBK[0];							//FG = EFBK (F16 直流制动频率 放大100倍，出厂45=0.45Hz)
		if(FOH <= EFBK[0]) {					//比较FOH 和 EFBK
			CONTROL |= 0x04;					//FOH <= EFBK 则让CONTROL.2=1  CONTROL |= #00000100B;
			if(DCCT > RDCCT[0]) {				//比较DCCT 和 RDCCT? 直流制动时间 和 直流制动时间参考比较
				RT &= 0xFE;				//RT.0=0  RT &= #11111110B;
			}
		}
	} else {
		FG = #LSP;								//FG = LSP (运算最小频率高16bit HZ)
		
		if(FOH <= #LSP) {						//FOH和LSP比较(运算最小频率高16bit Hz)
			RT &= 0xFE;					//RT.0=0  不停止?  RT &= #11111110B;
		}
	}
}*/

/***************************************************
*        继电器输出子程序
*IN  PARAMETER: S  当前门宽位置百分数
*         HAD  继电器配置关系表
*		OUT1 = 启动     继电器 InRush Relay
*		OUT2 = 门开齐   继电器
*		OUT3 = 门关齐   继电器
*		OUT4 = 指定位置 继电器
*自学习完成后，在F41这个百分数位置时继电器使能，断开机械安全触板
*OUT PARAMETER: (RT)
****************************************************/
/*void J123()
{
	AX1 = S - EOPL[0];                						//AX1 = S - EOPL(F53 门开齐继电器信号 95% 放大100倍 950)  OPEN LOCATION
	
	if(S <= EOPL[0]) {
		AL1 = P2_REG[0];									//S < EOPL 则要OUT2继电器失效
		AL1 &= 0xBF;               							//CLR 95%  OUT2 继电器失效 (门开齐 失效) AL1 &= #10111111B;
		P2_REG[0] = AL1;
		AX1 = S - EPOS[0];                					//AX1 = S - EPOS(F41 位置继电器设定30% 放大100倍 300）  OPEN OPS.
		if(S >= EPOS[0]) {
			AX1 = WG_OUT[0];								//如果 S > EPOS 则准备OUT4继电器失效 (指定位置 失效)
			WG_OUT[0] = AX1;
		} else {
			AX1 = WG_OUT[0];
			AX1 |= 0x0080;									//打开OUT4继电器 (指定位置 有效) AX1 |= #0000000010000000B;
			WG_OUT[0] = AX1;
		}
		AX1 = S - ECLL[0];                					//AX1 = S - ECLL(F54 门关齐继电器信号 5% 放大100倍 50)  CLOSE LOCATION
		if(S >= ECLL[0]) {
			AL1 = P2_REG[0];								//S > ECLL 则准备让OUT3继电器失效 (门关齐 失效)
			AL1 &= 0x7F;		     						//OUT3继电器失效 (门关齐 失效) AL1 &= #01111111B;
			AL1 = P2_REG[0];
			RT &= 0xF7;										//关闭自学习推进?
		} else {
			if(COMD.bit1 == 0) {							//不是强迫关门则转 ON0B
				RT |= 0x08;									//当前是强迫关门则让 RT.3=1 继电器延时发信号?	RT |= #00001000B;
				if(JDLY > RJDLY[0]) {						//比较JDLY 和 RJDLY(继电器延时参考)
					AL1 = P2_REG[0];
					AL1 |= 0x80;		      				//打开OUT3继电器 (门关齐 有效) AL1 |= #10000000B;	
					P2_REG[0] = AL1;
				}
			} else {
				AL1 = P2_REG[0];
				AL1 |= 0x80;		      					//打开OUT3继电器 (门关齐 有效) AL1 |= #10000000B;
				P2_REG[0] = AL1;
			}
		}
	} else {
		AL1 = P2_REG[0];
		AL1 |= 0x40;										//AL1 |= #01000000B;
	 	AL1 &= 0x70;	             						//(2003-5-13)  AL1 &= #01111111B;
		P2_REG[0] = AL1;		     						//OUT3继电器失效 (门关齐 失效)

		AX1 = WG_OUT[0];
		AX1 &= 0xFF7F;							//OUT4 继电器失效 (指定位置 失效) AX1 &= #1111111101111111B;
		WG_OUT[0] = AX1;
	}
}*/


/********************************************************
*          开门阶段位置对应速度计算子程序  TRQUE CHECK
*IN  PARAMETER: S (当前门位置百分比，放大1000倍)
*MID PARAMETER:
*OUT PARAMETER: FG (的确是频率Hz)
* 该段程序是详细区分不同的位置区间来算出不同阶段的速度
* 目的是出现S形 润滑的速度曲线，让门机工作更稳定
*********************************************************/
/*void OPEN()
{
	if(S <= DOS0[0]) {									//比较当前门位置S和开门换速位置#0 F25 (%,放大1000倍)
		FG = DOV0[0];									//当前门在 < DOS0区间， 则让FG  =开门换速点#0的速度 F33_DOV0(400) (Hz,放大100倍的表示)
	} else if(S <= DOS01[0]) {							//继续比较当前位置S和开门换速位置#01 (DOS01) 比较{
														//当前门在DOS0 和 DOS01之间按如下处理:
		AX1 = S - DOS0[0];								//AX1 = S - 开门换速位置#0 (F25_DOS0)
		CX1 = AX1 * DOVA[0];							//CX1 = AX1 * 开门平方特性参数DOVA  CX1=乘法的低16bit DX1=乘法的高16bit
		CX1 = CX1 / DOSA[0];             				//CX1 = CX1 / 开门平方特性参数DOSA  CX1 = 除法的商 DX1 = 除法的余数

		CX1 = CX1 * AX1;								//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
		CX1 = CX1 / DOSA[0];							//CX1 = CX1 / 开门平方特性参数DOSA CX1 = 除法的商 DX1 = 除法的余数
		FG = CX1 + DOV0[0];								//FG(当前最大脉冲?) =CX1 + 开门换速点#0速度 F33(Hz,放大100倍的表示)
														//FG = F25_DOV0 +[(S - DOS0)^2 * (DOV1 - DOV0) *2] / (DOS1 - DOS0 )^2
	} else {
		AX1 = S - DOS1[0];								//AX1 = S -开门换速位置#1 F26 (%,放大1000倍)
		if(S <= DOS1[0]) {
			AX1 = -AX1;									//把AX1 每位取反 +1，改变操作数的符号，绝对值不变
			CX1 = AX1 * DOVA[0];						//CX1 = CX1 * 开门平方特性参数DOVA  CX1=乘法的低16bit DX1=乘法的高16bit
			CX1 = CX1 / DOSA[0];						//CX1 = CX1 / 开门平方特性参数DOSA  CX1 = 除法的商 DX1 = 除法的余数

			CX1 = CX1 * AX1;							//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
			CX1 = CX1 / DOSA[0];						//CX1 = CX1 / 开门平方特性蚕食DOSA CX1 = 除法的商 DX1 = 除法的余数
			AX1 = DOV1[0];								//AX1 = AX1 + 开门换速点#1的速度 F34(Hz,放大100倍的表示)
			FG = AX1 - CX1;								//FG  = DOV1 -[(|S - DOS1|^2 * (DOV1 - DOV0) *2] / (DOS1 - DOS0)^2
		} else if(S <= DOS2[0]) {
														//S > 开门换速位置#2 F27(%),则跳转OPEN4
														//当前门在DOS1 - DOS2之间按如下处理:
			FG = DOV1[0];								//否则FG = 开门换速点#1的速度 F34(Hz,放大100倍的表示)
		} else if(S <= DOS23[0]) {						//比较当前门位置和开门换速位置#23
														//当前门在DOS2 - DOS23之间按如下处理
			AX1 = S - DOS2[0];							//AX1 = S - 开门换速位置#2 F27(%)
			CX1 = AX1 * DOVB[0];						//CX1 = CX1 * 开门平方特性参数DOVB 【DOS23阶段是先DOVB，感觉是均减加速度】
			CX1 = CX1 / DOSB[0];						//CX1 = CX1 / 开门平方特性参数DOSB CX1 = 除法的商 DX1 = 除法的余数

			CX1 = CX1 * AX1;							//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
			CX1 = CX1 / DOSB[0];						//CX1 = CX1 / 开门平方特性参数DOSB   CX1 = 除法的商 DX1 = 除法的余数
			AX1 = DOV1[0];								//AX1 = 开门换速点#1的速度 F34(Hz,放大100倍的表示)
			FG = AX1 - CX1;								//FG  = AX1 - CX1
		} else {
			AX1 = S - DOS3[0];							//AX1 = S - 开门换速位置#3 F28(%,放大1000倍)
			if(S <= DOS3[0]) {
				AX1 = -AX1;								//把AX1 每位取反 +1，改变操作数的符号，绝对值不变
				CX1 = AX1 * DOVB[0];					//CX1 = CX1 * 开门平方特性参数DOVB  CX1=乘法的低16bit DX1=乘法的高16bit
				CX1 = CX1 / DOSB[0];					//CX1 = CX1 / 开门平方特性参数DOSB CX1 = 除法的商 DX1 = 除法的余数

				CX1 = CX1 / AX1;						//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
				CX1 = CX1 / DOSB[0];					//CX1 = CX1 / 开门平方特性参数DOSB CX1 = 除法的商 DX1 = 除法的余数
				FG = CX1 + DOV2[0];						//FG  = CX1 + 开门换速点#2的速度 F35(Hz,放大100倍的表示)
			} else {
				FG = DOV2[0];							//当前门在DOS3 之后，则FG  = 开门换速点#2的速度 F35(Hz,放大100倍的表示)
			}
		}
	}
}*/

	 
/********************************************************
*     关门阶段位置对应速度计算子程序  TRQUE CHECK
*IN  PARAMETER: S (当前门位置百分比，放大1000倍)
*MID PARAMETER:
*OUT PARAMETER: FG (的确是频率Hz)
* 该段程序是详细区分不同的位置区间来算出不同阶段的速度
* 目的是出现S形 润滑的速度曲线，让门机工作更稳定
*********************************************************/
/*void CLOSE()
{
	if(S <= DCS0[0]) {												//比较当前门位置S 和关门换速位置#0 F29_DCS0(980)(%,放大1000倍) 98%
		if(S <= DCS01[0]) {											//比较当前门位置和关门还速位置#1 F30(%,放大1000倍)
			AX1 = S - DCS1[0];										//AX1 = S - 关门换速位置#1 F30 (%,放大1000倍)
			
			if(S >= DCS1[0]) {
				CX1 = AX1 * DCVA[0];								//CX1 = CX1 * 关门平方特性参数DCVA  CX1=乘法的低16bit DX1=乘法的高16bit
				CX1 = CX1 / DCSA[0];								//CX1 = CX1 / 关门平方特性参数DCSA CX1 = 除法的商 DX1 = 除法的余数
				CX1 = CX1 * AX1;									//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
				CX1 = CX1 / DCSA[0];								//CX1 = CX1 / 关门平方特性DCSA CX1 = 除法的商 DX1 = 除法的余数
				AX1 = DCV1[0];										//AX1 = 关门换速位置#1的速度 F38 (Hz,放大100倍)
				FG = AX1 - CX1;										//FG  = AX1 - CX1
			} else {
				if(S > DCS2[0]) {									//比较当前门位置S和关门换速位置#2 F31(%,放大1000倍)
					FG = DCV1[0];									//FG = 关门换速位置#1的速度 F38 (Hz,放大100倍)
				} else {
					if(S > DCS23[0]) {								//比较当前门位置和关门换速位置#23
						AX1 = S - DCS2[0];							//AX1 = S - 关门换速位置#2 F31(%,放大1000倍)
						AX1 = -AX1;									//把AX1 每位取反 +1，改变操作数的符号，绝对值不变
						CX1 = AX1 * DCVB[0];						//CX1 = AX1 - 关门平方特性参数DCVB 【减速开始】 CX1=乘法的低16bit DX1=乘法的高16bit
						CX1 = CX1 / DCSB[0];						//CX1 = CX1 - 关门平方特性参数DCSB CX1 = 除法的商 DX1 = 除法的余数
						CX1 = CX1 * AX1;							//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
						CX1 = CX1 / DCSB[0];						//CX1 = CX1 / 关门平方特性参数DCSB CX1 = 除法的商 DX1 = 除法的余数
						AX1 = DCV1[0];								//AX1 = 关门换速位置#1的速度 F38 (Hz,放大100倍)
						FG = AX1 - CX1;								//FG  = AX1 - CX1
					} else {
						if(S > DCS3[0]) {							//AX1 = S - 关门换速位置#3 F32(%,放大1000倍)
							CX1 = AX1 * DCVB[0];					//CX1 = AX1 * 关门平方特性参数DCVB  CX1=乘法的低16bit DX1=乘法的高16bit
							CX1 = CX1 / DCSB[0];					//CX1 = CX1 / 关门平方特性参数DCSB CX1 = 除法的商 DX1 = 除法的余数
							CX1 = CX1 * AX1;						//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
							CX1 = CX1 / DCSB[0];					//CX1 = CX1 / 关门平方特性参数DCSB CX1 = 除法的商 DX1 = 除法的余数
							FG = CX1 + DCV2[0];						//FG  = CX1 + 关门换速位置#2的速度 F39 (Hz,放大100倍)
						} else {
							FG = DCV2[0];							//当前门在 < DCS3 位置， 则FG  = 关门换速位置#2的速度 F39 (Hz,放大100倍)
						}
					}
				}
			}
			
		} else {													//S<= 关门换速位置#1 F30(%),则调整CLOSE2
																	//当前门在DCS01 - DCS0之间按如下处理:
			AX1 = S - DCS0[0];										//AX1 = S - 关门换速位置#0 F29 (%,放大1000倍)
			AX1	= -AX1;												//把AX1 每位取反 +1，改变操作数的符号，取绝对值
			CX1 = AX1 * DCVA[0];									//CX1 = AX1 * 关门平方特性参数DCVA 【应该是均加速】CX1=乘法的低16bit DX1=乘法的高16bit
			CX1 = CX1 / DCSA[0];									//CX1 = CX1 / 关门平方特性参数DCSA CX1 = 除法的商 DX1 = 除法的余数
			CX1 = CX1 / AX1;										//CX1 = CX1 * AX1  CX1=乘法的低16bit DX1=乘法的高16bit
			CX1 = CX1 / DCSA[0];									//CX1 = CX1 / 关门平方特性参数DCSA CX1 = 除法的商 DX1 = 除法的余数
			FG = CX1 + DCV0[0];										//FG  = F37_DCV0 + CX1	
		}
	} else {
		FG = DCV0[0];												//当前门在 > DCS0 区间 则让FG = 关门换速位置#0的速度 F37 (Hz,放大100倍)
	}
}*/


/********************************************
*         找门宽自学习处理子程序
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER:
*********************************************/
/*void STUDY()
{
	 ADCL0();									//频率跳变点预处理--自学习常规加减速处理子程序
	 
	 if(SD_FLG == 0) {							//是自学习开始阶段?
		//******自学习第一过程****找门宽自学习开始阶段SD_FLG=0***

		if(RT & 0x10 == 0) {	    	        //自学习推进无效  RT.bit4 != 1
			FG = EFG0[0];						//自学习推进无效，则让FG = EFG0 (自学习常规速度F46 出厂12Hz 放大100倍 1200) SELF_STUDY CLOSE SPEED
			
			if(FOH == FG && DTS ==0) {			//FOH 不等于 FG,  则转返回
												//(编码器遇到障碍物,DTS=0), FOH = FG,如果DTS  不等于0 则转返回
				RT |= 0x10;  	        		//FOH = FG,  遇到障碍物 则让RT.4=1(ZERO is found)
				NUMBER = 0;                 	//把当前获得的编码器脉冲数置为0  29/1-2000
				TIMER1[0] = NUMBER;
				S = 0;							//没有遇到障碍物，清除门宽位置百分数寄存器S
			}
		} else {
			FG = EFG1[0];						//RT.0=1, 则FG = EFG1 (F47 自学习推进速度Hz 出厂3Hz 放大100倍 300) SELF_STUDY PUSH SPEED
			if(SPT > RSPT[0]) {					//SPT = RSPT ?(自学习时间参考)
				NUMBER = 0;						//把当前获得的编码器脉冲数置为0
				TIMER1[0] = NUMBER;
				FSSK = 0;						//清除FSSK 这个变量在本程序中没有特别用意，应该是ZDK之前采用CAPTURE方式处理编码器时使用的
				S = 0;							//清除门宽位置百分数寄存器S
				J123();							//调用继电器输出子程序
				if(COMD & 0x04 == 0) {          //是开门命令 COMD.bit2 == 0
					SD_FLG = 1;	        		//门关齐结束第一过程完成
					RT &= 0xEF;					//关闭自学习推进? RT.4=1没有找到零点?
				}
			}
		}
	 } else {
		/*** 找门宽自学习第二过程 SD_FLG=1 直到完成 自学习 SD_FLG=2****
		CL1 = ESDY[0];							//CL1 = ESDY(F56 自学习记忆模式0,1,2出厂是0 自学习脉冲数保存F50 EEPROM单元 2 不保存 1 条件保存)
		if(CL1 != 2) {							//F56 自学习记忆模式不是2 则转SD_B0
			if(MST & 0x01 != 0) {				//关门方向 则跳转SD_B2   MST.bit0 != 0
				FG = #LSP;						//关门方向 则  FG = LSP (运算最小频率高16bit HZ)
				if(FOH <= #LSP) {				//FOH和LSP比较
					MST &= 0xFE;         		//置开门方向
				}								//FOH > LSP 则返回
			} else {
				if(RT & 0x10 == 0) {         	//关门方向 RT.4=1? 则转SD_B4 (自学习推进有效?)   RT.bit4 == 0
					FG = EFG0[0];              	//RT.4=0, 则让FG = EFG0 (自学习常规速度F46 出厂12Hz 放大100倍 1200)
					if(FOH == FG && DTS == 0) {
						RT |= 0x10;				//遇到障碍物 则则让RT.4=1 (自学习推进有效?) 后返回
					}
				} else {
					FG = EFG1[0];				//FG = EFG1 (F47 自学习推进速度Hz 出厂3Hz 放大100倍 300)
					
					if(SPT > RSPT[0]) {			//SPT = RSPT ?(自学习时间参考)
						if(NUMBER <= 10) {		//当前获得的编码器脉冲数 和 10比较
							NUMBER = 8;  		//如果当前获得的编码器脉冲<10，则让当前获得的编码器脉冲置为8   NO-PULSE
							TIMER1[0] = NUMBER;
							AX1 = NUMBER;		//AX1 = NUMBER (当前获得的编码器脉冲数)
						} else {
							CX1 = #NMAX;		//CX1 = 本机支持的最大脉冲数
							DX1 = ENULL[0];		//DX1 = ENULL (F14 门宽矫正%) 出厂2% 放大1000倍 2
							AX1 = DX1 * 5;		//AX1 = ENULL(%) * 5  CX1=乘法的低16bit DX1=乘法的高16bit
							DX1 = CX1 - AX1;	//??? DX1 = 本机支持的最大脉冲数 - ENULL(%)*5    16/3-2003
	 
							AX1 = NUMBER * DX1;			//AX1 = NUMBER * DX1  当前的获得的编码器脉冲数 * DX1  CX1=乘法的低16bit DX1=乘法的高16bit
							AX1 = AX1 / CX1;			//AX1 =(NUMBER * DX1) / #NMAX AX1 = 除法的商 BX1 = 除法的余数
							NUMBER = AX1;				//经过F14门宽矫正后的脉冲数置给TIMER1
							TIMER1[0] = NUMBER;
						}
						
						CL1 = ESDY[0];					//CL1 = ESDY(F56 自学习记忆模式0,1,2出厂是0)
						
						if(CL1 == 0) {
							EPULSE[0] = NUMBER;			//F56=0(自学习记忆模式)条件下  把当前获得的编码器脉冲数置给F50 EPULSE(自学习总脉冲数)
							AX9 = #XPULSE;				//XPULSE代表F50在EEPROM的地址，准备把当前获得的编码器脉冲数保存到F50中
							EWRE();						//把当前或的编码器脉冲数保存到F50 EEPROM单元中
							DELAY2(2);					//延时2.4 * 2= 4.8ms
							SD_FLG = 2;               	//设置自学习为结束状态  END  SELF-STUDY
							RT &= 0xEF;      		//关闭自学习推进标志位  END  AUTO-STUDY PUSH
						} else if(CL1 == 1) {
							CX1 = EPULSE[0];			//CX1 = F50 (本机自学习总脉冲数,出厂是1000)
							DX1 = NUMBER - CX1;			//DX1 = 当前获得的编码器脉冲数 - EPULSE
							if(NUMBER < CX1) {
								DX1 = -DX1;
							}
							
							if(DX1 > 20) {
								SD_FLG = 0;				//(NUMBER - F50) >  20, 当前置为自学习开始阶段?
								RT &= 0xFE;		//RT.0=0 停止?
							} else {
								SEPULSE[0] = NUMBER;			//F56=0(自学习记忆模式)条件下  把当前获得的编码器脉冲数置给F50 EPULSE(自学习总脉冲数)
								AX9 = #XPULSE;				//XPULSE代表F50在EEPROM的地址，准备把当前获得的编码器脉冲数保存到F50中
								EWRE();						//把当前或的编码器脉冲数保存到F50 EEPROM单元中
								DELAY2(2);					//延时2.4 * 2= 4.8ms
								SD_FLG = 2;               	//设置自学习为结束状态  END  SELF-STUDY
								RT &= 0xEF;      		//关闭自学习推进标志位  END  AUTO-STUDY PUSH
							}
						} else {
							SD_FLG = 2;               					//设置自学习为结束状态  END  SELF-STUDY
							RT &= 0xEF;      						//关闭自学习推进标志位  END  AUTO-STUDY PUSH
						}	
					} 
				}
			}
		} else {								
			SD_FLG = 2;               					//设置自学习为结束状态  END  SELF-STUDY
			RT &= 0xEF;      						//关闭自学习推进标志位  END  AUTO-STUDY PUSH
		}
	 }
}*/





/**************************************************************
*         力矩监控检查子程序    DOOR TRQUE CHECK
*IN  PARAMETER: S COMD MST RT
*MID PARAMETER: AL1
*OUT PARAMETER: RT
*   这段程序目的不清楚!
***************************************************************/
/*void OBSTCL()
{
	AL1 = EOBSTCL[0];						//AL1 = 当前力矩监控标志F24 (0有效 1无效)
	
	//if(AL1.bit0 != 0 && S > 80 && S <= 800 && COMD.bit1 == 1 && MST.bit0 == 1 && DTS != 0) {		//F24=1力矩监控有效	
	if(AL1 & 0x01 != 0 && S > 80 && S <= 800 && COMD & 0x02 == 1 && MST & 0x01 == 1 && DTS != 0) {		//F24=1力矩监控有效					
	//如果AL1.0=1 F24=1力矩监控有效，则检查当前门宽百分比 8% < S <= 80%，COMD不是强迫关门命令，MST处在关门方向，DTS编码器没有停顿
		RT |= 0x02;					//遇到障碍物  则让RT.1=1  力矩监控标志有效? (有障碍物，编码器停顿)  RT |= #00000010B;

	} else {
		RT &= 0xFD;					//没有遇到障碍物 则让RT.1=0  力矩监控无效?  RT &= #11111101B;
	}
}*/

/*************************************************************
*        低频力矩补偿子程序  LOW FRQ. TRQ BOOST
*IN  PARAMETER: SD_FLG，MST, TZ
*MID PARAMETER:
*OUT PARAMETER: VTRQ
* 把开关门的低频转矩补偿额度F04_EOTRQ( F52_ECTRQ) * TZ = VTRQ
* 这和传统的V-F曲线感觉与不同
* 电压最后转换成TZ的变化额度, TZ本身不变，应该是改变PWM占空比
**************************************************************/
/*void BOOST()
{
	if(SD_FLG != 2) {							//自学习没有结束  (意味如果是自学习阶段只做开门低频转矩补偿)
		CX1 = EOTRQ[0];							//把当前开门低频转矩补偿F04 赋予CX1 (%) (放大1000倍，出厂80 8%)	
	} else {									//自动运行阶段要区分是开门方向 还是 关门方向 做不同的 低频转矩补偿处理
		if(MST.bit0 == 1) {
			CX1 = ECTRQ[0];						//自动运行关门方向 CX1 = 关门低频转矩补偿F52 (%)(放大1000倍，出厂80 8%)
		} else {
			CX1 = EOTRQ[0];						//把当前开门低频转矩补偿F04 赋予CX1 (%) (放大1000倍，出厂80 8%)
		}
	}

	AX1 = CX1 * TZ;								//AX1 = CX1 * TZ ( % * S )  AX1=乘法的低16bit BX1=乘法的高16bit
	AX1 = AX1 / 1000;							//AX1 = AX1 /1000               AX1 = 除法的商 BX1 = 除法的余数
	VTRQ = AX1;									//VTRQ = AX1  难道VTRQ是秒单位?
												//VTRQ开门 = (F04_EOTRQ(80) * TZ ) /1000 的商
												//VTRQ关门 = (F52_ECTRQ(80) * TZ ) /1000 的商 
}*/

/**************************************************************
*            V/F特性处理子程序  V/F CHARACTISTICS
*IN  PARAMETER: TZ,VTRQ(转矩补偿) SD_FLG
*MID PARAMETER:
*OUT PARAMETER: KV1 V-F曲线斜率*T  (周期V-F系数)
*主要注意的是，这个V-F曲线是在刨除低频力矩补偿后的曲线斜率
*具体可以看笔计本阐述，相当于传统的V-F曲线*T(载波周期)
***************************************************************/
/*void VANDF()
{
	AX1 = TZ - VTRQ;						//AX1 = TZ - VTRQ
	AX1 = AX1 * 16384;						//AX1 = ( TZ - VTRQ ) * 16384  AX1=乘法的低16bit BX1=乘法的高16bit
											//16384 = 65536 /4
											
	if(SD_FLG == 2) {						//检查自学习是否结束 自学习结束, 意味是自动运行阶段 就跳转VANDF1
											//自学习阶段的V-F曲线处理方式有不同!
		if(MST.bit0 == 1) {					//MST.0=1 关门方向 
			CX1 = ECF1M[0];					//CX1 = 当前关门基本转矩频率(Hz) F51 (放大100倍65HZ 6500)
		} else {
			CX1 = EOF1M[0];					//MST.0=0 开门方向 则CX1 = 当前开门基本转矩频率F03 (放大100倍55HZ 5500)赋予CX1
		}
	} else {
		CX1 = EF1M0[0];						//CX1 = 把当前自学习力矩频率F49 (Hz 出厂50Hz 放大100倍 5000)
	}
	
	AX1 = AX1 / CX1;						//AX1 = AX1 / CV1  AX1 = 除法的商 BX1 = 除法的余数de
	KV1 = AX1;								//KV1 = (( TZ - VTRQ ) * 16384 ) / F51_ECF1M (or F03_EOF1M or F49_EF1M) [都是经过放大100倍]的商
}*/

/**************************************************************
*    间接(RT.2=1判断有效)数码管数据发送显示处理子程序
*IN  PARAMETER:
*OUT PARAMETER:
***************************************************************/
void DISPLY() 
{
	//if(RT & 0x04 != 0) {
		DISPLY1();										//RT.2=1 则转具体显示发送处理
	//}
}

/**************************************************************
*      直接(强制RT.2=0)数码管数据发送显示处理子程序
*IN  PARAMETER: DBUF1,DBUF2,DBUF3,DBUF4,DDBUG5
*OUT PARAMETER: QX,QZ,
*OUT PARAMETER: DBUF1,DBUF2,DBUF3,DBUF4,DDBUG5
***************************************************************/
void DISPLY1()
{ 
	char i;
	
	GpioDataRegs.GPASET.bit.GPIO28 = 1;   		//清除GPIO28
	
	for(i = 4; i >= 0; i --) {
		PUT8(DBUF[i]);                     		//调用发送8bit子函数
	}
	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;   	//清除GPIO28
}

/**************************************************************
*            显示内容8次推送字节子程序
*IN  PARAMETER: QZ(需要显示的BUFF内容)
*MID PARAMETER: QX,QY
*OUT PARAMETER:
***************************************************************/
void PUT8(unsigned char data)
{
	unsigned char num;
	unsigned char i;

	num = data;
	
	for(i = 0; i < 8; i ++) {
		GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;				//发低电平
		delay_loop(1);
		num = data & (1 << i); 								//1左移i位，保留第i位的值

		if(num) {    										//如果第i位是1则GPIO30置1，如果是0则清零
			GpioDataRegs.GPASET.bit.GPIO30 = 1; 
		}
		else { 
			GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
		}
		
		delay_loop(1);
		GpioDataRegs.GPASET.bit.GPIO29 = 1;
		delay_loop(2);
	}
}


/*****************************************************
*          VDC MEASUREMENT 直流母线电压测量
*IN  PARAMETER: 根据核奥达说明书支持输入交流180V - 265V
*MID PARAMETER:
*OUT PARAMETER: VDC  公式=(1023*VDC)/5(为采样参考电压)10bitAD情况下
******************************************************/
void VADCH()
{
	AdcRegs.ADCSOCFRC1.bit.SOC0 = 1;
    
	while(AdcRegs.ADCINTFLG.bit.ADCINT1 == 0) {}
	
	AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;		//Clear ADCINT1 flag reinitialize for next SOC
 	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
 	  
	VDC = AdcResult.ADCRESULT0;
	//Voltage1[ConversionCount1] = AdcResult.ADCRESULT0; 	
  	//volt_A0 = (3.3 * Voltage1[ConversionCount1]) / 4096;

	
}




/******************************************************
*          AUTO模式切换程序
*IN   PARAMETER: ECMD
*MID  PARAMETER: 
*OUT  PARAMETER: ECMD
*			ECMD=1 点动Key_JOG(本软件屏蔽?)
*			ECMD=2 外部端子继电器控制
*			ECMD=0 键盘控制
*******************************************************/
void AUTO()
{
														//读AUTO键盘(高保持原来F11的参数不变,底则把F11对应的RAM单元ECMD变为2)
														//P0.5=AUTO
    if(GpioDataRegs.GPADAT.bit.GPIO7 == 1) {           	//检查AUTO键有没有按下?
		E2M[ECMD] = 0;									//AUTO平时为高，则把控制模式ECMD设置为0, 键盘控制模式
	} else {
		E2M[ECMD] = 2;									//ECMD ( F11 控制模式),注意并不是写到F11 对应的EEPROM单元，掉电不恢复
	}
}




