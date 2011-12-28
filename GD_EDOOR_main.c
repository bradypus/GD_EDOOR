
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


int16 AX8 = 0;	 				//��Ŷ�Ӧ����������ò���  ;BACK FOR RAM
int16 AX9 = 0;	 				//6AH ��Ŷ�Ӧ������
char AL9 = 0;
char AH9 = 0;


Uint16 FLO = 0;			//���EEPROM RAM���е�EFLO(��СƵ����ֵ)
Uint16 FOM = 0;      	//���EEPROM RAM���е�EFOM(���Ƶ����ֵ)
Uint16 VTRQ = 0;	   	//Johnson 2011-10 ��Ƶ����������ѹ��ֵ

Uint16 KA = 0;	   		//2011-10 Johnson �������û���õ�  32H
Uint16 INCFOL = 0;	   	//2011-10 Johnson ��ǰ����״̬�µ�����ֵ�ĵ�16bit(Hz)  34H
Uint16 DECFOL = 0;	   	//2011-10 Johnson ��ǰ����״̬�µļ���ֵ�ĵ�16bit(Hz)  38H
Uint16 DECFOH = 0;	   	//2011-10 Johnson ��ǰ����״̬�µļ���ֵ�ĸ�16bit(Hz)  3AH

Uint16 DFINR1 = 0;	   	//2011-10 Johnson ???? ���ֻ�������ط��õ�?   3CH

Uint16 NUMBER = 0;	   	//2011-10 Johnson ��ǰ��õı�����������       3EH

Uint16 T1 = 0;	   		//40H SVPWM�����ALPA���ʱ��(�������ñȽϼ�������ֵ����ʾ)
Uint16 T2 = 0;	   		//42H SVPWM�����BELTA���ʱ��(�������ñȽϼ�������ֵ����ʾ)
Uint16 TZ = 0;	   		//44H �൱���ز����� (ʵ����Ҫд��WG_RELOAD����ֵ)(�����ز����ڵļ��㹫ʽ�����Ŀ�ʼ)  
Uint16 S = 0;	    	//46H ����λ������ٷ��� (�ٷֱ��ǷŴ�1000��)

Uint16 FIA = 0;		   	//2011-10 Johnson SVPWMʸ�������еĽǶ�THETA������  48H
Uint16 TAB = 0;	 	   	//2011-10 Johnson �̶�Ϊ2048 �о��Ǳ�����1024�����SINE��ĵ���2��  4AH

Uint16 KV1 = 0;	 		//2011-10 Johnson �Ǹ���VANDF�����������VF����б��*T (����v-fб��ϵ��) 4CH
Uint16 FGC = 0;	   		//2011-10 Johnson ��ǰ�����е��Ƶ��(�ٶ�)�Ļ����� (Hz) 4EH
Uint16 FOL = 0;	 		//2011-10 Johnson �ϴ����е��Ƶ��(�ٶ�)��16bit(Hz)     50H
Uint16 FOH = 0;			//2011-10 Johnson �ϴ����е��Ƶ��(�ٶ�)��16bit(Hz)     52H
Uint16 FG = 0;		   	//2011-10 Johnson ��ǰ�����е��Ƶ��(�ٶ�) (Hz)         54H
Uint16 DTS = 0;	   		//2011-10 Johnson ��ŵ��ϴα������������뱾�α������������Ĳ� (�����������ϰ���,DTS=0) 56H


int16 AX3 = 0;	   		//2011-10 Johnson �����ϴα����������� 58H
int16 BX3 = 0;	 		//2011-10 Johnson ������û���õ� 5AH
Uint16 FUN = 0;			//5CH ��ŵ�ǰ�Ĺ�������ֵ

Uint16 DLY = 0;	 		//5EH �����ʱ��ѭ������
Uint16 DLYS = 0;	 	//60H �����ʱ��ѭ������
Uint16 FSSK = 0;		//2011-10 Johnson Ƶ�����Կ�  ������Ҫ��ZDK�ڲ���CAPTURE��ʽ��������������ʱ�õ� ���������û������ 62H
Uint16 NMAX = 30000;	//#NMAX����C196.inc����30000  ASMUUE A MAX NUMBER WHEN ON
Uint16 TDC = 0;			//2011-10 Johnson �о��Ǻ�����ʱ���йصĲ���  64H
Uint16 JDLY = 0;	 	//2011-10 Johnson �̵�����ʱʱ��(s) 66H  Relay Delay

						

Uint16 DCCT = 0;	 	//2011-10 ֱ���ƶ�ʱ�� (��ӦRDCCT)  6CH
Uint16 SPT = 0;	   		//2011-10 Johnson��ӦRSPT ��ѧϰ�ƽ�ʱ�� (S)  6EH

Uint16 DOCPT = 0;	 	//2011-10 Johnson ��ӦRDOPT  70H
Uint16 OBPT = 0;	 	//72H ���ؼ��(OBSTCL)�ƽ�ʱ�� PUSH TIME
Uint16 VRMS = 0;	 	//2011-10 Johnson ��ѹ�ľ�����ֵ�������ñ������ֵ,�Ƿ�ֵ��0.707 74H
Uint16 FIX = 0;	      	//2011-10 Johnson SVPWMʸ�������еĽǶ�THETA    76H
Uint16 FIY = 0;	       	//2011-10 Johnson SVPWMʸ�������еĽǶ�60-THETA 78H
Uint16 VDC = 0;	        //2011-10 Johnson ֱ��ĸ�ߵ�ѹ 79H

char SECTOR = 0;    	//2011-10 Johnson �����ű�־ �״���SECTOR.0=1Ȼ�����ƣ�����1-6��������֮���ٴδ�1��ʼ 7AH        75H
char MODE = 0;	   		//2011-10 Johnson �������������û��ʹ��           7BH        76H
char NEWVECT = 0;	   	//2011-10 Johnson ����WG(SPWM)���η����жϷ��������л����ر�־       7CH        77H

char RT = 0;	 	   	//2011-10 Johnson �о��Ǹ�����ģʽ�µ�״̬�л���־ 7DH        78H

char KEY = 0;	 	   	//2011-10 Johnson ��ŵ�ǰ��ȡ����Ч��ֵ(�����ֵ�Ǿ���ת���ķ������ʹ��)7EH       79H
char FRSH = 0;	 	   	//2011-10 Johnson ʱ�������  ��ʱ��˸ˢ�±�־�����õ�FRSH.4 =10000=2ms FRSH.3=1000=1ms 7FH       7AH
char DYCT = 0;	 	   	//2011-10 Johnson ��ʾʱ�������                   80H	7BH
char CONTROL = 0;		//2011-10 Johnson �о�����������Ŀ����л�״̬     81H	7CH
char MST = 0;           //2011-10 Johnson ���������״̬��־    MST.0=0���� MST.0=1����       82H	7DH
						//�˲���ʷ�汾֮ǰZDK�����MST����CAPTURE��ʽ�����������ʹ�� MST=#XXXX RZ RY RX DIR
char DSCD = 0;	 		//2011-10 Johnson ��ӦEDSCD ��ʾģʽ?               83H	7EH
char COMD = 0;	 		//2011-10 Johnson �ⲿ���������־�Ĵ���(����λ��Ч)�������ֵ�ǰ�Ǽ��̿��� ���Ǽ��̵㶯 �����ⲿ����84H		7FH
						//���۵�ǰ���ⲿ���ӿ��ƻ��Ǽ��̿��ƶ�ͳһӳ���COMD�Ĳ�ͬ��־λ��
						//COMD= # 1111 1111 B û���κο�������
						//COMD= # 1111 1110 B ֹͣ����
						//COMD= # 1111 1101 B ǿ�ȹ������� (GBV10��֧���������:����û�������������Ҳû���������)
						//COMD= # 1111 1011 B ��������
						//COMD= # 1111 0111 B ��������
char AL0 = 0;	 		//2011-10 Johnson �ⲿ���ϱ�־�Ĵ��� #1111 1101B(��ѹ) #1111 1110B(����)      85H		80H

char SMT = 0;	 		//2011-10 Johnson ���������Ȼֻ��һ���ط�����, �������SMT��־û����         86H		81H
char AL2 = 0;	 		//2011-10 Johnson �����������������û��ʹ��                 87H		82H
//char QX = 0;	 		//2011-10 Johnson ��SIN��SCT��ʾ������ݺ�ʱ�Ӵ����еĻ����� 88H		83H
//char QY = 0;	 		//2011-10 Johnson ��SIN��SCT��ʾ������ݺ�ʱ�Ӵ����еĻ����� 89H		84H
//char QZ = 0;	 		//2011-10 Johnson ��SIN��SCT��ʾ������ݺ�ʱ�Ӵ����еĻ����� 9AH		85H

char DBUF[5] = {0};	 
//DBUF1	 EQU	DBUF+1	   ;9CH           87H ��һ�������ʾ����
//DBUF2	 EQU	DBUF1+1    ;9DH           88H �ڶ��������ʾ����
//DBUF3	 EQU	DBUF2+1    ;9EH           89H �����������ʾ����
//DBUF4	 EQU	DBUF3+1    ;9FH           8AH �����������ʾ����
//DBUF5	 EQU	DBUF4+1    ;0A0H          8BH �����������ʾ����
char TERMAL = 0; 		//2011-10 Johnson ������������״̬��־ TERMAL=#1111 XXXX ����λ�ǵ�ǰ���ӿ���״̬	0A1H          8CH
						//����HAD��:  TERMAL=#1111 IN4   IN3   IN2   IN1   ������Ч�������Ǹߣ����������ǵ͵�CPU,����ͨ��NOT��ʽȡ����������ЧΪ��
						//����GDV10��:  TERMAL=#1111 IN4   IN3   AUTO   GND  ������Ч�������ǵͣ������������Ǹ�CPU,����ͨ��NOT��ʽȡ����������ЧΪ��
						//����  ����  AUTO(ǿ�ȹ��� ) ����(ֹͣ)
char SAMP_T = 0;  		//2011-10 Johnson ȡ��ʱ�䣬����ֻ�������ط��õ���			0A2H          8DH
char SD_FLG = 0;		//2011-10 Johnson ��ѧϰ��־(���ò���0,1,2)       			0A3H          8EH
						//2011-10 SD_FLG= 0:��ѧϰ��ʼ��1:�Ź��������һ���̡�2:��ѧϰ����
char APPL = 0;	 		//0A4H            ���EEPROM RAM�е�F58(�ɵ�Ƶģʽ����): 1->�ϵ���ʾ���������Ŀ,0->�ϵ���ʾ���Ƶ��ֵ

Uint16 FX = 0;	 		//0A5H            ���ӵĻ���Ĵ���(������AT93C66��EESK) 2006-10-6 JJ

Uint16 PASS = 0;		//0A7H EEPROM��Ȩ����,���û����Ȩ���룬�򿪻������û����ʾ,����ͨ����д���ʵ����Ȩ��3FH(63)-->(C8H)

Uint16 LSP0 = 0;	 	//0A8H 2011-10 Johnson �����õ���СƵ�ʵ�16bit CACULATING MIN FREQ-LO
Uint16 LSP = 0;	 		//0A9H 2010-10 Johnson �����õ���СƵ�ʸ�16bit CACULATING MIN FREQ-HI.running min freq.

Uint16 FUNM = 0;	 	//0AAH 2011-10 Johnson ��ʱʵ��ָ�������ZDK ��ֵ  58       ;���������Ŀ����58                  63

//Uint16 TMIN0 = 0; 		//0ABH 2011-10 Johnson ������û���õ�

                           
/*====================================================
*        EEPROM ��ַ����(��RAM��ַ��Ӧ)
* E��ͷ��ʾ��EEPROM (AT93C66 16bit Data)
* �ö�RAM��Ŵ�EEPROM��ȡ�Ķ�Ӧ����������ò���
* ʵ�ʳ��������ж�Ӧ���м�RAM���ͬ���Ĳ�������
* �������. (ȫ��Ϊż����ַ=FUN+FUN)
*====================================================*/
Uint16 E2M[160] = {0};	 						//0ACH �����EEPROM��ȡ�Ĺ�������ֵ��ŵ�RAM��Ԫ�׵�ַ     REFLECT THE SERIES-E2ROM


#define 	EFG		0     		//00 2011-10 Johnson "�趨Ƶ��"��ʲô��˼? �趨�������Ƶ����(Hz �Ŵ�100��)�����Լ�����,����Ϊ5000,ֻ������ѧϰ�����г����Լ��ı�
#define 	EFLO    1      		//01 2011-10 Johnson ���ʼ��Ƶ�� (Hz���Ŵ�100��)        MIN FREQ.
#define 	EFOM    2      		//02 2011-10 Johnson �������Ƶ�� (Hz,�Ŵ�100��)         MAX FREQ.
#define 	EOF1M   3      		//03 ���Ż���ת��Ƶ��(Hz) ��ֵԽ��ת��ԽС����֮��Ȼ     OPEN BASE FREQ
#define		EOTRQ   4      		//04 ���ŵ�Ƶת�ز���(%)  ��ֵԽ��ת��Խ��               OPEN TRQUE BOOST
#define		EACL0   5      		//05 ��ѧϰ�������ʱ��(S)    �����ò�����ʹ��ѧϰ��˳������ STUDY SPEED ACL
#define		EDCL0   6      		//06 ��ѧϰ�������ʱ��(S)    �����ò�����ʹ��ѧϰ��˳������ ----  ----- DCL

#define		EACL1   7		    //07 ��������(�������ʱ��)(��������) �����ò�����ʹ����Ļ���������ؼ��ʱ����ƽ���������ߡ�REVERSING SPEED ACL
#define		EDCL1   8	      	//08 ��ת����ʱ��(S)  �����ò�����ʹ����Ļ���������ؼ��ʱ����ƽ���������ߡ� -------	 DCL

#define		EACL2   9      		//09 �������ʱ��(S)  �����ò�����ʹ�ų���������ƽ���������ߡ�    NORMAL SPEED	 ACL
#define		EDCL2   10      	//10 �������ʱ��(S)  �����ò�����ʹ�ų���������ƽ���������ߡ�    ----  -------	 DCL

#define		ECMD    11      	//11 ����ģʽ  2=�ⲿ�̵������ӿ��ƣ�1�ǵ㶯(Ŀǰ��������)��0�Ǽ��̿��� ����ģʽ(����ĳ�����ӿ����趨) COMMAND MODE
								//HAD: F11=0������ģʽ��F11=1�ֶ�ģʽ��F11=2����ģʽ(���ӿ���ģʽ)
#define		ESTT    12      	//12 ��������(����ȡ��?) ֹͣģʽ 0,1  ����������ֹͣ��ʽ������ֹͣ���Ǽ���ֹͣ?    STOP MODE
#define		EOPM    13     		//13 2011-10 Johnson �����������û���õ�  ��������(����ȡ��?)  0-SINGAL 1-DOUBLE
								//λ�ò�����ʽ: 0=��ͨ����1=˫�ص�ͨ����2=˫ͨ����������2
#define		ENULL   14      	//14 �ſ����(%)   �˲�����������ҪΪ������ѧϰ���������ڱ���������е������ɵ��������       PULSE OFFSET

#define		EEXCM   15      	//15 ��������(�ⲿͨ��ģʽ)(��������)                EXT COM MODE 22/12-1999
#define		EFBK	16      	//16 ��������(ֱ���ƶ���ʼƵ��)(��������)(Hz)�Ŵ�100��   BRAKE FREQ.
								//�ƶ�Ƶ��һ����ָ��Ƶ����ֱ���ƶ���ʼƵ�ʣ�����Ƶ�����Ƶ�ʵ���ͣ��ֱ���ƶ���ʼƵ��ʱ����Ƶ��������ֱ���ƶ����ܣ�
								//�����ע��ֱ��������������̬ɲ������ͣ��
								//ֱ���ƶ���һ��ָ����Ƶ�����Ƶ�ʽӽ�Ϊ�㣬���ת�ٽ��͵�һ����ֵʱ����Ƶ�������첽�綯������������ͨ��ֱ�����γ�
								//��ֹ�ų�����ʱ�綯�������ܺ��ƶ�״̬��ת����ת���и�þ�ֹ�ų��������ƶ�ת�أ�ʹ�綯��Ѹ��ֹͣ��

#define		ETBK    17      	//17 ��������(ֱ���ƶ�ʱ��)(��������)                BRAKE TIME
#define		EVBK    18      	//18 ��������(ֱ���ƶ�ǿ��)(��������)(Hz)            BRAKE AMOUNT
#define		EFC     19      	//19 �ز�Ƶ��(Hz,�Ŵ�100��)                          CARRIER FREQ.

#define		ESAV    20    		//20 ���ݱ���(418���޸ı���) (888�ָ�������ֵ)(8080ȫ����ʾ������) DATA SAVE
								//129H �ִ洢
#define		XSAV    20     		//����F20��Ӧ��EEPROM��ʼ��ַ, ��������Ҫ����16bit, ���Ե�ַ��ż��. ����֮����Ҫ16bit����Ϊ���ܲ����и�����Ҫ��16bit����

#define 	EDELY   21       	//21 ��������(ͨ����ʱ)(��������)�����δ�� [����汾����ʾʱ��ĳ��̣�Ҳ��Ԥ���̵���OUT1����ʱʱ�� by ����?]  SWITIH ON DELAY
#define		ESYSTM  22      	//22 ��������(���������ʽ����/����ģʽ)(��������) �����δ��  NORMAL/TEST MODE
#define		EDEADT  23			//23 ��������(����ʱ��)(us)(��������)                          DEAD TIME

#define		EOBSTCL 24 			//24 �Ż����ؼ�� 0:���ؼ����Ч 1:���ؼ����Ч                DOOR TRQUE SUP
					//�����Źر�ʱ���Է������ض���Χ��(�ſ��8%~80%)�Ĺ����˶��������ڳ����������������صķ�������ʩ�ӣ����Ż�ֹͣ�رգ�
					//�����Զ�ִ�п��ţ��ſ�������Զ��رս��š�
					//�������й��Ź����м���������������Ƿ�ͣ�٣�1=��� (�����ͣ��һ��ʱ�����Ϊײ�����壬�ٿ���)��0=ʧЧ�������һֱ���������ȥ��
					//Ŀ�ģ���ֹ��ĻʧЧ������£����˻������塣�������ؼ����Ч��Χ�ڿ��ſ�ȵ�80%--8%�����ܵ���

#define		DOS0    25    		//25 ���Ż���λ��0(%)  OPEN POSITION 0
#define		DOS1    26     		//26 ���Ż���λ��1(%)  ------------- 1
#define		DOS2    27    		//27 ���Ż���λ��2(%)  ------------- 2
#define		DOS3    28			//28 ���Ż���λ��3(%)  ------------- 3
			   //2011-10 Johnson �ĸ����ٵ�ѿ��ŷֳ����

#define		DCS0    29			//29 ���Ż���λ��0(%)  CLOSE POSITION 0
#define		DCS1    30			//30 ���Ż���λ��1(%)  -------------- 1
#define		DCS2    31			//31 ���Ż���λ��2(%)  -------------- 2
#define		DCS3    32			//32 ���Ż���λ��3(%)  -------------- 3
			   //2011-10 Johnson �ĸ����ٵ�ѹ��ŷֳ����

#define		DOV0    33			//33 ���ŷֶ��ٶ�0(��ʼ�ٶ�)(Hz)    OPEN VELOCITY 0 (START SPEED)
#define		DOV1    34			//34 ���ŷֶ��ٶ�1(����)(Hz)        ------------- 1 (HIGH SPEED)
#define		DOV2    35			//35 ���ŷֶ��ٶ�2(�����ٶ�)(Hz)    ------------- 2 (END SPEED)

#define		DOV3    36			//36 ���ŷֶ��ٶ�3(�����ٶ�)(Hz)    ------------- 3 (PUSH SPEED)

#define		DCV0    37			//37 ���ŷֶ��ٶ�0(��ʼ�ٶ�)(Hz)    CLOSE VELOCITY 0 (START SPEED)
#define		DCV1    38			//38 ���ŷֶ��ٶ�1(����)(Hz)        -------------- 1 (HIGH SPEED)
#define		DCV2    39			//39 ���ŷֶ��ٶ�2(�����ٶ�)(Hz)    -------------- 2 (END SPEED)

#define		DCV3    40			//40 ���ŷֶ��ٶ�3(�����ٶ�)(Hz)    -------------- 3 (PUSH SPEED)

#define		EPOS    41			//41 λ�ü̵���λ���趨(%)          POSITION  RELAY
#define		EDOPT   42			//42 �����ƽ�ʱ��(S)                OPEN ?? PUSH TIME ָ�Źص�λ�󣬱�Ƶ�������Ƶ�г�������������г�������ʱ�䣬F42=100�������ƽ�
			   //   ����ָ�����ʹ�ܣ�����90�룬�Ź���󣬳������ӹ���ָ�����90������PWM�ƽ�����������Ϊ100�룬�������ӹ���ָ�������ʱ���ŵ����ƽ�

#define		EDCPT   43			//43 �����ƽ�ʱ��(S)                CLOSE?? PUSH TIME ָ�ſ���λ�󣬱�Ƶ�������Ƶ�г�������������г�������ʱ�䣬F43=100�������ƽ�
			   //2011-10 Johnson �о�ZDK������ѱ������ָ�ߵ���
			   //   ����ָ���ʹ�ܣ��ſ���󣬳������ӿ���ָ�����100�룬������ʱ���ŵ����ƽ�����������Ϊ99�룬�������ӿ���ָ�����99������PWM�ƽ���
			   //   �ֳ�һ�㶼��100.

#define		ESPT    44			//44 ��ѧϰ�ƽ�ʱ��(S)              STUDY  PUSH

#define		EDSCD   45			//45 ��������(��ʾģʽ)(��������)(����Ϊ��,ʵ��ֻ�е��ֽ�������) 0��1,2,3, Default 2  DISPLAY MODE
			   //0=��ʾ�趨Ƶ�ʣ�1=��ʾ���Ƶ�ʣ�2=��ʾλ�ðٷ�����3=��ʾλ��������

#define		EFG0    46			//46 ��ѧϰ�����ٶ�(Hz) �����ò����ɸı���ѧϰ�ٶ�            	STUDY SPEED
#define		EFG1    47			//47 ��ѧϰ�ƽ��ٶ�(Hz) �����ò����ɸı���ѧϰ�ٶ�                    STUDY PUSH SPEED

#define		EFG2    48			//48 ǿ�ȹ����ٶ�(Hz)   ���ƹ����ٶȣ������ò����ɸı����ƹ����ٶ�    NRDING SPEED
			   //   ǿ�ȹ�����ָ���ٹ��ţ� (GDV10�˲������Ѳ��ã�Ӳ���������ٹ�����ȥ��)

#define		EF1M0   49			//49 ��ѧϰ����Ƶ��(Hz) ��ֵԽ��ת��ԽС����֮��Ȼ                    STUDY TRQUE
#define		EPULSE  50			//50 ��ѧϰ��������(��)             TOTAL PULSE
#define 	XPULSE  50			//���F50 EPULSE(��ѧϰ��������)��Ӧ��EEPROM������ʼ��ַ, ����XSAV

#define		ECF1M   51			//51 ���Ż���ת��Ƶ��(Hz) ��ֵԽ��ת��ԽС����֮��Ȼ                  CLOSE BASE FREQ.

#define		ECTRQ   52			//52 ���ŵ�Ƶת�ز���(%)  ��ֵԽ��ת��Խ��    CLOSE TRQ BOOST
#define		EOPL    53			//53 �ſ���̵����ź�(%)  �����ò����ɸı��ſ����ź����λ��          OPEN  POS.
#define		ECLL    54			//54 �Ź���̵����ź�(%)  �����ò����ɸı��Ź����ź����λ��          CLOSE POS.
#define		ERST    155			//55 ��OU��OC���ϻָ�ģʽ(������ͣ��/��������Զ�����) ������1 ERST=0���̴������ģʽ ERST=1 ���̺Ͷ��Ӵ������ģʽ RESET MODE
				//   1)F55=1  �ڳ��ֹ���ʱ�������� ����ֹͣ �� ���ӿ��� ����ӹ���ʱ��ϵͳ��λ�Զ���ͷִ�У� ����һֱ��ѭ��
				//   2)F55=0  �ڳ��ֹ���ʱ��ֻ����������ֹͣ�� ����ϵͳ��λ������һֱѭ����
				//
				//F55=0��OU��OC���ϱ���װ�ô���ֹͣ״̬���������ϵ���ֹͣ�ź�ʱ�ɻָ�����״̬��
                //   F55=1��OU��OC���ϱ�����װ�ý��ⲿ�κ��źſ��Զ��ָ�����״̬��
#define		ESDY    56			//56 ��ѧϰ����ģʽ0,1,2 ��ѧϰ�������Ƿ񱣴浽F50 EEPROM��Ԫ ������0   AUTO-STUDY MODE
			   //                 0: ���浽F50  1:��ֵ<20�򱣴棬���򲻱��� 2:������
			   //F56����ѧϰ����ģʽ��F56=0װ�û���ÿ��ͨ���λ������ѧϰ�����������������һ����ѧϰ��������
                           //F56=1װ��ֻ����(F56=0)ʱ�����һ����ѧϰ���������Ժ�����ѧϰ�����ѧ�����������һ����ѧϰ��������15��������װ�û�����ѧϰ��
                           //ֱ�����������������ʱ��װ�òŻ��Զ�ת������ģʽ��F56=2װ�û�洢��ѧϰ�����������ϵ��ϵ�󲻻�����ѧϰ�����մ�����������������
                           //��ѧϰʱF56Ӧ�趨Ϊ"0"�������������趨Ϊ"0"��"2"�������趨Ϊ"1"��"1"���ұ���������

#define		EJDLY   57			//57 ǿ�ȹ�����ʱ���ź�ʱ��(S)           RELAY DELAY
			   //   ǿ�ȹ�����ָ���ٹ��ţ����ſ��������£����ӽӵ����ٹ���ָ���ʱ2�����ִ�й��Ŷ�����(GDV10�˲������Ѳ��ã�Ӳ���������ٹ�����ȥ��)

#define		EAPPL   58			//58 ��������(�ɵ�Ƶ��ģʽ)(��������) [0,1] 1->�ϵ���ʾ���������Ŀ,0->�ϵ���ʾ���Ƶ��ֵ   NORMAL/DOOR CON.
			   //   2011-10 Johnson ����������Ϳ��������⣬������ 1->��ʾ��ǰ���б����������� 0->��ʾ��ǰ���е��Ƶ��ֵ

/********���±�����ZDK�������ӵģ���Щ�Ǿ��������û�Ҫ���***************/

#define		ESSK    59			//59  PULSE    ������û����
#define		EF_HD   60			//60  �������� ������û����

				//178H	   ;F60:  ���Ź����ж��룬�ſ���λ��ǿ�з������ſ��롱�ź���ʱʱ�䣬���ʱ����F60��Ԫ������������F60=10Ϊ�ſ���λ10��󷢣�
                //         ;F60=20  Ϊ�ſ���λ20��󷢣��Դ����ƣ��趨F60=100ʱ��Ϊ������ʱ����
                //17AH     ;F61   �����ź��Ƿ���Ч F61=1�����ź���Ч������ʱ�����������źŷ��ɿ��ţ�F61=0����ʱ���������źţ����ɿ��š�
                //17CH     ;F62
                //17EH     ;F63   ��ʾ��ǰVDC�Ĳ����� (��������FUNM=63ʱ��F63��Ч)
				//����F59--F63�ǵ���HADΪ�������µ�˹��Ҫ�����ӵ�

/********PWD�Ǳ�����EEPROM����Ȩ�������*********************************/

#define		EPWD	100			//EEPROM����Ȩ�����ַ
#define 	XPWD	100      	//EEPROM����Ȩ����,ÿ�ο�����Ҫ����Ӧ��EEPROM����Ԫ����Ȩ��,���û��Ҫ��д���,����ʼ��û��LED��ʾ��
			   					//������Ȩ�������ݿ�PASS��������

/********���±����ĵ�ַ�����ǳ���ʵ�����м��������**********************/

#define		RDCCT	104			//ֱ���ƶ�ʱ��ο�               DC-BRAKE TIME REFERENCE
#define		RDOPT	105			//�����ƽ�ʱ��ο�               DOOR OPEN ??  TIME REF.

#define		RDCPT	106			//�����ƽ�ʱ��ο� �������δ��  DOOR CLOSE??  TIME REF. Ҳû��������ϵ�RAM ��������DCPT
				//2011-10 Johnson �о�ZDK������������������ֵߵ�

#define		RSPT	107		   	//��ѧϰʱ��ο�                   STUDY TIME REF.
#define		RJDLY	108			//ǿ�ȹ��ż̵�����ʱ���ź�ʱ��ο� RELAY DELAY REF.

/********���±����Ǹ���EEPROM�������������������������*****************/

#define		DOSA	112	   		//2011-10 Johnson DOSA=DOS1-DOS0   S���� ����ƽ������     SQUARE CHARACTICS OF DOOR OP
#define		DOSB	113			//2011-10 Johnson DOSB=DOS3-DOS2
#define		DOVA	114			//2011-10 Johnson DOVA=2*(DOV1-DOV0)
#define		DOVB	115			//2011-10 Johnson DOVB=2*(DOV1-DOV2)

#define		DOS01	116			//2011-10 Johnson Ӧ���ǿ��Ż���λ��01�������ڵ�0������λ�ú͵�1������λ��֮��ĵ� (%) ȷ��S������ȷ��
#define		DOS23	117			//2011-10 Johnson Ӧ���ǿ��Ż���λ��23�������ڵ�2������λ�ú͵�3������λ��֮��ĵ� (%) ȷ��S������ȷ��

#define		DCS01	118			//2011-10 Johnson Ӧ���ǹ��Ż���λ��01�������ڵ�0������λ�ú͵�1������λ��֮��ĵ� (%) ȷ��S������ȷ��
#define		DCS23	119			//2011-10 Johnson Ӧ���ǹ��Ż���λ��01�������ڵ�2������λ�ú͵�3������λ��֮��ĵ� (%) ȷ��S������ȷ��

#define		DCSA	120			//2011-10 Johnson DCSA=DCS1-DCS0  S���� ����ƽ������    SQUARE CHARACTICS OF DOOR CL
#define		DCSB	121			//2011-10 Johnson DSCB=DCS3-DCS2
#define		DCVA	122			//2011-10 Johnson DCVA=2*(DCV1-DCV0)
#define		DCVB	124			//2011-10 Johnson DCVB=2*(DCV1-DCV2)
			   //87C196MC 488byte RAM����ߵ�ַ��1FFH, �ʶ�û�г�Խ87C196MC RAM����


char ERROR[] = 	{0xFF, 0x49, 0x63, 0xFF,	      			//SC   0      (����ָʾ�����)
         		0xFF, 0xC5, 0xE5, 0xFF,	        			//oc   4
	 			0xFF, 0xE3, 0x83, 0x25,	        			//LU2  8
         		0xFF, 0xC5, 0xC7, 0xFF,	       				//ou   12
	 			0xFF, 0x61, 0xF5, 0xF5};	        		//Err  16

/*	  //	0   1	2   3	4   5	6   7
ADISP:	 DCB   03H,9FH,25H,0DH,99H,49H,41H,1FH	;0-7
	 DCB   01H,09H,11H,0C1H,63H,85H,61H,71H ;8-F

	  ; 14*16=224�ֽ�(0-222)
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
   
   	
   	

	MST = 0x05;         						//DIR=MST.0=1 ����  #0000 0101B; 
	SAMP_T = 0;									//��SAMP_T��־
	COMD = 0xFF;								//��COMDΪ0FFH
	SD_FLG = 0;                					//����ѧϰ��־  2011-10 Johnson ��Ϊ��ѧϰ��ʼ״̬ SET SELF_STUDY
	DOCPT = 0;									//��DOCPT��־
	OBPT = 0;									//��OBPT��־
	JDLY = 0;									//��JDLY��־
	FSSK = NMAX;            					//#NMAX����C196.inc����30000  ASMUUE A MAX NUMBER WHEN ON
	NUMBER = FSSK;								//�ѵ�ǰ��õı�������������Ϊ30000
	//TIMER1[0] = NUMBER;						//��DSP����ô��ʾ��װ�������������ȥ��
	S = NMAX >> 1;              				//2011-10 Johnson�о�S���岻��ȷ����ʱ�����50%�ſ�������������ÿ�����λ��Ϊ50%
												//SET INICAL DOOR POSTION AT 50.0%
												//�ڳ���֮��S ������ǰ���ſ�λ�õİٷֱ�(�ǷŴ�1000��)
	
	FORMAT();                					//��ʼ���ӳ���
	INE2M();                 					//��ʵF20������ֵ��������״����У�����Ҫ�״�������Ҫ�ѹ��������д���EEPROM�ӳ���
	BEGIN();                 					//������ʼ����
	
	//------- �ϵ��ѹ������2003-5-13��--------
 	/*VADCH();   
	while(VDC <= 384) {
		VADCH();
	}*/
	
	AX1 = VDC - 384;							//384=512*75% ʵ��512��Ӧ����V? Ϊʲôһ��Ҫ����384���ܹ���? ����˵220Vac(310DC) ����VDC=2.5V����ô170Vac����1.93
                                        		//2011-10 Johnson ���ݹ�ʽ10bit AD: 384��ӦVDC=1.877V,Ӧ����180V��������
                                        		//�˰´�˵����֧�ֽ������뷶Χ��180V - 265V
												//ʵ�ʲ���HAD�İ���VDC=1.877Vʱ������174VAC��GDV10����173VAC
	
												//�߼���֮ǰINT_MASKΪ00H,�����ζ����TOVF(Timer1 Or Timer2 ����жϹ���)
	  											//֮ǰPI_MASKΪ00H
	   											//OPEN TF2
												//�߼���֮ǰINT_MASK1Ϊ00H,�����ζ����Extint�ⲿ�ж�
	 											//��ָ�����ж�
	 											
	//***  03H,9FH,25H,0DH,99H,49H,41H,1FH  ;0-7
    //***  01H,09H,11H,0C1H,63H,85H,61H,71H ;8-F

	DBUF[0] = 0xC1;           					//b(#0C1H)׼����ʾ������Ϣ������汾����: bb5.0
 	DBUF[1] = 0xC1;           					//b(#0C1H)
	DBUF[2] = 0x48;            					//5.(#48H)
	DBUF[3] = 0x03;           					//0(#03H)
	DBUF[4] = 0xFF;           					//�ر�����LEDָʾ
	BX1 = 1000;
	while(BX1 --) {
												//��ʾ�汾�������Լ��ʱ�� 100xDelay TIME 100x2.4ms=2.4��
		DELAY();
	   	DISPLY();								//���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���
	}
	
	//----------  switch on pre-charger relay ----------
	//2011-10 P2.5 OUT1�����̵������� (�����̵��� ��Ч)

	//�߼���֮ǰINT_MASKΪ00H,�����ζ����TOVF(Timer1 Or Timer2 ����жϹ���)
	//2011-10 Johnson ��κ�START2���ظ�����
	//�߼���֮ǰINT_MASK1Ϊ00H,�����ζ����Extint�ⲿ�ж�
	//�����г���������Ҫ���ж�,׼�����볣����ѭ������
	
	
	
	//************************************************
	//*	      ���澲̬��ѭ������	        *
	//************************************************
	/*GETPOS();											//����λ�������Ӻ���,λ�ô���NUMBER��
	VADCH();											//���ֱ��ĸ�ߵ�ѹ
	AUTO();												//�ж�AUTO���Ƿ��£�Ӱ��ECMD[0]
	
	//P2_REG[0] |= #00010000B;							//����P2.4(THR)Ϊ��,��ʾZDK��PWM�����245�رգ�
														//�������THR���Ϊ��ģ�ʵ��û��THR, ��ζ�����BB50û������
	DSCD = E2M[EDSCD] ;									//��ʾģʽF45? Default 2 0=��ʾ�趨Ƶ�ʣ�1=��ʾ���Ƶ�ʣ�2=��ʾλ�ðٷ�����3=��ʾλ��������
	//INT_MASK.bit0 = 1;									//OPEN TIMOV (Timer1 & 2 ����ж�)
	
	FOM = E2M[EFOM]										//��װ��,��EEPROM RAM F02��Ӧ�����Ƶ����(Hz �Ŵ�100��)��ŵ�FOM��
	FLO = E2M[EFLO]										//��װ��,��EEPROM RAM F01��Ӧ�����ʼ��Ƶ����(Hz �Ŵ�100��)��ŵ�FLO��
	FG = E2M[EFG]											//��װ��,��EEPROM RAM F00��Ӧ���趨(����)Ƶ����(Hz �Ŵ�100��)��ŵ�FG��
	
	S = NUMBER * 1000 / E2M[EPULSE];						//����S Ϊ��ǰ���ŵ�%λ�� (%���ǷŴ�1000��) 30000% ?  EPULSE[0]ΪF50 ��ѧϰ��������(������1000)	
														//����ʱ����EPULSE=1000,����NUMBER * 1000 /1000 =NUMBER Ӧ���ǵ�һ�μӵ� AX1����30000�Ŷ�
														//�ѵ�ǰ��õı����������� * 1000 / ��������ѧϰ�������� Ӧ���Ǽ��㵱ǰ��õ��ſ�λ�ðٷֱȲ��Ŵ�1000��
														//�п��ܻ��½�� ( S ) >100% (1000), ֻ��ͨ����ѧϰ��ſ��ܵ���<=1000 (100%)
										
	KEYB();												//����ǰ��Ч��ֵ,���ڲ���KEY,RT, ����ǰ����״̬�ӳ�
	AUTODY();											//�����Զ���ʾ�ӳ���
	
	if(KEY & 0xBF == 0) {								//�ǹ��ܼ�?(���ܼ�Ϊ#10111111B),�״��ϵ�KEY=#0FFH 
		//DELAY2(200);									//�ǹ��ܼ� ����ʱ200��������λʱ�� 200x2.4ms=480ms
		GETS();											//�ѵ�ǰ�������Ӧ�����ò���������Ӧ��RAM��Ԫ��
		CONTROL.bit0 ^= 0x01;							//CONTROL���#0000 0001B ��������ʾ�Ͳ���ʾ������֣������Ϊ����ʾ��˸
	} //end KEY.bit6 == 0
	
	if(CONTROL & 0x01 == 0) {							//CONTROL.0=0 ������ʾ��ǰ���������
		PARS();											//���ҵ��������Ӧ���ò��������ӳ���
		
		if(KEY & 0x20 == 0 || KEY & 0x10 == 0 || FRSH & 0x10 == 0){			//�����UP��DOWN����תDPAR�������(UP��=#11011111B,DOWN��=#11101111B) 
																		//FRSH����˼10000B x 16ms = 256ms
			DISPAR();									//���ò�����ʾ(����λ)�ӳ���
		} else {
			BLACK();									//����������ʾ����������� ֻ�ǰ�DBUF1,2,3,4,5��Ϊ0FFH�����´�DISPLAY����ʾ��������˸
		}  //end KEY.bit5 == 0 || KEY.bit4 == 0 || FRSH.bit4 == 0
		
		WRS();											//��ǰ���������ò���д���ӦEEPROM �����뵥Ԫ�ӳ���
	} else {							
		FUNS();											//CONTROL.0=1 ������ʾ��ǰ������ ���ù�����ɸѡ�������
		DISFUN();										//�����������ʾ����ɸѡ��Ĺ������ӳ���
	}  //end CONTROL.bit0 == 0
	
	DISPLY();											//���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���
	
	if(KEY & 0x04 == 0 || (KEY & 0x08 == 0 && KEY & 0x04 == 0)){		//��ǿ�ȹ��ż��������λ��? (ǿ�ȹ��ż�=#11111011B �����=#11110011B),��תREADY�������
																		//�������λ��?(�����=#11110011B)?,��תREADY [GDV10�岻֧��ǿ�ȹ��ż�]
		LJMP	  START	             ;�������λ�����ͷ��ʼִ��     RESET AUTO-STUDY    �ⲿ����C������ô�����þ��忼��   ?????????????????
	} else {
		APPL = EAPPL[0];	        					//��RAM�ж�ӦEPPROM��F58�Ŀɵ�Ƶ��ģʽ ����װ��APPL�� 1->�ϵ���ʾ���������Ŀ,0->�ϵ���ʾ���Ƶ��ֵ
		
		if(APPL & 0x01 == 1) {							//���APPL.0�ǲ���1?(1->�ϵ���ʾ���������,0->�ϵ���ʾ���Ƶ��ֵ),�������תREADY1����
			COMMAND();									//(ֻ�����ϵ����������ʾģʽ)������ӳ�
			
			if(SD_FLG != 2) {							//��ѧϰ�Ƿ����?  sd_flg=2 means self-study finished
				P2_REG[0] &= #00111111B;				//��OUT2��OUT3�̵����ر� (�ſ��� �Ź���ʧЧ)
				WG_OUT[0] &= #1111111101111111B;  		//��OUT4Ҳ�ر� (ָ��λ�� ʧЧ)
				
				if(SD_FLG == 0) {						//����ѧϰ��ʼ״̬��? ����ǣ���ʼSFIRST(��ѧϰ��һ�׶�)   ��ѧϰ��һ�׶�Ϊ���Ų���
														//SD_FLG=0 	����ǰ��Ҫ������ѧϰ��һ�׶Σ�����ֱ���Զ�����
					if(COMD & 0x08 == 0 || COMD & 0x04 == 0) {			//COMD.3=0 (��������)? ��תSFIST1 self study first course begins  only by "close" pressed
																	//COMD.2=0 (��������)? ��װSFIST1 self study begins (open)   2004-03-10
						DELAY2(10);						//��ѧϰ��һ�׶ο�ʼ����ʱ2.4x10=24����
						MST |= 0x01;					//��MST.0=1 ���� f/r dir  close
					} else {
						DELAY2(1);						//��ʱһ��������λʱ��2.4ms
						LJMP	  STAY			;���¿�ʼ���澲̬��ѭ������
					}
				} else {								//��ѧϰ�ڶ��׶�Ϊ���Ų���
					DELAY2(10);							//��ѧϰ�ڶ��׶ο�ʼ����ʱ2.4x10=24����
					MST &= 0xFE;						//MST.1=0 ���� direction open
				}  //end SD_FLG == 0
			} else {
				if(COMD & 0x02 == 0) {		        	//2011-10 (COMD.1=0����ǿ�ȹ�������) �����ǿ�ȹ������� ��תRCLOSE1����
					MST |= 0x01;						//��MST.0=1 ���� f/r dir  close
				} else if(COMD & 0x04 == 0) {         	//(COMD.2=0������ָ��) ����ǿ������� ��תROPEN1����
					MST &= 0xFE;						//MST.1=0 ���� direction open	
				} else if(COMD & 0x08 == 0) {        	//(COMD.3=0�����������) ����ǹ������� ��תRCLOSE1����
					MST |= 0x01;						//��MST.0=1 ���� f/r dir  close
				} else {
					J123();                  			//���ݵ�ǰ��S (�ſ�λ�ðٷֱ�) ���ü̵�������ӳ���
					DELAY2();							//��ʱһ��������λʱ��2.4ms
					EXCOM3:    LJMP	  STAY			;���¿�ʼ���澲̬��ѭ������
				}  //end COMD.bit1 == 0
			}  //end SD_FLG != 2
		} else {
			if(KEY & 0x01 == 1) {  						//���ϵ���ʾ���Ƶ��ģʽ�£�������ǿ��ż�(���ż�=#11111110B),��תEXCOM3 ���¿�ʼ���澲̬��ѭ��
				EXCOM3:    LJMP	  STAY		;���¿�ʼ���澲̬��ѭ������
			} else {
				MST &= 0xFE;							//MST.1=0 ���� direction open
			}  //end KEY.bit0 == 1
		}  //end APPL.bit0 == 1
		
		
		//------------------��������(����ѭ����ʽ)-----------------------------------
		//��������� ͨ��MST�����ֵ�ǰ�ǿ��Ż��ǹ��ŷ���
		
		INICIAL();										//�������г�ʼ��INICIAL�ӳ���
		ADCL0();	                					//STUDY SPEED Ƶ�������Ԥ����--��ѧϰ����Ӽ��ٴ����ӳ���
														//���INCFOL,INCFOH; DECFOL,DECFOH
														//��ѧϰ�׶β���DECFOL,DECFOH
		
		DELAY2(10);					               		//��ʱ2.4 * 10 = 24ms
	   
		P2_REG[0] &= #11101111B;						//��ʱ���൱��THR�ͣ���245�������׼�����PWM��
		
		DELAY();										//��ʱ24ms
		OPWAVE();										//����PWM����CPU����ӳ���
		DELAY();										//��ʱ24ms
		OPSPWM();										//����SPWM���η����ӳ���
		DSCD = EDSCD[0];								//��ʾģʽF45? Default 2 0=��ʾ�趨Ƶ�ʣ�1=��ʾ���Ƶ�ʣ�2=��ʾλ�ðٷ�����3=��ʾλ��������
		RT |= 1;										//RT.0=1 ֹͣ?
		GETPOS();										//���õ�ǰ�ſ��õ�ǰ��λ���������ӳ���
		VADCH();										//����ֱ��ĸ�ߵ�ѹ����ӳ���--VDC
        AUTO();											//����AUTO�ӳ����ж�AUTO�Ƿ���? ���ֶ�ģʽ(���̿���) �����Զ�ģʽ?
		
        if(APPL & 0x01 != 0 && SD_FLG != 0) {			//0->�ϵ���ʾ���Ƶ��ֵ ģʽ��תRUNDIR
														//1->�ϵ���ʾ���Ƶ��ֵ ģʽ�����������
			S = NUMBER * 1000 / EPULSE[0]; 				//����AX1 = NUMBER * 1000 (�Ŵ�1000��) AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
														//AX1 = AX1 / ��ѧϰ��������  (AX1 = ��������   BX1=����������)
														//S Ӧ���ǵ�ǰ��λ�õ�%�� (%���ǷŴ�1000����)
			J123();										//���ü̵�������ӳ���
		}  //end APPL.bit0 != 0 && SD_FLG != #0
		
		if(MST & 0x01 == 1) {							//MST.0=1? ���ŷ��� ����DREV
			DBUF[5] &= 0xBF;
			DBUF[5] |= 0x80;							//׼������"����LED"
		} else {
			DBUF[5] &= 0x7F;
			DBUF[5] |= 0x40;							//׼������"����LED"
		}  //end MST.bit0 == 1
		
		KEYB();											//����ǰ����״̬�ӳ���
		AUTODY();										//�����Զ���ʾ�ӳ���
		
		if(KEY & 0x40 == 0) {								//KEY.6=0? �ǹ��ܼ�
			DELAY2(200);                				//��ʱ200 * 2.4ms=480ms
			CONTROL ^= 0x01;						//���CONTROL��#00000001B ??

			GETS();										//�ѵ�ǰ�������Ӧ�����ò���������Ӧ��RAM��Ԫ��	
		}  //end  KEY.bit6 == 0
		
		if(CONTROL.bit2 == 0) {							//CONTROL.2=0? ����תDIS_MD		2011.11.22 liuhao ��ʾ���ǲ��������ǹ�����
			if(CONTROL.bit0 == 0) { 					//CONTROl.0=0? ����תDIS_MD1   	2011.11.22 liuhao ��ʾ����
				PARS();		     						//���ҵ��������Ӧ���ò��������ӳ���
				DISPAR();								//���ò�����ʾ(����λ)�ӳ���
					
				if(FUN == #11) {						//��F11��? F11 �ǿ���ģʽ COMD ��0->���̿��� 1-> �㶯  2->�ⲿ����?
					WRS();								//F11 ����ģʽ���������ò���д���ӦEEPROM �����뵥Ԫ�ӳ���
				}
			} else {
				FUNS();									//���ù�����ɸѡ�������
				DISFUN();								//������ʾ����ɸѡ��Ĺ������ӳ���
			}  //end CONTROL.bit0 == 0
		} else {
			JTDC();                  					//������ʾ "BCD-"
		}  //end CONTROL.bit2 == 0
		
		DISPLY();										//���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���

		if(APPL.bit0 == 1) {							//1->�ϵ���ʾ���������Ŀ ����װADCL_SP �ɵ�Ƶ��ģʽF58
														//0->�ϵ���ʾ���Ƶ����Ŀ ������������
			if(SD_FLG != 2 || (SD_FLG == 2 && RT.bit1 == 0)) {
				if(SD_FLG == 2 && RT.bit1 == 0) {
					OBSTCL();							//���������ؼ���ӳ���
				}  //end SD_FLG == 2 && RT.bit1 == 0
				
				COMMAND();								//(ֻ�����ϵ����������ʾģʽ)������ӳ���
				
				if(S > #0) {							//�Ƚ�S �� 0%
					if(COMD != #0FFH) {					//COMD=#0FFH? �п�����������
						RT &= #01111111B;	        	//ֻҪ�п�������(GDV10ֻ�п��ţ�������������ָ��)����RT.7=0  CLR AUTO ClOSE DOOR MODE
					}  //end COMD == #0FFH
				} else {
					RT &= #01111111B;	        		//ֻҪ�п�������(GDV10ֻ�п��ţ�������������ָ��)����RT.7=0  CLR AUTO ClOSE DOOR MODE
				} //end S > #0
				
				if(RT.bit0 == 0) {		    			//RT.0=0? ��תEMG1
					 SJMP	  EMG1 ;	 ??????????		
				} else {
					if(COMD.bit0 == 0) {				//��ֹͣ����? ��תEMG0
						if(SD_FLG == #2) {				//�˲���ѧϰ�Ƿ����?
							FG = #LSP;					//FG = LSP(�������СƵ��HZ�ĸ�16bit)
							ADCL1();					//Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
							
							if(FOH > #LSP) {			//FOH= LSP(�������СƵ��HZ�ĸ�16bit)
								LJMP	  RUNL5			;��תRUNL5  ��ô�������Ű�ɣ�����������������
							}  //end FOH > #LSP
						} else {
							SD_FLG = 0;					//�����ѧϰ��û�н�������ǿ������Ϊ��ѧϰ��ʼ��
						}  //end SD_FLG == #2
						
						 SJMP	  EMG1       ;       ?????????????
					} else {
						if(SD_FLG == #2) {		//�˲���ѧϰ�Ƿ����?
							if(RT.bit7 == 1) { 		//RT.7=1? ��AUTO_DC(�Զ�����?)
								if(MST.bit0 ==1) {		//MST.0=1? ���ŷ��� ��תDC_2
									ADCL2();			//Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
									CLOSE();			//���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���	
								} else {
									ADCL1();			//Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
									FG = #LSP;			//FG = LSP (������СƵ�ʸ�16bit HZ)
									
									if(FOH <= #LSP) {		//FOH= LSP (������СƵ�ʸ�16bit HZ)
										MST |= #00000001B	//MST.0=1 ���ŷ���
									}
								}
								DC_1:	   SJMP	  RUNL5       ;���Ű�????
							} else {
								if(COMD.bit1 == 1) { DO_0		//COMD.1=1? ����ǿ�ȹ�����������DO_0
									if(COMD.bit2 == 1) { DC_0		//COMD.2=1? ���� �������� ��תDC_0
										if(COMD.bit3 == 1) { STOP_0		//COMD.3=1?�ǹ������� ��תSTOP_0
											if(S <= #50) {			//�Ƚ�S�� 5%  (%���ǷŴ�1000����)
												if(MST.bit0 == 0) { STOP		//MST.0=0? ���ŷ��� ��תSTOP
													  JH	  STOP			;DOCPT > RDOPT,��תSTOP???????
												
													AL1 = ESTT[0];		//AL1 = ESTT (F12 ֹͣģʽ0:����ֹͣ 1:����ֹͣ)
													if(AL1.bit0 == 1) { STOP_A		//���AL1.0=1 �Ǽ���ֹͣ ��תSTOP_A
														ADCL1();			//Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
														DC_BK();			//ֱ���ƶ������ӳ���
													} else {
														RT &= #11111110B;		//����ֹͣ���� RT.0=1	
													}  //end AL1.bit0 == 1
													SJMP	  FGC00     ??????????????????????
												} else {
													RT |= #01000000B;         //RT.6=1
													
													if(DOCPT > RDOPT[0]) {	 //DOCPT ��RDOPT(�����ƽ�ʱ��ο�)�Ƚ�
														JH	  STOP			;DOCPT > RDOPT,��תSTOP???????
													} else {
														FG = DCV3[0];        //FG = DCV3 (���Ż���λ��#3��Ӧ���ٶ� F40 Hz) DC PUSH SPEED
														SJMP	  FGC00  ?????????
													}
												}
											} else {
												if(S > #95) {			//�Ƚ�S��9.5% (%���ǷŴ�1000����)
													if(MST.bit0 == 0) {STOP_4		//MST.0=0? ���ŷ��� ��תSTOP_4
														RT |= #00100000B		//RT.5=1 ����?
														
														if(DOCPT > RDOPT[0]) {	    	//DOCPT ��RDOPT(�����ƽ�ʱ��ο�)�Ƚ�
															 SJMP	  FGC00   ?????????
														} else {
															FG = DOV3[0];            //FG = DOV3 (���Ż���λ��#3��Ӧ���ٶ� F36 Hz) DO PUSH SPEED
															SJMP	  FGC00      ??????????
														}
													} else {
														SJMP	  STOP
													}
												} else {
													RT &= #10011111B         		//S <= 9.5% ����RT,5,6=0
													SJMP	  STOP
												}
											}
										} else {  //�ǹ���������
											if(MST.bit0 == 1) { DC_2		//MST.0=1? ���ŷ��� ��תDC_2
												ADCL2();			//Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
												CLOSE();			//���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���
											} else {
												ADCL1();			//Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
												FG = #LSP;			//FG = LSP (������СƵ�ʸ�16bit HZ)
												if(FOH <= #LSP) {		//FOH= LSP (������СƵ�ʸ�16bit HZ)
													MST |= #00000001B;	//MST.0=1 ���ŷ���
												}
											}
											DC_1:	   SJMP	  RUNL5           ?????????????
										}
									} else {
											;����� �������� ��������洦��
														;****�Զ������Ŵ���**************
										if(MST.bit0 == 0) { DO_2		//MST.0=0? �ǿ��� ��תDO_2
											ADCL2();			//Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
											OPEN();			 	//���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���
											SJMP	  DO_1
										} else {
											ADCL1();			//���� Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
											FG = #LSP;			//FG = LSP (������СƵ�ʸ�16bit HZ)
											
											if(FOH >= #LSP) {		//FOH��LSP�Ƚ�(������СƵ�ʸ�16bit HZ)
												MST &= #11111110B;	//MST.0=0 ���ŷ���
											}
										}												
										 SJMP	  RUNL5 ??????????????
									}  //end COMD.bit2 == 1
								} else {
									if(MST.bit0 == 1) {ND_2		//��ǿ�ȹ�������, MST.0=1?(�ǹ��ŷ���?)�ǹ��� ����ND_2(GDV10û��ǿ�ȹ���)���۲���ִ�У�ʵ�ʷ�����ִ�У����
										ADCL2();			//Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
										CLOSE();			//���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���
										if(FG <= EFG2[0]) {            //�Ƚ�FG �� EFG2 (F48 ǿ�ȹ����ٶ�Hz ����12Hz)? NRDING SPEED
											ADCL2();			//Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
										} else {
											ADCL1();			//���FG > EFG2 Ƶ�������Ԥ����#1�ӳ���
											FG = EFG2[0];		//FG = EFG2 (F48 ǿ�ȹ����ٶ�Hz
										}
										ND_1:	   SJMP	  DC_1 ?????????
									} else {
										ADCL1();				//������ Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
										FG = #LSP;				//FG = LSP(������СƵ�ʸ�16bit HZ)
										if(FOH <= #LSP) {		//FOH��LSP�Ƚ�(������СƵ�ʸ�16bit HZ)
											MST |= #00000001B;	//MST.0=1  ���ŷ���
										}
									}  //end MST.bit0 == 1
									DC_1:	   SJMP	  RUNL5       ???????????????
								}  //end COMD.bit1 == 1
							}  //end RT.bit7 == 1
						} else {
							STUDY();					//�������ſ���ѧϰ�����ӳ���
							LJMP	  FGC00
						}  //end SD_FLG == #2
					}  //end COMD.bit0 == 0
				}  //end RT.bit0 == 0
			} else {
				if(RT.bit1 == 1) { XRUNL3		//RT.1=1? ���ؼ�ر�־λ��Ч ����תXRUNL3
					RT &= #01111111B;			//RT.7=0
		
					if(S <= #960) {		;�Ƚ�S ��960 ���ǱȽ�S �� 96% (%���ǷŴ�1000����)
						if(OBPT > #300) {		//OBPT <300? ���ؼ���ƽ�ʱ�� 
												//��תAUTO_OC (�Զ�������?)
							RT &= #11111101B;			//RT.1=0 ���ؼ�ر�־λ��Ч
							OBPT = #0000H;				//OBPT=#0000H (���ؼ���ƽ�ʱ��)
						} 
						
					} else {
						RT &= #11111101B;			//RT.1=0 ���ؼ�ر�־λ��Ч
						OBPT = #0000H;				//OBPT=#0000H (���ؼ���ƽ�ʱ��)
					}	
				
					if(MST.bit0 == 0) {DO_2		//MST.0=0? �ǿ��� ��תDO_2
						ADCL2();				//Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
						OPEN();					//���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���
					} else {
						ADCL1();				//���� Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
						FG = #LSP;				//FG = LSP (������СƵ�ʸ�16bit HZ)
						
						if(FOH <= #LSP) {		//FOH��LSP�Ƚ�(������СƵ�ʸ�16bit HZ)
							MST &= #11111110B;	//MST.0=0 ���ŷ���
						}
					}
							
					SJMP	  RUNL5   ????????????????
				}	// end RT.bit1 == 1   �ⲿ���ж��������д��ı䣿��������������
			} //end SD_FLG != 2 || (SD_FLG == 2 && RT.bit1 == 0)
		} else {
			if(RT.bit0 == 1) {		            		//RT.0=1 ����תNORR1
				BOOST();								//���õ�Ƶ���ز����ӳ��� -->VTRQ
				VANDF();								//����V/F���Դ����ӳ���  -->KV1
				FGC = EFG[0];							//�ѵ�ǰϵͳ���趨(����)����Ƶ��(�Ŵ�100�� )����FGC
				LJMP	  RUNL			;��תRUNL  Ҫ������������ʲô�취����ȥ����������������������������
			} else {
				if(SD_FLG == #2) {						//�˲���ѧϰ�Ƿ����?
					FG = #LSP;							//FG = LSP(�������СƵ��HZ�ĸ�16bit)
					ADCL1();							//Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
					if(FOH > #LSP) {					//FOH= LSP(�������СƵ��HZ�ĸ�16bit)
						 LJMP	  RUNL5			;��תRUNL5  ��ô�������Ű�ɣ�����������������
					}  //end FOH > #LSP		
				} else {
					SD_FLG = 0;							//�����ѧϰ��û�н�������ǿ������Ϊ��ѧϰ��ʼ�׶�
				}  //end SD_FLG == #2
					
				FUN	= 0;								//����������
				RT &= #00001110B;						//RT.1=RT.2=RT.3=1
				CONTROL = #00H;							//CONTROL=#00H
				STWAVE();								//��ֹPWM����CPU����ӳ���
				STSPWM();								//ֹͣSPWM���η����ӳ���
				COMD = #0FFH;							//COMD=#0FFH ������е�ǰ�Ŀ�������
				DELAY();                 				//��ʱ24ms
				LJMP	  STAY0		;ֹͣ������ �ٴν�����Χ���澲̬��ѭ������!???????????????????????
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
	KEY = 0xFF;									//�ѵ�ǰ���̶�ȡ��ֵ��Ϊ0FFH
	TERMAL = 0xFF;								//�ѵ�ǰTERMAL����0FFH,���TERMAL���������Ķ��������ź�״̬��־ ����λ��Ч
	CONTROL	= 0;								//CONTROL=#00H ���Ʊ�־�Ĵ��� ����Ч
	DBUF[4] = 0xFF;	        					//��ָʾLEDȫ���������ʵ��û�б�Ҫ����ΪOPENΪ�ߣ�����ܰ�û�е磩
}


/*****************************************************************
*           ��ʱ�ӳ���
*��׼DELAYΪ10��������ʱ��16MHZ�»�����ʱDELAY2Ϊ����С״̬���ں�30x125nsx666=2.4ms
*IN  PARAMETER: NONE /DLY                DELAYΪ2.4x10=24ms
*MID PARAMETER: DLYS
*OUT PARAMETER: NONE
*16MHZ ״̬������125ns
*12MHZ ״̬������167ns
*8MHZ  ״̬������250ns
******************************************************************/
//�ⲿ����Ҫ��ϸ����DLEAY��ʱ������
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
DELAY2:   LD   DLYS,#666                  ;4-10״̬ 1S��ʱ
DELAY3:   DEC  DLYS                       ;3״̬
	  CMP  DLYS,#0                    ;4-9״̬
	  JNE  DELAY3                     ;4״̬
	  DEC  DLY                        ;3״̬
	  CMP  DLY,#0                     ;4-9״̬
	  JNE  DELAY2                     ;8״̬
	  RET*/


/******************************************************************
*            Translation����ʾ����ת���ӳ��� (16����-10����)
*IN  PARAMETER: AX1(��Ҫת����16bit����)
*MID PARAMETER: BX1
*OUT PARAMETER: CL1,CH1,DL1,DH1,EL1
*******************************************************************/
/*void TRANS(Uint16 AX1)
{     
	Uint16 BX1;
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	CL1 = ADISP[BX1];          				//��ʾAX1�ĵ�һλ
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	CH1 = ADISP[BX1];	  					//��ʾAX1�ĵڶ�λ
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	DL1 = ADISP[BX1];						//��ʾAX1�ĵ���λ
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	DH1 = ADISP[BX1];						//��ʾAX1�ĵ���λ
	BX1 = AX1 % 10;
	AX1 = AX1 / 10;
	EL1 = ADISP[BX1];						//��ʾAX1�ĵ���λ
			//0   1	2   3	4   5	6   7
//ADISP:	 DCB   03H,9FH,25H,0DH,99H,49H,41H,1FH	;0-7
//	 DCB   01H,09H,11H,0C1H,63H,85H,61H,71H ;8-F
}*/


/************************************************************
*           FUNCTION PROCESSION ������ɸѡ�����ӳ���
*IN  PARAMETER:  KEY(7EH)
*MID PARAMETER: FUN(5CH)
*OUT PARAMETER: FUN(5CH)
*F20=8080��ʱ��FUNCTION������ȫ����ʾ������Լ���ĵ�����Χ(�����û�����)
*�����������еı���������ʾ,FUN 01- 02, FUN 15-17, FUN 13-0F, FUN 0B-0D, FUN 38-3A ������ʾ
*************************************************************/
/*void FUNS() {
    if(ESAV[0] == #8080) {              			//���F20�Ƿ���8080,������ʾ����������
		if(KEY.5 == 0) {							//����UP�� ��תFUNS02
			FUN += 1;
			if(FUN > #FUNM) {						//������ѭ���ӣ������������������
				FUN = 0;
			}
		}
		
		if(KEY.4 == 0) {							//����DOWN�� �򷵻�
			FUN -= 1;								//�������Լ���С��0ʱ����Ϊ���������
			IF(FUN > #FUNM) {
				FUN = #FUNM;
			}
		}
	} else {										//�������ֹ�����,FUN 01- 02, FUN 15-17, FUN 13-0F, FUN 0B-0D, FUN 38-3A ������ʾ
		if(KEY.5 == 0) {
			if(FUN == 0 || FUN == 0x06 || FUN == 0x0A || FUN == 0x0E \
				|| FUN == 0x14 || FUN == 0x2C || FUN == 0x2F || FUN == 0x37) {			
				
				FUN += 1;									
				if(FUN == 0x01) {					//���ι�����FUN 01- 02
					FUN += 1;
				} else if(FUN == 0x0B || FUN == 0x15 || FUN == 0x38) {		//���ι�����FUN 0B-0D, FUN 15-17, FUN 38-3A
					FUN += 2;
				} else if(FUN == 0x0F) {			//���ι�����FUN 13-0F
					FUN += 4;
				}
			} else {
				FUN += 1;
				if(FUN > #FUNM) {					//������ѭ���ӣ������������������
					FUN = 0;
				}
			}
		}
		
		if(KEY.4 == 0) {
			if(FUN == 00) {							//���ι����� FUN 38-3A	
				FUN = 0x37;
			} else if(FUN == 0x31 || FUN == 0x2E || FUN == 0x18 || FUN == 0x14 \
				|| FUN == 0x0E || FUN == 0x08 || FUN == 0x03) {
				
				FUN -= 1;
				if(FUN == 0x17 || FUN == 0x0D) {		//���ι����� FUN 15-17   FUN 0B-0D
					FUN -= 2;
				} else if(FUN == 0x13) {				//���ι����� FUN 13-0F
					FUN -= 4;
				} else if(FUN == 0x02) {				//���ι�����FUN 01- 02
					FUN -= 1;
				}
			}
		}
	}
}         */


/************************************************************
*            ��ʾ�������ӳ���
*IN  PARAMETER: FUN
*MID PARAMETER: AX1,BX1  ����ʾ������"b" "F"��+������
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
	DBUF5 |= #3FH;										//#0011 1111B ��ζ���ֵ�ǰ DO DC LED��״̬
}*/


/****************************************************************
*         ���ҵ��������Ӧ���ò��������ӳ���
*IN  PARAMETER: KEY,FUN,AX8(��ǰ����������ò���)
*MID PARAMETER: AX1(AL1,AH1),BX1,DX1
*OUT PARAMETER: AX8(��ǰ�������Ӧ�ĺ��ʵ����ò���)
*****************************************************************/
/*void PARS()
{
	AX1 = 2*FUN;
	BX1 = AX8;
	
	if(KEY.bit5 == 0) {									//��UP����װPARD1(UP��=#1101 1111B)
		DX1 = UPPAR[AX1];								//UP�� ��ѵ�ǰ������Ĳ������޷���DX1��,�������PARU1
		
		if(FUN == 0) {
			BX1 = EFG[0];								//��F00 �趨( ����Ƶ�� )Ƶ��(Hz �Ŵ�100��)��ŵĲ�������BX1��
			BX1 += 1;
			if(BX1 > FOM) {								//���Ƶ�ʴ���FOM�������Ƶ�ʣ��򽫵�ǰƵ����Ϊ���Ƶ��
				BX1 = FOM;
			} else {
				if(BX1 < FLO) {							//�����ǰƵ��С����СƵ�ʣ��򽫵�ǰƵ����Ϊ��СƵ��
					BX1 = FLO;
				}
				EFG[0] = BX1;
				AX8 = BX1;
			}
		} else {
			BX1 += 1;
			
			if(BX1 > DX1) {								//�����ǰ�����������Ƶ�ʣ���ѵ�ǰ������Ϊ������
				BX1 = DX1;
			}
			AX8 = BX1;
		}
	} else if{											//��DOWN ��,(DOWN��= #1110 1111B)
		DX1 = INPAR[AX1];								//������ǰ����������޲�����DX1
		if(FUN == 0) {									//��F00 �趨( ����Ƶ�� )Ƶ��(Hz �Ŵ�100��)��ŵĲ�������BX1��
			BX1 = EFG[0];
			BX1 -= #1;
			if(BX1 < FLO) {								//��ֹ���õ�Ƶ�ʳ���FLO - FOM֮��
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
	  
			if(BX1 > #65530) {							//�Լ������ Ϊʲô��65530��
				BX1 = DX1;
			}
			AX8	= BX1;
		}
	}
}*/
	  
/***************************************************
*            ���ò�����ʾ(����λ)�ӳ���
*IN  PARAMETER: FUN,AX8
*OUT PARAMETER: NONE
****************************************************/
//DFREX:	  LJMP   DFRQCYB				//��仰�Ǹ�ʲô�ģ� �Ȳ��ù�
/*
void DISPAR() 
{ 										//------------- FREQUENCY
	if(FUN != #0) {                		//���ǲ���F00? FG,FOUT
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
	  CMP    FUN,#3                 ;���ǲ���F03? FLO,FOM,F1M
	  JNH    DFRQCYB
	  CMP    FUN,#33                ;���>F03,���ǲ���F33?
	  JLT    DAA1                   ;С��F33,��תDAA1 ( F03< <F33)
	  CMP    FUN,#40
	  JGT    DAA1
	  SJMP   DFRQCYB
DAA1:	  CMP    FUN,#16                ;���ǲ���F16?       FBK
	  JE     DFRQCYB
	  CMP    FUN,#51		;���ǲ���F51?
	  JE     DFRQCYB                ;CLOSE BASE FREQ.
	  CMP    FUN,#19		;���ǲ���F19?
	  JE     DFRQCYC                ;FC
	  CMP    FUN,#46		;���ǲ���F46?
	  JLT    GGG1			;С����תGGG1
	  CMP    FUN,#49		;���ǲ���F49?
	  JGT    GGG1			;����F49,��תGGG1
	  SJMP   DFRQCYB
GGG1:	  LJMP   NEXT_RAG		;SECOND�ڶ�����ʾ�����ӳ���

DFRQCYA:  JBS    APPL,0,DDOOF           ;APPL.0=1�ϵ���ʾ���������Ŀģʽ����תDDROOF F58 �ɵ�Ƶ��ģʽ (1:�ϵ���ʾ���������Ŀ,0:�ϵ���ʾ���Ƶ����)
	  JBS    RT,0,DFRQCYA1
	  SJMP   DFG0

DDOOF:	  CMPB   SD_FLG,#2		;�˲���ѧϰ�Ƿ����?
	  JE     DFRQCYA1
	  LD     AX1,NUMBER		;AX1 = ��ǰ��õı�����������
	  LJMP   PEER1			;ֱ�Ӳ���(�޵�λ)��ʾ�ӳ���

DFRQCYA1: CMPB   DSCD,#0		;��ʾģʽF45�ǲ���0 ��ʾ�趨Ƶ��?
	  JE     DFG0
	  CMPB   DSCD,#1		;��ʾģʽF45�ǲ���1 ��ʾ���Ƶ��?
	  JE     DFOH0
	  CMPB   DSCD,#2		;��ʾģʽF45�ǲ���2 ��ʾλ�ðٷ���?
	  JE     DWEIZ
	  LD     AX1,NUMBER		;��ʾģʽF45��3 λ��������?
	  LJMP   DNOUNIT		;��F63ʶ��Ĳ���(�޵�λ)��ʾ�ӳ���

DWEIZ:	  LD     AX1,S			;2011-10 Johnson ��ʾλ�ðٷ��� �ѵ�ǰ�ſ�% ��AX1
	  LJMP   DXX1X0			;���������������ٷ�����ʾ�ӳ���

DFOH0:	  LD     AX1,FOH		;��ʾ���Ƶ��
	  CMP    AX1,#LSP		;AX1 ��LSP�Ƚ�(������СƵ��HZ�ĸ�16bit)
	  JH     DFOH1
	  CLR    AX1
DFOH1:	  LJMP   DISFRE			;Ƶ�ʲ�����ʾ�ӳ���
DFG0:	  LD     AX1,FG		        ;2011-10 Johnson ��ʾ�趨( ���� )Ƶ��  �ѵ�ǰ���Ƶ�ʸ���AX1 "0"HZ MODE
	  CMP    AX1,#LSP		;AX1 ��LSP�Ƚ�(������СƵ��HZ��16bit)
	  JH     DFG1
	  CLR    AX1
DFG1:	  LJMP   DISFRE			;Ƶ�ʲ�����ʾ�ӳ���

DFRQCYB:  LD     AX1,AX8		;**.** HZ--***.*HZ
	  LJMP   DISFRE			;����Ƶ����ʾ��Ƶ�ʲ�����ʾ�ӳ���
DFRQCYC:  LD     AX1,AX8		;**** HZ
	  LCALL  DNOUNIT		;��F63ʶ��Ĳ���(�޵�λ)��ʾ�ӳ���
	  ORB	 DBUF5,#00011111B
	  ANDB	 DBUF5,#11011111B       ;"HZ"LED��
	  RET
}

;****************************************
;   FREQUENCY DISPLAY Ƶ�ʲ�����ʾ�ӳ���
;IN  PARAMETER:
;MID PARAMETER:
;OUT PARAMETER: DBUF1,2,3,4,5
;****************************************
DISFRE:   LCALL  TRANS			;Translation����ʾ����ת���ӳ��� 16����ת10����
	  CMPB	 EL1,#03H
	  JNE	 DISFRE2
	  CMPB	 DH1,#03H
	  JNE	 DISFRE1
	  LDB	 DH1,#0FFH
DISFRE1:  LDB	 DBUF1,DH1	      	;**.** HZ  ÿ��Ҫ�ж���λ�ǲ���"0"�ַ�����λ"0"����ʾ
	  ANDB	 DL1,#0FEH
	  LDB	 DBUF2,DL1              ;��#0FEH����DL1��С����
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  ORB	 DBUF5,#00011111B
	  ANDB	 DBUF5,#11011111B      	;HZ
	  RET
DISFRE2:  LDB	 DBUF1,EL1	        ;***.* HZ
	  LDB	 DBUF2,DH1
	  ANDB	 DL1,#0FEH
	  LDB	 DBUF3,DL1              ;��#0FEH����DL1��С����
	  LDB	 DBUF4,CH1
	  ORB	 DBUF5,#00011111B
	  ANDB	 DBUF5,#11011111B	;HZ
	  RET
;******************************************************
;         SECOND�ڶ�����ʾ�����ӳ���
;IN  PARAMETER: FUN
;MID PARAMETER:    �����ǰѹ�����ֳ����������ִ���
;OUT PARAMETER:
;******************************************************
NEXT_RAG: CMP    FUN,#5                	;���ǲ���F05?     ACL1
	  JLT    GGG3                  	;<F05,��תGGG3
	  CMP    FUN,#10               	;���ǲ���F10?     DCL1
	  JGT    GGG3                  	;>F10,��תGGG3
	  LJMP   SECONDX
GGG3:	  CMP    FUN,#17               	;���ǲ���F17?     TBK
	  JE     GGG4
	  CMP    FUN,#57               	;���ǲ���F57?
	  JE     GGG4
	  CMP    FUN,#42               	;���ǲ���F42?
	  JLT    GGG5                  	;<F42,��תGGG5
	  CMP    FUN,#44               	;���ǲ���F44?
	  JGT    GGG5                  	;>F44,��תGGG5
GGG4:	  LJMP   SECOND
	;-----------**.* %
GGG5:	  CMP    FUN,#4                 ;���ǲ���F04?
	  JE     GGG6
	  CMP    FUN,#52                ;���ǲ���F52?
	  JE     GGG6
	  CMP    FUN,#53                ;���ǲ���F53?
	  JE     GGG6
	  CMP    FUN,#54                ;���ǲ���F54?
	  JE     GGG6
	  CMP    FUN,#18                ;���ǲ���F18?
	  JE     GGG6
	  CMP    FUN,#14                ;���ǲ���F14?
	  JE     GGG6
	  CMP    FUN,#41                ;���ǲ���F41?
	  JE     GGG6
	  CMP    FUN,#25                ;���ǲ���F25?
	  JLT    GGG7                   ;<F25,��תGGG7
	  CMP    FUN,#32                ;���ǲ���F32?
	  JGT    GGG7                   ;>F32,��תGGG7
GGG6:	  LJMP   DXX1XA	              	;**.*% λ������ٷ���������ʾ�ӳ���
GGG7:	  LJMP   NOUNIT                 ;����F32,С��F25����, ���ô���ʾ����(�޵�λ)��ڵ���ʾ�ӳ���

DPERCNT:  LD	 AX1,AX8              	;*** %
DPCTW:	  LCALL  TRANS		      	;Translation����ʾ����ת���ӳ��� 16����ת10����
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
	  LCALL  DNOUNIT		;��F63ʶ��Ĳ���(�޵�λ)��ʾ�ӳ���
SECOND1:  ORB	 DBUF5,#00111110B
	  ANDB	 DBUF5,#11111110B       ;����"SEC" LED  2011-10 ����о�Ӧ���ǶԵģ�������DO DC LED���Ʋ�����
	  RET
SECONDX:  LD	 AX1,AX8
	  LCALL  TRANS			;Translation����ʾ����ת���ӳ��� 16����ת10����
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
;   ����ʾ����(�޵�λ)��ڵ���ʾ�ӳ���
;******************************************
NOUNIT:   LD     AX1,AX8		;�ѵ�ǰ���������ò���-->AX1   ****

;******************************************
;	��F63ʶ��Ĳ���(�޵�λ)��ʾ�ӳ���
;******************************************
DNOUNIT:  CMP	 FUN,#63		;�������ǲ���63? ��������������58 �����Ա�����Ч��
	  JNE	 PEER1
	  ;LD	AX1,TIMER1[0]
	  LD	 AX1,VDC		;���F63 ��׼����ʾVDC���� AX1 = ��ǰֱ��ĸ�ߵ�ѹ
;******************************************
;	ֱ�Ӳ���(�޵�λ)��ʾ�ӳ���
;******************************************
PEER1:	  LCALL  TRANS			;Translation����ʾ����ת���ӳ��� 16����ת10����(���AX1��������EL1,DL1,CH1,CL1)
	  CMPB	 EL1,#03H               ;ADISP[03]="0" ��EL��ת�������λ��Ҳ���жϴ�����С���ı�־��EL����"0"�ַ����Ǵ���
	  JNE	 HINUT
	  CMPB	 DH1,#03H
	  JNE	 LONUT
	  LDB	 DH1,#0FFH              ;TRANS���˳����EL1,DH1,DL1,CH1,CL1, ��λ'0"�ַ�����ʾ
	  CMPB	 DL1,#03H               ;�Ƚ���һλ�ǲ���"0"�ַ�? ���������Ҫ�ٴβ���ʾ
	  JNE	 LONUT
	  LDB	 DL1,#0FFH
	  CMPB	 CH1,#03H               ;�Ƚ���һλ�ǲ���"0"�ַ�? ���������Ҫ�ٴβ���ʾ
	  JNE	 LONUT
	  LDB	 CH1,#0FFH
LONUT:	  LDB	 DBUF1,DH1
	  LDB	 DBUF2,DL1
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  RET

HINUT:	  ANDB	 DH1,#0FEH               ;**.** ��0FEH���õڶ�λ��С����
	  LDB	 DBUF1,EL1
	  LDB	 DBUF2,DH1
	  LDB	 DBUF3,DL1
	  LDB	 DBUF4,CH1
	  RET*/

/******************************************
*        λ������ٷ���������ʾ�ӳ���
*IN  PARAMETER: AX8(Ҫ��ʾ�Ĳ���)
*MID PARAMETER:
*OUT PARAMETER:
*******************************************/
/*
DXX1XA:   LD	 AX1,AX8		;AX1 = ��ǰҪ��ʾ�Ĳ���

;****���������������ٷ�����ʾ�ӳ���****

DXX1X0:   LCALL  TRANS	                ;Translation����ʾ����ת���ӳ��� 16����ת10���� **.*%
	  CMPB	 DH1,#03H               ;DH-DL-CH-CL
	  JNE	 DXX2X0
	  LDB	 DH1,#0FFH              ;��λ�������"0"�ַ�Ҫ����ʾ
	  CMPB	 DL1,#03H
	  JNE	 DXX2X0
	  LDB	 DL1,#0FFH
DXX2X0:   LDB	 DBUF1,DH1
	  LDB	 DBUF2,DL1
	  ANDB	 CH1,#11111110B	        ;DIP ��0FEH,��CL1��С����
	  LDB	 DBUF3,CH1
	  LDB	 DBUF4,CL1
	  ORB	 DBUF5,#00111011B	;HAD LED ˳��: DO  DC  Hz % SEC MIN MIC MIS (����MIN MIC MIS HAD����,GDV10��֧��SEC MIN MIC MIS)
	  ANDB	 DBUF5,#11111011B       ;Ϊʲô�����"%"��?  ������Ե�ȷ���ˣ�
	  RET*/

/******************************************************************
*	    ��ǰ���������ò���д���ӦEEPROM �����뵥Ԫ�ӳ���
*IN  PARAMETER: KEY,FUN,AX8(Ҫд������ò���)
*MID PARAMETER:
*OUT PARAMETER: AX8(����д��EERPOM���ٴζ��ص����ò���)
*******************************************************************/
/*void WRS()
{
	AX1 = FUN + FUN;									//AX1 = FUN + FUN ���ݹ�������������ӦEEPROM�����ַ
	if(KEY.bit7 == 1) {									//��д���(#0111 1111B)��תWRS1
		if(FUN != #20) {								//��ǰ��������F20��? ��Ҫ��Ϊ�����ַ�F20�������ǲ��ǵ�ǰ֧�ֲ�д418����ʾ���й�����8080
			BX1 = ESAV[0];								//�������F20����Ҫ���ڴ�ESAV ���ݱ��������BX1
			if(BX1 == #8080 || BX1 == #418) {			//��8080?
														//F20=8080 ��Ҫȫ����ʾ������	
				AX9 = FUN + FUN;
				AX1 = AX8;
				EWRE();		     						//����EEPROMд�����
				DELAY2(2);								//Add for AT93C66
				ERDE();									//д��������

				E2M[AX9] = AX1;							//�ٴα��浽�ù������Ӧ��RAM��Ԫ
				AX8	= AX1;								//AX8 = д��EEPROM����ص����ò���

				DBUF1 = #61H;         					//E
				DBUF2 = #99H;           				//4 RESTORE FACETORY OK LABLE->E200 #25H
				DBUF3 = #0FDH;          				//-
				DBUF4 = #0FDH;          				//-
				DBUF5 |= #3FH;           				//���ֵ�ǰLEDԼ����ʽ
				BL1 = #50;
				while(BL1 != 0) {
					DISPLY();							//���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���
					DELAY();
					BL1 --;
				}
			} else {
				AX1 = FUN + FUN;						//���ESAV����8080 Ҳ���� 418�����ڱ������ι�����������,��֧��д��
				BX1 = E2M[AX1];							//�Ѷ�Ӧ�������RAM��Ԫ������BX1
				AX8	= BX1;								//AX8 = ��ǰ����������ò��������ǲ�֧��д��eeprom������ʾ"-Err"����
				DBUF1 = #0FFH;							//-
				DBUF2 = #61H;							//E
				DBUF3 = #0F5H;							//r
				DBUF4 = #0F5H;							//r
				DBUF5 |= #3FH;							//���ֵ�ǰLEDԼ����ʽ

				BL1 = #50;
				while(BL1 != 0) {
					DISPLY();							//���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���
					DELAY();
					--BL1;
				}
			}
		} else {										//F20=8080 ��Ҫȫ����ʾ������
			AX9 = FUN + FUN;
			AX1 = AX8;
			EWRE();		     						//����EEPROMд�����
			DELAY2(2);								//Add for AT93C66
			ERDE();									//д��������

			E2M[AX9] = AX1;							//�ٴα��浽�ù������Ӧ��RAM��Ԫ
			AX8	= AX1;								//AX8 = д��EEPROM����ص����ò���

			DBUF1 = #61H;         					//E
			DBUF2 = #99H;           				//4 RESTORE FACETORY OK LABLE->E200 #25H
			DBUF3 = #0FDH;          				//-
			DBUF4 = #0FDH;          				//-
			DBUF5 |= #3FH;           				//���ֵ�ǰLEDԼ����ʽ
			BL1 = #50;
			while(BL1 != 0) {
				DISPLY();							//���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���
				DELAY();
				BL1 --;
			}
		}
	}
}*/


/*************************************************************
*         ����ǰ����״̬�ӳ���
*IN  PARAMETER: KEY(�ϴ���ѭ��������ֵ)
*MID PARAMETER: AL1,CL1
*OUT PARAMETER: KEY(�����ӳ��������ֵ),RT,TERMAL
**************************************************************/
void KEYB()
{
	AL1 = read_key();								//��P0��,P0.0/0.1/0.2/0.3�����������
	KEY138();                  						//AL1 Ϊ���ض�������ȷ��ֵ����λ��Ч 1111XXXXB ������ֵ����ת���ӳ���
	
	if(AL1 != KEY) {								//�Ƚϱ��ζ����ļ�����ֵ���ϴε��Ƿ�һ����
		DELAY();									//�����һ��������ʱ���ٶ����ٱȽϣ��൱�ڼ��̵�����˲�
		AL1 = read_key();

		KEY138();
		if(AL1 != KEY) {
			KEY = AL1;								//��������˲���ȷ���µļ�����ֵ�������KEY
		}
	}
	
	if(APPL & 0x01 == 0) {							//��鵱ǰ����ʾ������������������Ƶ����? APPL.bit0 == 0 1->�ϵ���ʾ���������Ŀ,0->�ϵ���ʾ���Ƶ��ֵ
		if(KEY & 0x08 == 0) {						//Key.3=1�ʹ���ǰ������λ��ֹͣ��	KEY.bit3 == 0
			RT &= 0xFE;								//�������λ��ֹͣ��������RT=#1111 1110B
		}
	} else {
		AL1 = read_key();							//AL1 = P0_PIN[0]
		CL1 = E2M[EEXCM];							//CL1 = EEXCM (F15�ⲿͨ��ģʽ �̶�Ϊ1)
		if(CL1 & 0x01 == 0x01) {             		//EEXCM=0 ��תEXCM1  CL1.bit0 == 1
			AL1 = ~AL1;								//EEXCM=1,  AL1ȡ��������IN4��IN3��ͨ�����������뵽CPU�ģ��ʶ�ȡ���������ʵ���� 15/12-1999
		}
		
		AL1 = AL1 >> 4;                  			//����4λ���൱�ڰѶ��ӿ�������״̬�Ƶ�AL1�ĵ�4λ
        AL1 |= 0xF3;          						//ȡIN4��IN3		AL1 |= #11110011B; 
		if(AL1 != TERMAL) {							//�Ƚϱ��ν�����ϴ�TERMAL�����Ƿ����?
			DELAY();								//����ȣ�����ʱ24ms
			AL1 = read_key();						//�ٴζ�  AL1 = P0_PIN[0]  �൱�ڼ��̵�����˲�
			
			if(CL1 & 0x01 == 0x01) {             	//EEXCM=0 ��תEXCM2		CL1.bit0 == 1
				AL1 = ~AL1;                    		//EEXCM=1,  AL1ȡ��������IN4��IN3��ͨ�����������뵽CPU�ģ��ʶ�ȡ���������ʵ���� 15/12-1999
			}
			
			AL1 = AL1 >> 4;                  		//����4λ���൱�ڰѶ��ӿ�������״̬�Ƶ�AL1�ĵ�4λ
			AL1 |= 0xF3;		          			//ȡIN4��IN3	AL1 |= #11110011B;
			
			if(AL1 != TERMAL) {              		//�Ƚϱ��ν�����ϴ�TERMAL�����Ƿ����?
				TERMAL = AL1;              			//����λ����ⲿ���������TERMAL��־
			}
		}
	}
	
}

/*�����ֵ��������ֵ�������temp����*/
unsigned char read_key()
{
	unsigned char temp = 0xFF; 
	
   if(GpioDataRegs.GPADAT.bit.GPIO31 == 1)  		//���м�����ʱ���ͰѼ�ֵ����temp�ĵ�λ
   		temp &= 0xFE;
   if(GpioDataRegs.GPBDAT.bit.GPIO32 == 1)  		//���м�����ʱ���ͰѼ�ֵ����temp�ĵڶ�λ
   		temp &= 0xFD;
   if(GpioDataRegs.GPBDAT.bit.GPIO33 == 1)  		//���м�����ʱ���ͰѼ�ֵ����temp�ĵ���λ
   		temp &= 0xFB;
   if(GpioDataRegs.GPBDAT.bit.GPIO34 == 1)  		//���м�����ʱ���ͰѼ�ֵ����temp�ĵ���λ
   		temp &= 0xF7;
   	
   	return temp; 									//����ֵ
}


/**********************************************************
*          �Զ���ʾ�ӳ���
*IN  PARAMETER: KEY CONTROL,DYCT
*OUT PARAMETER: CONTROL
***********************************************************/
/*void AUTODY()
{
	if(KEY.bit5 == 1 && KEY.bit4 == 1) {							//��UP������תAU1�������
		CONTROL &= #11111101B;										//�������UPҲ����DOWN�������CONTROL.1=0
		DYCT = #0;													//DYCT����0����DYCT���¼���
	} else {
		if(CONTROL.bit1 == 1) {           							//CONTROL.1=0 ��תAU4
			if(DYCT <= #160) {               						//DYCT��TIMER2�м�1�������160��Ӧ 160 x 16ms = 2.56��
				DELAY2(8);
			}
		} else {
			DELAY2(200);
			CONTROL &= #00000010B;								//��CONTROL��#02H�ֽ��߼���
		}
	}
}*/


/***************************************************
*                ��ʾ "DCB-"�ӳ���
*IN  PARAMETER:
*OUT PARAMETER: DBUF1,DBUF2,DBUF3,DBUF4
****************************************************/
/*void JTDC() 
{
	DBUF1 = #85H;					//"B"
	DBUF2 = #63H;					//"C"
	DBUF3 = #0C1H;					//"D"
	if(FRSH.bit4 ==1) {            	//Ŀ���������һλ������˸
		DBUF4 = #0FFH;				//NONE
	}

	DBUF4 = #0FDH;					//"-"
}*/
	  
/****************************************************
*               SPWM_WG ���������жϷ����ӳ���
*IN  PARAMETER:
*MID PARAMETER: �ڱ������ʼ���������ĶԳƷ�ʽ�����ϼ���
*OUT PARAMETER: ÿ���ز����ڽ�������һ���жϣ�����ֻ�з�
*       ���ز����ڸı䣬����ÿ�ζ���һ���ز����ڽ���һ��
*****************************************************/
/*
SPWM:	  PUSHF                     	;6
	  EI			    	;2
	  XORB	 NEWVECT,#1		;	NEWVECT ���00000001B,ÿ�ν����ֱ����SPWM0 �� SPWM1
					;       ���������ǰʸ���ǶȺ�Vrms���´ν������Ta,Tb,Tc�ƶ�PWMռ�ձȱ仯
	  JBC	 NEWVECT,0,SPWM0	;	NEWVECT=0 ����תSPWM0
	  SJMP	 SPWM1			;	������תSPWM1

SPWM0:	  SUB	 AX,FGC,FOH	     	;5	AX = FGC - FOH (��ǰ����Ƶ�� - �ϴ�����Ƶ��)
	  JH	 INC11		    	;8	>0,����תINC11
	  NEG	 AX		    	;3	���FGC-FOH<0, ��AX��Ϊȡ�������Ÿı䣬������ֵ����
	  CMP	 AX,DECFOH	    	;4	�Ƚ�AX ��DECFOH
	  JH	 DEC21			;	���AX > DECFOH, ����תDEC21
	  LD	 FOH,FGC		;	���AX < DECFOH, ��FOH = FGC
	  SJMP	 ANGAL1		    	;SUM=28 ��תANGAL1

DEC21:	  SUB	 FOL,DECFOL          	;4+8(JH)FOL = FOL - DECFOL ?
	  SUBC	 FOH,DECFOH          	;4	FOH = FOH - DECFOH - ��λΪ0������1
	  SJMP	 ANGAL1	            	;7	��תANGAL1
INC11:	  CMP	 AX,INCFOH		;	�Ƚ�AX��INCFOH
	  JH	 INC21			;	���AX > INCFOH,����תINC21
	  LD	 FOH,FGC		;	���AX <=INCFOH,��FOH = FOH + FGC
	  SJMP	 ANGAL1			;	��תANGAL1
INC21:
	  ADD	 FOL,INCFOL	    	;4	FOL = FOL + INCFOL
	  ADDC	 FOH,INCFOH	    	;4	FOH = FOH + INCFOH + ��λ��־

ANGAL1:   MULU	 CX,KV1,FOH	    	;14	CX  = KV1 * FOH  (KV1�Ǹ���V-F�����������V��ֵ)  CX=�˷��ĵ�16bit DX=�˷��ĸ�16bit
	  SHLL	 CX,#2		    	;7+2	CX ����2λ��CX = CX * 4  �൱��(KV1 * FOH ) *4    ˫����λ��CX��DXͬʱ����
	  ADD	 AX,DX,VTRQ	    	;4	AX  = DX + VTRQ (VTRQ��������ĵ�ǰ���ز�����ѹ��ֵ)
					;	AX  = ((KV1 * FOH)*4 / 65536 ) + VTRQ   [DX �� KV1 * FOH �ĸ�16bit, �ͺñ���KV1 * FOH ����16λ ����DX]

	  LD	 DX,AX			;	DX  = AX = ((KV1 * FOH)*4 / 65536 ) + VTRQ

	  CLR	 CX			;	CX =#0000H
	  SHRL	 CX,#7			;	˫������7�Σ��൱�ڰ�DXCXͬʱ����7�Σ�CX = DX / 128

	  DIVU	 CX,VDC			;	CX = (DX/128) / VDC ���̣�DX = (DX/128) / VDC ������ (VDC �ǵ�ǰֱ��ĸ�ߵ�ѹ������ֵ)
					;       CX = (((KV1 * FOH)*4 / 65536 ) + VTRQ ) /(128 * VDC) ���� (s)


					;	VDC=(1023*��ʵ����ĸ�ߵ�ѹ��ֵ)/5 (V) (Ϊ5V �����ο���ѹ 10bitAD )

	  LD	 VRMS,CX		;	Vrms =(((KV1 * FOH)*4 / 65536 ) + VTRQ ) /(128 * VDC) ����  (Vrms �ǵ�ѹ�ľ�����ֵ)

	  CMP	 VRMS,TZ	    	;4	�Ƚ�Vrms��TZ��С Ϊʲô?
					;	Vrms=(Uref/Vdc)TZ=mTZ ���ǱȽ�m(�������)�Ƿ�>1 ?

	  JNH	 JJX1		    	;4	���Vrms<=TZ ����תJJX1
	  LD	 VRMS,TZ	    	;4 SUM=70 ����������>1 ,���õ������=1������Rrms=TZ

JJX1:	  MULU	 AX,FOH,DFINR1	    	;14	AX  = FOH * DFINR1  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	  SHLL	 AX,#3 ;SPWM0 MODE  	;7+5	AX  = (BX AX ) * 8 (����3���൱��* 8) �ĵ�16λ
					;	SPWM0 MODE �������Ķ�׼PWMģʽ������ģʽ�õ���PWM��������г��С AC������������ʽ
	  ADD	 BX,BX			;	BX  = BX * 2 = ((FOH * DFINR1 *8 )/65536) * 2   BX�� FOH * DFINR1 �ĸ�16λ���൱��FOH * DFINR1����16λ

	  ADD	 FIA,BX 	    	;4	FIA = FIA + ((FOH * DFINR1 * 8)/65536) * 2  (FIA ÿ��WG�жϻ���������BX,����һ��������н�λ������)
					;	��DFINR1��ʽ����: FIA = FIA + ( FOH * TZ * 10 )/5086  Ӧ��40960��Ŀ��֪���ˣ����Ա�Լ��
				        ;	FIA = ��ǰ��ʸ����ת�Ƕ�
					;       ZDK��((FOH * DFINR1 * 8)/65536) * 2 ��*2 ����Ϊ���ٴν���WG�ж�SPWM0�м���һ�Σ�������2�νǶ����ӡ�
					;	�� / 65536 ����ΪZDK��60�ȶ�Ӧ��65536 ����������������λ��Ҫ�ƶ�SECTOR��

	  JNC	 THISSEC	    	;4	���û�н�λ��־������תTHISSEC
					;	����н�λ��־��ζFIA������65536(2^16)�����ǳ�Խ��һ��60������

	  SHLB	 SECTOR,#1	    	;6+1	�����Խ��һ������������SECTOR ����1λ�� SECTOR��1λ����ǰ���ĸ�����
	  JBC	 SECTOR,6,THISSEC   	;5	SECTOR.6=0? �����0����ζ��ǰ����������1-6֮�� ����תTHISSEC
	  LDB	 SECTOR,#1	    	;4 SUM=50 ���SECTOR.6=1��ζ�ǵ�7����������ǿ����������SECTOR.0=1�ӵ�һ��������ʼ

THISSEC:  LD	 FIX,FIA	    	;4	FIX = FIX + FIA
	  SHR	 FIX,#5 	    	;6+5	FIX = FIX / 32 (����5λ������/32 ʣ��11bit���� MAX to 2048) ��ʾ�Ƕȱ仯ֻ����2048���ֽ���(1024�����)
	  AND	 FIX,#0FFFEH	    	;5	FIX = FIX �� #0FFFEH (Ŀ����ȡż����SINE�����ִ洢�����Ҫż����ַ)
	  SUB	 FIY,TAB,FIX	     	;5	FIY = TAB - FIX (TAB ��2048, �൱��2048���ֽڣ�60��)
	  POPF				;       FIY = 60Degree - FIX
	  RET

SPWM1:	  MULU	 T1,VRMS,SINTA[FIX]	;15	T1  = Vrms * SINTA[FIX]  T1=�˻���16bit T2=�˻���16bit= (Vrms * SINTA[FIX]) / 65536
					;	���۹�ʽt2 =mT*sin(theta)
					;
	  MULU	 CX,VRMS,SINTA[FIY]    	;15	CX  = Vrms * SINTA[FIY] = Vrms * SINTA( 60 - FIX)  CX=�˻���16bit DX=�˻���16bit
					
	  LD	 T1,DX		       	;4	T1  = DX = (Vrms * SINTA[FIY]) / 65536   SINTABLE[] = 65536 * SIN ( 60N/1024)
					;       ���۹�ʽt1 =mT*sin(60-theta)
					;  
       ;  JBS	NEWVECT,7,DCBAKE1   	;5 SUM=64 �ƶ�?
	  JBC	 CONTROL,2,OVERM	;	CONTROL.2=0 (ֱ���ƶ���Ч?) ��תOVERM (����Ƿ������?)

DCBAKE1:  LD	 T1,TDC			;	T1  = TDC
	  LD	 T2,TDC			;	T2  = TDC
	  LDB	 SECTOR,#1		;       SECTOR = 1
ENDSPWM1:

;*********������?******************
OVERM:	  ADD	 CX,T1,T2           	;9+5	CX =  T1 + T2
	  SUB	 DX,TZ,CX           	;5	DX =  TZ -CX  (DX = TZ - T1 - T2)  ����SVPWM�е���Чʸ��ʱ��t0=Tpwm-t1-T2
	  JGT	 FULLPWM1           	;4	���t0 (TZ - T1 -T2)>0, ����תFULLPWM1
	  CLR	 DX	            	;3	t0( TZ - T1 - T2 ) <=0  ����DX, ��ζ��t0 = 0
	  SUB	 T1,TZ,T2           	;5	T1 = TZ - T2

FULLPWM1:
	 ;SHR	 DX,#1		    	;6+1 	SUM=38
	  CLR	 DX			;	��DX��Чʸ��ʱ��=0   ʵ�ʷ��治��0��������
SPLIT21:  JBS	 SECTOR,0,SEC11	    	;5	SECTOR.0=1? ����SEC11
	  JBS	 SECTOR,1,SEC21	    	;5	SECTOR.1=1? ����SEC21
	  JBS	 SECTOR,2,SEC31	    	;5	SECTOR.2=1? ����SEC31
	  JBS	 SECTOR,3,SEC41	    	;5	SECTOR.3=1? ����SEC41
	  JBS	 SECTOR,4,SEC51	    	;5	SECTOR.4=1? ����SEC51
					;       DX = ��Чʸ��ʱ��=0

SEC61:	  LD	 AX,DX		    	;4	��ζ���ֻ���ǵ�6��������  AX =  DX
	  ADD	 BX,AX,T2	    	;5	BX = AX + T2
	  ADD	 CX,BX,T1	    	;5	CX = BX + T1
	  SJMP	 WOUT1		    	;7 SUM=46 ��תWOUT1(���PWM)

SEC51:	  LD	 BX,DX			;	BX = DX
	  ADD	 AX,BX,T1		;       AX = BX + T1
	  ADD	 CX,AX,T2		;       CX = AX + T2
	  SJMP	 WOUT1			;       ��תWOUT1
SEC41:	  LD	 BX,DX			;	BX = DX
	  ADD	 CX,BX,T2		;	CX = BX + T2
	  ADD	 AX,CX,T1		;       AX = CX + T1
	  SJMP	 WOUT1			;	��תWOUT1

SEC31:	  LD	 CX,DX			;	CX = DX
	  ADD	 BX,CX,T1		;	BX = CX + T1
	  ADD	 AX,BX,T2		;       AX = BX + T2
	  SJMP	 WOUT1			;	��תWOUT1

SEC21:	  LD	 CX,DX			;	CX = DX
	  ADD	 AX,CX,T2		;	AX = CX + T2
	  ADD	 BX,AX,T1		;	BX = AX + T1
	  SJMP	 WOUT1			;	��תWOUT1

SEC11:	  LD	 AX,DX			;	AX = DX
	  ADD	 CX,AX,T1		;	CX = AX + T1
	  ADD	 BX,CX,T2		;	BX = CX + T2

WOUT1:	  JBS	 MST,0,RUNREV1		;	MST.0=1? ���� ����תRUNREV1 ( ������)

;**************�������?*****************

RUNFWD1:  ST	 AX,WG_COMP1[0]		;	���������Ƚϼ��������ı�ռ�ձȾ��Ǹı�������Ƶ��
	  ST	 BX,WG_COMP2[0]		;	DUTY = ( WG_COMPx / WG_RELOAD ) * 100%
	  ST	 CX,WG_COMP3[0]
POPA6:	  POPF
	  RET
;*******�����ת�ͷ�תֻ�ǻ���UV��˳��***

;**************������?*****************

RUNREV1:  ST	 BX,WG_COMP1[0]       	;8+9	���������Ƚϼ��������ı�ռ�ձȾ��Ǹı�������Ƶ��
	  ST	 AX,WG_COMP2[0]       	;8	DUTY = ( WG_COMPx / WG_RELOAD ) * 100%
	  ST	 CX,WG_COMP3[0]       	;8
POPA7:	  POPF			      	;7
	  RET			      	;11     SUM=51
				      	;TOTAL  346 -> 86.5 US (346 x 250ns=86.5us by Johnson) */

						
/************************************************
*        ��ʱ�� TIMER2����жϷ������
*
*IN  PARAMETER:
*OUT PARAMETER:
*REMARK: ʵ��ֻ��TIMER2�Ķ�ʱ����TIMER1ΪPOLLING
*��ʱ������ʱ����Ӧ����16ms
*TIMER2(WORD)=65535 TIMER2�ķֱ�����250ns
*���Լ��=65536 x 250ns=16384000ns = 16.4ms
*���Գ�����6024(��)= 6024 x 16.4=98.79��=99��
*          6000(��)= 6000 x 16.4=98.4��
*************************************************/
/*TIMOV:	  PUSHF
	  EI
	  INCB  FRSH			;FRSH��־+1
	  ORB   RT,#00000100B		;ÿ�ν��붨ʱ�ж϶���RT.2=1����
	  JBC   CONTROL,1,TOV1		;CONTROL.1=0������תTOV1
	  INCB  DYCT			;DYCT��־+1
	  CMPB  DYCT,#200		;DYCT�Ƿ�200? 200* 16ms = 3.2��
	  JNH   TOV2			;���DYCT<=200 ���תTOV2
	  LDB   DYCT,#200		;���DYCT>200����DYCT=200
	  SJMP  TOV2
TOV1:	  CLRB  DYCT			;DYCT=00H
TOV2:	  JBC   RT,4,TOV3              	;RT.4=0����תTOV3 ��ѧϰ�ƽ���ʼ? STUDY PUSH BEGINS
	  INC   SPT			;SPT��־+1
	  CMP   SPT,#6000              	;6024==99SEC
	  JNH   TOV31			;SPT<=6000����תTOV31
	  LD    SPT,#6000	       	;6024==99��
	  SJMP  TOV31
TOV3:	  CLR   SPT			;SPT=00H
TOV31:	  JBS   RT,5,TOV4              	;RT.5=1 ����תTOV4	ANY OF DC or OC BEGIN TO COUNT
	  JBS   RT,6,TOV4		;RT.6=1 ����תTOV4
	  SJMP  TOV41

TOV4:	  INC   DOCPT			;DOCPT+1
	  CMP   DOCPT,#6000		;
	  JNH   TOV42			;DOCPT<=6000����תTOV42
	  LD    DOCPT,#6000		;���DOCPT>6000 ����DOCPT=6000
	  SJMP  TOV42

TOV41:	  CLR   DOCPT			;DOCPT=00H
TOV42:	  JBC   CONTROL,2,TOV43		;CONTROL.2=0 ����תTOV43
	  INC   DCCT			;DCCT+1
	  CMP   DCCT,#6000
	  JNH   TOV5			;DCCT<=6000 ����תTOV5
	  LD    DCCT,#6000		;���DCCT>6000 ����DCCT=6000
	  SJMP  TOV5

TOV43:	  CLR   DCCT			;DCCT=00H
TOV5:	  JBC   RT,1,TOV6		;RT.1=0 ����װTOV6
	  INC   OBPT			;OBPT+1 ���ؼ���ƽ�ʱ��
	  CMP   OBPT,#6000		;���ؼ���ƽ�ʱ��
	  JNH   TOV61			;OBPT<=6000 ����תTOV61
	  LD    OBPT,#6000		;���OBPT>6000 ����OBPT=6000
	  SJMP  TOV61

TOV6:	  CLR   OBPT			;OBPT=00H  ���ؼ���ƽ�ʱ��
TOV61:	  ADDB  SAMP_T,#32		;SAMP_T=SAMPT+32

	  CMPB  SAMP_T,#0		;SAMP_T�Ƿ����0?  SAMP_T + 8��32�ܵ��㣬��ζ����8*16ms=128ms

	  JNE   PPPP			;���SAMP_T������0������תPPPP

	  JBC   FRSH,3,PPPP          	;FRSH.3=0 ����תPPPP  SAMPLE TIME PER 4-TIMOV

	  SUB   DTS,AX3,NUMBER		;FRSH.3=1 (1000B *16ms=128ms) DTS = AX3 - NUMBER �˲��Ƿ������ϰ���?
	  LD    AX3,NUMBER		;AX3=NUMBER
PPPP:	  JBC   RT,3,PPPP2		;RT.3=0 ����תPPPP2
	  INC   JDLY			;JDLY+1
	  CMP   JDLY,#6000		;JDLY��6000�Ƚ�
	  JNH   PPPP1			;JDLY<=6000 ����תPPPP1
	  LD    JDLY,#6000		;JDLY>6000 ����JDLY=6000
PPPP1:	  SJMP  POPA9			;ֱ�ӷ���

PPPP2:	  CLR   JDLY			;JDLY=00H
POPA9:	  POPF
	  RET				;����*/

/***************************************************
*          EXTINT�ⲿ�жϱ����������
*IN  PARAMETER: NONE
*OUT PARAMETER:
***************************************************/
/*
EXINT:	  DI			        ;�ر������ж�
	  LDB   AL,P2_REG[0]
	  ORB   AL,#00010000B		;��ζZDK��PWM�����245�ر�,���ҵİ�û������,����IR2130Sʵ�ʵ�ǰ�Ѿ��ر�PWM������.����Σ�յ���IR2130S����ʱ�����Զ���PWM����!!
	  STB   AL,P2_REG[0]

	  LDB   AL,P1_PIN[0]		;���P1.0�Ƿ��ѹ?  ��ѹOU �͵�ƽ��Ч!
	  JBC   AL,0,EXINT1		;�����ѹ��תEXINT1
	  ANDB  AL0,#11111110B	        ;���ǹ�ѹ���ǹ��� SC-ACH9-P1.1
	  SJMP  TEST
EXINT1:   ANDB  AL0,#11111101B	        ;OU-ACH8-P1.0
TEST:	  DI				;�ر������жϣ��е��ظ�

	  LDB   AL,P2_REG[0]		;��ζZDK��PWM�����245�ر�,���ҵİ�û������,����IR2130Sʵ�ʵ�ǰ�Ѿ��ر�PWM������.
	  ORB   AL,#00010000B		;�о��е��ظ�
	  STB   AL,P2_REG[0]

	  CLRB  AL			;�ֽ���AL�Ĵ���
	  STB   AL,PI_PEND[0]
	  STB   AL,PI_MASK[0]		;��������PI�ж�Դ

	  LDB   AL,WG_PROTECT[0]
	  ANDB  AL,#0FEH		;�ֽ���#11111110B,��ζ��WG_PROTECT.0(EO)=0,��ֹ���PWM
	  STB   AL,WG_PROTECT[0]

	  CLRB  INT_PEND
	  CLRB  INT_PEND1
	  CLRB  INT_MASK
	  CLRB  INT_MASK1
	  LDB   AL1,PI_MASK[0]
	  ANDB  AL1,#11101111B		;���������WG�ж�
	  STB   AL1,PI_MASK[0]

TEST1:	  LDB   AL1,#0
	  STB   AL1,PWM0[0]		;���PWM0 ռ�ձȿ��ƼĴ���(MC186.inc)��PWM0->P6.6=0 E2CK/ZDK SCH,EECK/JJ,�����������������������֤���Ϸ���

OCX:	  JBS   AL0,0,OUX		;����λΪ1��ת���ж��Ƿ�ǰ�����Ƿ��ѹAL0.0?
	  LD    AX,#4			;������ǹ�ѹ���ض��ǹ���
	  SJMP  FLUR

OUX:	  JBS   AL0,1,ERRX		;�ж�AL0.1�Ƿ�1?�Ƿ�ͬʱҲ����? �����ѹOU������OCͬʱ������תERRX
	  LD    AX,#12
	  SJMP  FLUR
ERRX:	  LD    AX,#16			;�����ѹ����ͬʱ����������ʾ��Ϲ���ָʾ
FLUR:	  LD    DX,AX
	  LDB   DBUF1,ERROR[DX]		;��ʾ���ϱ� oc ��ou ��ͬʱ
	  INC   DX
	  LDB   DBUF2,ERROR[DX]		;
	  INC   DX
	  LDB   DBUF3,ERROR[DX]		;
	  INC   DX
	  LDB   DBUF4,ERROR[DX]
	  LDB   DBUF5,#0FFH             ;�ر����е�LEDָʾ

	  LCALL DISPLY1			;��ʾ"err" ֱ��(ǿ��RT.2=0)��������ݷ�����ʾ�����ӳ���
	  LD    DLY,#50                 ;��ʱ50��������λʱ��,2.4msx50=100ms
	  LCALL DELAY2

NOMEM:	  LDB   AL,P2_REG[0]		;��ζZDK��PWM�����245�ر�,���ҵİ�û������,����IR2130Sʵ�ʵ�ǰ�Ѿ��ر�PWM������.
	  ORB   AL,#00010000B		;�о��ٴ��ظ�
	  STB   AL,P2_REG[0]
	  CLRB  AL
	  STB   AL,PI_MASK[0]		;��������PI�������ж�Դ
	  STB   AL,PI_PEND[0]
	  CLRB  INT_PEND
	  CLRB  INT_PEND1
	  CLRB  INT_MASK
	  CLRB  INT_MASK1

FLOOP:	  LDB   AL,ERST[0]		;��F55 ��ǰ�������ϵĻָ�ģʽ���� ERST=0���̴������ģʽ ERST=1 ���̺Ͷ��Ӵ������ģʽ
	  JBS   AL,0,TORST
	  LCALL KEYB			;����ǰ����״̬�ӳ���
	  JBC   KEY,3,NEXT1		;�����ֹͣ������ֱ�Ӹ�λϵͳ
	  SJMP  FLOOP			;Ҫô�������̵�ֹͣ����Ҫô�޸ĵ�ǰ���ϻָ�ģʽ��ֵ������һֱ�ڵȴ�

TORST:	  LCALL KEYB			;����ǰ����״̬�ӳ���
	  JBC   KEY,3,NEXT1		;�����ֹͣ�� ��ֱ�Ӹ�λϵͳ
	  JBC   TERMAL,3,NEXT1		;���ӿ��ţ���ֱ�Ӹ�λϵͳ
	  JBC   TERMAL,2,NEXT1		;���ӹ��ţ���ֱ�Ӹ�λϵͳ
	  SJMP  FLOOP                   ;����ѭ���ȴ�
NEXT1:	  RST				;ϵͳ��λ PSW/PC����,�൱�ڳ�����������
*/


/***********************************************
*          ��ֹPWM����CPU����ӳ���
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
*          ����PWM����CPU����ӳ���
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
*          ����SPWM���η����ӳ���
*IN  PARAMETER:
*OUT PARAMETER:
*************************************************/
void OPSPWM()
{
	
}
/*OPSPWM:   LD    AX1,TZ			;AX = TZ (�ز�����) (s)
	  SHR   AX1,#1			;����һλ������/2
	  ST    AX1,WG_COMP1[0]		;U Phase �Ƚϼ�����Ϊ�ز����ڵ�һ�� ռ�ձ�50%
	  ST    AX1,WG_COMP2[0]		;V Phase �Ƚϼ�����Ϊ�ز����ڵ�һ�� ռ�ձ�50%
	  ST    AX1,WG_COMP3[0]		;W Phase �Ƚϼ�����Ϊ�ز����ڵ�һ�� ռ�ձ�50%
					;����Intel 97C196MC ���ռ�ձ�Ϊ50% Duty=WG_COMPx/WG_RELOAD
	  LDB	AL1,PI_PEND[0]	        ;CLRB  PI_PEND[0]

	  LDB	AL1,PI_MASK[0]
	  ORB	AL1,#00010000B
	  STB	AL1,PI_MASK[0]		;��WG �ж�
	  ORB	INT_MASK1,#01100000B	;��EXTINT��PI�ж�
	  ST	TZ,WG_RELOAD[0]		;�ñ�����ǰ���ز�����
	  LD	AX1,WG_CON[0]		;ȡ��ǰ��WG_CON״̬
	  OR	AX1,#0000010000000000B	;����ȷ��WG_COUT.10=1 ʹ��(start counter)
	  AND	AX1,#1100111111111111B 	;SPWM0 MODE 2011-10 Johnson ģʽ0 �� 1 SPWM��ʽ ���Ķ�׼��ֻ����1�λ�2�μĴ��������Ӽ�����ʹ�ܿ�ʼ������
					;�������ֻ��ȷ��M1=M0=0 ֻ����ζ��SPWM0 �� SPWM1ģʽ
	  ST	AX1,WG_CON[0]
	  RET*/
	  
	  
/************************************************
*         ֹͣSPWM���η����ӳ���
*IN  PARAMETER:
*OUT PARAMETER:
*************************************************/
void STSPWM()
{
	
}
/*
STSPWM:   LDB	AL1,PI_PEND[0]
	  LDB	AL1,PI_MASK[0]
	  ANDB	AL1,#11101111B		;����WG�ж�
	  STB	AL1,PI_MASK[0]
	  ANDB	INT_MASK1,#11011111B	;����PI�жϣ���ζȫ��ֹͣSPWM����
	  CLR	AX1
	  ST	AX1,WG_RELOAD[0]	;�ز���������,��ζֹͣ�ȽϱȽ�
	  ST	AX1,WG_COMP1[0]
	  ST	AX1,WG_COMP2[0]
	  ST	AX1,WG_COMP3[0]
	  RET*/
	  
/***********************************************
*           ������ʾ�����������
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
*        ���г�ʼ��INICIAL�ӳ���
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER:
******************************************/
/*void INICIAL()
{	
	CONTROL |= 0x01;							//CONTROL.0=1
	AX1 = 4000;									//AX1 = 4000
	AX1 = AX1 * 1000 / EFC[0];					//AX1 = AX1 * 1000  ( 4000 * 1000 ) �ֳ˷� AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
												//AX1 = AX1 / EFC�ز�Ƶ��(F19 Hz) �̷���AX1  ��������BX1   AX1=011DH(285)
	TZ = AX1;									//�о�TZ Ӧ�����ز������ڣ�����Ϊʲô��4000 000 /14000(14Khz)?
												//�ѵ����ز����� �Ŵ�1000 * 4000 ��?
												//TZ�ز�����S(��),ʵ����WG_RELOAD����������ֵ������Intel 87C196MC����鹫ʽ��ȷ��������ز�����

	BOOST();									//���õ�Ƶ���ز����ӳ��� -->VTRQ
												//VTRQ = [(ת�ز����ٷ��� �Ŵ�1000����)/ 1000 ] * [4000 *1000 / �ز�Ƶ��14000]

	FOM = EFOM[0];								//��װ��,��EEPROM F02��Ӧ�ı������õ��������Ƶ����ֵ��ŵ�FOM�У��Ŵ�100�� 5000 = 50HZ

       // ---------���������ò���--------

	BX1 = ETBK[0];								//BX1 = F17_ETBK  ( F17 ֱ���ƶ�ʱ��  ����2�� �̶�Ϊ2��)
	CX1 = (BX1 * 10000) / 163;					//CX1 = BX1 * 10000  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
												//2 * 10000 =20000����?
												//CX1 = CX1 / 163 why?  CX1 = ��������(122) DX1 = ����������(114)
	RDCCT[0] = CX1;								//RDCCT = CX1  (ֱ���ƶ�ʱ��ο�)
												//RDCCT = ( F17_ETBK(2) * 10000 ) / 163 ����

	BX1 = ESPT[0];								//BX1 = ESPT F44 ��ѧϰ�ƽ�ʱ�� ����2�� [0 - 10]
	CX1 = (BX1 * 10000) / 163;					//CX1 = BX1 * 10000  �Ŵ�10000��  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
												//CX1 = CX1 / 163  CX1 = ��������(122) DX1 = ����������(114)
	RSPT[0] = CX1;								//RSPT = CX1 (��ѧϰʱ��ο�)
												//RSPT = ( F44_ESPT(2) * 10000 ) / 163 ����

	BX1 = EJDLY[0];								//BX1 = EJDLY F57 ǿ�ȹ�����ʱ���ź�ʱ�� ����2�� [0 - 10]
	CX1 = (BX1 * 10000) / 163;					//�Ŵ�10000��  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
												//CX1 = CX1 / 163  CX1 = ��������(122) DX1 = ����������(114)
	RJDLY[0] = CX1;								//RJDLY = CX1 (ǿ�ȹ��ż̵�����ʱ���ź�ʱ��ο�)
												//RJDLY = ( F57_EJDLY(2) * 10000 ) /163 ����


	BX1 = EDOPT[0];								//BX1 = EDOPT F42 �����ƽ�ʱ��(�ص�λ�н�������ʱ�䣬�Ź����ĳ����ƽ�ʱ��)����100�� (������Զ�ƽ�) [0 - 100] ZDK����90��
	CX1 = (BX1 * 10000) / 163;					//�Ŵ�10000��  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
												//CX1 = CX1 / 163  CX1 = ��������(104) DX1 = ����������(8)
	RDOPT[0] = CX1;								//RDOPT = CX1 (�����ƽ�ʱ��ο�) ��ZDK ��������ѱ������ָ�ߵ���
												//RDOPT = (F42_EDOPT(100) * 10000 ) /163 ����

	BX1 = EDCPT[0];								//BX1 = EDCPT F43 �����ƽ�ʱ��(����λ��ֹ������ʱ�䣬
												//�ſ����ĳ����ƽ�ʱ��)����100�� (������Զ�ƽ�) [0 - 100] ZDK����100��
	CX1 = BX1 * 10000 / 163;					//�Ŵ�10000��  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
												//CX1 = CX1 / 163  CX1 = ��������(104) DX1 = ����������(8)
	RDCPT[0] = CX1;								//RDCPT = CX1 (�����ƽ�ʱ��ο�) ��ZDK ��������ѱ������ָ�ߵ���
												//RDCPT = ( F43_EDCPT(100) * 10000 ) /163 ����


	AX1 = EVBK[0];								//AX1 = F18_EVBK (F18 �ƶ�ǿ��  ����2Hz �Ŵ�10�� 20 (�̶���ֵ))
	AX1 = AX1 * TZ / 2000;						//AX1 = AX1 * TZ  �е����⣬TZ����������ѽ AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
												//AX1 = AX1 / 2000  why?  AX1 = �������� BX1 = ����������
	TDC = AX1;									//TDC = AX1 (ֱ���ƶ�SVPWM��T1��T2ʱ����ֵ 1%��TZ)
												//TDC = ( F18_EVBK(20) * TZ ) /2000 ����


	AX1 = TZ * 40960 / 5086;					//AX1 = TZ * 40960 why?  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
												//AX1 = AX1 / 5086 why?  AX1 = �������� BX1 = ����������
	DFINR1 = AX1;								//DFINR1 = AX1 DFINR1��ʲô?
												//DFINR1 = ( 40960 * TZ ) / 5086 ����

	VANDF(); 									//����V/F���Դ����ӳ��� -->KV1

	/*********���㿪�ŵ�DOS01��ֵ***
	* �ڵ�0������λ�ú͵�1������λ��֮��ĵ� (%) ȷ��S������ȷ��
	********************************
	AX1 = DOS1[0];								//AX1 = ���Ż���λ��#1 DOS1(%)
	AX1 = AX1 - DOS0[0];						//AX1 = AX1 - ���Ż���λ��#0 DOS0 (%)
	DOSA[0] = AX1;								//DOSA = AX1 (�е���˼��DOSA=DOS1-DOS0)
	AX1 = AX1 >> 1;								//AX1 = AX1 /2 (DOS1-DOS0)/2 �����Ǽ����һ�͵ڶ�������λ�õ��е�
	AX1 = AX1 + DOS0[0];						//AX1 = AX1 + DOS0 (�ǿ��ſ�ʼ����0�͵�1����λ�����ĵĳ��Ȼ���%)
	DOS01[0] = AX1;								//�㶨DOS01 ���ǵ�һ�͵ڶ����ٵ������%
												//DOS01 (%) = DOS0 + ( DOS1 - DOS0) /2

	/*********���㿪�ŵ�DOS23��ֵ***
	* �ڵ�2������λ�ú͵�3������λ��֮��ĵ� (%) ȷ��S������ȷ��
	********************************
	AX1 = DOS3[0];								//AX1 = ���Ż���λ��#3 DOS3(%)
	AX1 = AX1 - DOS2[0];						//AX1 = AX1 - ���Ż���λ��#2 DOS2
	DOSB[0] = AX1;								//DOSB = AX1 (DOSB=DOS3-DOS2)
	AX1 = AX1 >> 1;								//AX1 = AX1/2  (DOS3-DOS2)/2 ���2�͵�3������λ�õ�����
	AX1 = AX1 + DOS2[0];						//AX1 = AX1 + DOS2
	DOS23[0] = AX1;								//�㶨DOS23 ���ǵڶ��͵������ٵ������%
												//DOS23 (%) = DOS2 + ( DOS3 - DOS2 )/2

	/*********������ŵ�DCS01��ֵ***
	* �ڵ�0������λ�ú͵�1������λ��֮��ĵ� (%) ȷ��S������ȷ��
	********************************
	AX1 = DCS0[0];
	AX1 = AX1 - DCS1[0];
	DCSA[0] = AX1;
	AX1 = AX1 >> 1;
	AX1 = AX1 + DCS1[0];
	DCS01[0] = AX1;								//�㶨DCS01 ���ǹ��ŵ�0�͵�1���ٵ������%
												//DCS01 (%) = DCS1 + ( DCS0 - DCS1 )/2
												//ע����ŵ���㹫ʽ�Ϳ��Ų�ͬ

	/*********������ŵ�DCS23��ֵ***
	* �ڵ�2������λ�ú͵�3������λ��֮��ĵ� (%) ȷ��S������ȷ��
	********************************
	AX1 = DCS2[0];
	AX1 = AX1 - DCS3[0];
	DCSB[0] = AX1;
	AX1 = AX1 >> 1;
	AX1 = AX1 + DCS3[0];
	DCS23[0] = AX1;								//�㶨DCS23 ���ǹ��ŵڶ��͵������ٵ������%
												//DCS23 (%) = DCS3 + ( DCS2 - DCS3 )/2
												//ע����ŵ���㹫ʽ�Ϳ��Ų�ͬ

	/***���㿪��DOVA,DOVB��ֵ***
	* �����ٶ�ƽ������
	********************************
	AX1 = DOV1[0];								//AX1 = DOV1 (���Ż���λ��#1��Ӧ���ٶ�Hz)
	AX1 = AX1 - DOV0[0];						//AX1 = DOV1 -DOV0(���㿪�Ż���λ��#0��#1��Ӧλ�õ��ٶȲ� Hz)
	AX1 = AX1 << 1;								//AX1 = AX1 *2 (DOV1-DOV0)*2
	DOVA[0] = AX1;								//����ƽ�����Բ���DOVA =  (DOV1-DOV0)*2
												//��ζ DOV1 > DOV0 (Hz)

	AX1 = DOV1[0];
	AX1 = AX1 - DOV2[0];
	AX1 = AX1 << 1;
	DOVB[0] = AX1;								//����ƽ�����Բ���DOVB =  (DOV1-DOV2)*2
												//��ζ DOV1 > DOV2 (Hz)

	/***�������DCVA,DCVB��ֵ***
	* �����ٶ�ƽ������
	********************************
	AX1 = DCV1[0];
	AX1 = AX1 - DCV0[0];
	AX1 = AX1 << 1;
	DCVA[0] = AX1;								//����ƽ�����Բ���DCVA =  (DCV1-DCV0)*2
												//��ζ DOV1 > DOV0 (Hz)

	AX1 = DCV1[0];
	AX1 = AX1 - DCV2[0];
	AX1 = AX1 << 1;
	DCVB[0]	= AX1;								//����ƽ�����Բ���DCVB =  (DCV1-DCV2)*2
												//��ζ DOV1 > DOV2 (Hz)

	FIA = 0;									//FIA=#0000H
	NEWVECT = 0;								//NEWVECT=00H
	SECTOR = 1;									//������SECTOR=1
	FGC = #LSP0;								//FGC = LSP0 (�������СƵ�ʵ�16bit) Hz
	FOH = #LSP0;								//FOH = LSP0 (�������СƵ�ʸ�16bit) Hz
	FG = #LSP0;									//FG  = LSP0 (�������СƵ�ʸ�16bit) Hz
	FOL = 0;									//FOL =#0000H
	TAB = 2048;									//TAB = 2048 (SINE��1024���������ݣ����ֽھ���2048)

	DTS	= 0;									//DTS = #0000H
	AX3 = NUMBER;								//AX3 = NUMBER (30000) AX3 ���汾�λ�õı�����������

	CONTROL = 0;								//CONTROL=#00H
	FUN = 0;									//FUN=00

	AX1 = EDEADT[0];							//2011-10 Johnson EDEADT (F23)����ʱ�����Ϊ2 ʵ�ʶ�Ӧ80C196MCΪ1us û�����ⲿIR2130S�Դ���
	AX1 = AX1 << 2;								//����ʱ���Сһ��,����80C196�Ĺ����Tdead=DT/8 ZDK��F23����ֵ�Ŵ�4������ F23/2=��ǰ������ʱ��(us)
	AX1 &= 0x03FF;								//ȡ��10bitӰ������ʱ�� AX1 &= #0000001111111111B;
	BX1 = WG_CON[0];
	BX1 &= 0xFC00;								//������ǰWG_CON�ĸ���λ״̬���ص��ǵ������������ݽ�ȥ BX1 &= #1111110000000000B;	
	BX1 |= AX1;
	WG_CON[0] = BX1;
}*/

/*********************************************************
*             ������ʼ���ӳ���
*IN  PARAMETER: NONE
*OUT PARAMETER: NONE
**********************************************************/
void BEGIN() 
{
	TERMAL = 0xFF;									//2011-10 Johsnon�о��ظ���TERMALͨ�Ų�����־����Ϊ#FFH����ʾ��ǰû�а������룬Ҳû���ⲿ����(AUTO��ᵽ�ⲿ���ƣ�
	DBUF[4] = 0xFF;
	//P2_REG[0] = 0x1C;								//2011-10 Johnson ͬʱ������������Ϊ0��ֻ��SCT/SIN/THR�ø�(��ʼ��LED��λ�Ĵ������ر�PWM���) (��������ʱ��֧��THR)
													
													//F00����(����)Ƶ��->AX1  default EFG =5000 50HZ
													//F02�������Ƶ��->BX1  default EFOM=5000 50HZ  
	if(E2M[EFG] > E2M[EFOM]) {						//��ǰ�趨Ƶ��F00���������Ƶ����F02תBEGIN1
		E2M[EFG] = E2M[EFOM];						//���F00 �趨Ƶ�� <= F02 �������Ƶ�� ���� �趨Ƶ��F00=�������Ƶ��
	}
	
	FUN = 0;										//��������Ϊ00H
	RT &= 0x00;										//����һ��ʼRT��־Ϊ00H, 1��Ч

	CONTROL = 0;									//CONTROl=00H
	FOH = 0;										//FOH=0000H
	FOL = 0;										//FOL=0000H
	DCCT = 0;										//DCCT=0000H ������ֱ���ƶ�ʱ��

	DSCD = E2M[EDSCD];								//�ѵ�ǰ��ʾģʽF45 EDSCD����DSCD 0=��ʾ�趨Ƶ�ʣ�1=��ʾ���Ƶ�ʣ�2=��ʾλ�ðٷ�����3=��ʾλ��������
	AL0 = 0xFF;										//AL0��־��ΪFFH
	SMT = 0;										//��SMT��־
}

/************************************************************
*        Ƶ�������Ԥ����--��ѧϰ����Ӽ��ٴ����ӳ���
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: INCFOL, INCFOH, DECFOL, DECFOH
*      �о�Ӧ�����ٶȣ�����Ϊʲô��ʱ��?
*************************************************************/
/*void ADCL0()
{
	CX1 = EACL0[0];									//CX1 = EACL0 (F05 ��ѧϰ�������ʱ��s) ����20s �Ŵ�10�� 200
	BX1 = 0;
	AX1 = 4096;									//ΪʲôҪ��4096?
	AX1 = AX1 / CX1;								//˫�ֳ���  AX1 = AX1 / CX1 ( 4096 / 200 = 20.����96 ) ����AX1(20) ������BX1(96)

	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  =20 * TZ(285)= 5700(1644H) BX1(0000H)  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
					
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 ����2λ�Ŵ�4�� 5700*4=22800(5910H) 18/1-2000
													//AX1 = [((4096 / F05_EACL0) * TZ )* 4] ��16bit
													//BX1 = [((4096 / F05_EACL0) * TZ )* 4] / 65536
	INCFOL = AX1;									//INCFOL = AX1  (5910H)(22800)
	INCFOH = BX1;									//INCFOH = BX1  (0000H)
   /***2011-10����Ϊʲô��H��L?**********
   *ԭ��L����32λ�˷��ĵ�16λ���
   *    H����32λ�˷��ĸ�16Ϊ���
   *F05_EACL0 =[1 - 999]���淢���������INCFOH=0000H
   *INC(DEC) FOL(FOH)������ʲô��˼ ?
   **************************************
	CX1 = EDCL0[0];									//CX1 = EDCL0 (F06 ��ѧϰ�������ʱ��s) ����10s �Ŵ�10�� 100
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 (4096 / 100=4. ����96) ����AX1(4) ������BX1(96)
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ = 4 * 285 = 11400(2C88H),BX1�ų˻���16λ(������0) AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	AX1 = AX1 << 2;								//AX1 = AX1 *4  ����2λ�Ŵ�4��  18/1-2000
	DECFOL = AX1;									//DECFOL = AX1 = [((4096 / F06_EDCL0) * TZ ) * 4] ��16bit  (0B220H) 45600
	DECFOH = BX1;									//DECFOH = BX1 = [((4096 / F06_EACL0) * TZ )* 4] / 65536   (0000H)
}*/

/************************************************************
*	Ƶ�������Ԥ����--��ת�Ӽ��ٴ����ӳ���
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: INCFOL, INCFOH, DECFOL, DECFOH
*    �о�Ӧ�����ٶȣ�����Ϊʲô��ʱ��?
*************************************************************/
/*void ADCL1()
{
	CX1 = EACL1[0];									//CX1 = EACL1 (F07 �������ʱ��s) ����30s �Ŵ�10�� 300
	BX1 = 0;
	AX1 = 4096;									//ΪʲôҪ��4096?
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 300 = 13 ��196) AX1 = �������� BX1 = ����������
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 ����2λ�Ŵ�4��  18/1-2000
	INCFOL = AX1;									//INCFOL = AX1 = [((4096 / F07_EACL1) * TZ ) * 4] ��16bit  (39E4H) 14820
	INCFOH = BX1;									//INCFOH = BX1 = [((4096 / F07_EACL1) * TZ )* 4] / 65536   (0000H)

	CX1 = EDCL1[0];									//CX1 = EDCL1 (F08 �������ʱ��s) ����10s �Ŵ�10�� 100
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 100 = 4 ��96) AX1 = �������� BX1 = ����������
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 ����2λ�Ŵ�4�� 18/1-2000
	DECFOL = AX1;									//DECFOL = AX1 = [((4096 / F08_EDCL1) * TZ ) * 4] ��16bit   (0B220H) (45600)
	DECFOH = BX1;									//DECFOH = BX1 = [((4096 / F08_EDCL1) * TZ )* 4] / 65536    (0000H)
}*/

/************************************************************
*	Ƶ�������Ԥ����--����Ӽ��ٴ����ӳ���
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: INCFOL, INCFOH, DECFOL, DECFOH
*   �о�Ӧ�����ٶȣ�����Ϊʲô��ʱ��?
*************************************************************/
/*void ADCL2()
{
	CX1 = EACL2[0];									//CX1 = EACL2 (F09 �������ʱ��s ) ����10s �Ŵ�10�� 100
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 100 = 4 ��96 ) AX1 = �������� BX1 = ����������
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	AX1 = AX1 << 2;								//AX1 = AX1 * 4 ����2λ�Ŵ�4�� 18/1-2000
	INCFOL = AX1;									//INCFOL = AX1 = [((4096 / F09_EACL2) * TZ ) * 4] ��16bit (0B220H) 45600
	INCFOH = BX1;									//INCFOH = BX1 = [((4096 / F09_EACL1) * TZ )* 4] / 65536   (0000H)

	CX1 = EDCL2[0];									//CX1 = EACL2 (F10 �������ʱ��s ) ����15s �Ŵ�10�� 150
	BX1 = 0;
	AX1 = 4096;
	AX1 = AX1 / CX1;								//AX1 = AX1 / CX1 ( 4096 / 150 = 27��46 )  AX1 = �������� BX1 = ����������
	AX1 = AX1 * TZ;									//AX1 = AX1 * TZ  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	AX1 = AX1 << 2;                 				//AX1 = AX1 * 4 ����2λ�Ŵ�4�� 18/1-2000
	DECFOL = AX1;									//DECFOL = AX1 = [((4096 / F10_EDCL2) * TZ ) * 4] ��16bit   (783CH) 30780
	DECFOH = BX1;									//DECFOH = BX1 = [((4096 / F10_EDCL2) * TZ )* 4] / 65536     (0000H)
}*/

/******************************************
*         ������ֵ����ת���ӳ���
*IN  PARAMETER: AL1
*MID PARAMETER:
*OUT PARAMETER: AL1
*2011-10 Johnson �����ֻ֧��8����
*******************************************/
void KEY138()
{
	AL1 |= 0xF0;         						//ֻȡ����λ ENABLE B0-B3
	
	if(AL1 == 0xFF) {							//��û�м�����?
		AL1 = 0xFF;								//��ʵ�ʼ�ֵת���#1111 1111B �޼�����
	} else if(AL1 == 0xFE) {					//���ż�?
		AL1 = 0xFE;								//��ʵ�ʼ�ֵת���#1111 1110B ���ż�
	} else if(AL1 == 0xFD) {					//���ż�?
		AL1 = 0xFD;								//��ʵ�ʼ�ֵת���#1111 1101B ���ż�
	} else if(AL1 == 0xFC) {					//ǿ�ȹؼ�? ��������֧�������
		AL1 = 0xFB;								//��ʵ�ʼ�ֵת���#1111 1011B ǿ�ȹؼ�? ��������֧�������
	} else if(AL1 == 0xFB) {					//ֹͣ��?
		AL1 = 0xF7;								//��ʵ�ʼ�ֵת���#1111 0111B ֹͣ��
	} else if(AL1 == 0xFA) {					//DOWN��?
		AL1 = 0xEF;								//��ʵ�ʼ�ֵת���#1110 1111B DOWM��
	} else if(AL1 == 0xF9) {					//UP��?
		AL1 = 0xDF;								//��ʵ�ʼ�ֵת���#1101 1111B UP��
	} else if(AL1 == 0xF8) {					//���ܼ�?
		AL1 = 0xBF;								//��ʵ�ʼ�ֵת���#1011 1111B ���ܼ�
	} else if(AL1 == 0xF7) {					//д���?
		AL1 = 0x7F;								//��ʵ�ʼ�ֵת���#0111 1111B д���
	} else if(AL1 == 0xF6) {					//�����λ��?
		AL1 = 0XF3;								//��ʵ�ʼ�ֵת���#1111 0011B �����λ��
	} else if(AL1 == 0xF5) {					//ֹͣ+ǿ�ȹ���+����+���ż�?��������֧�������
		AL1 = 0xF0;								//��ʵ�ʼ�ֵת���#1111 0000B ֹͣ+ǿ�ȹ���+����+���ż�?��������֧�������
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


/*------[  BEGIN FROM '1' ��ʼ״̬����]--------*/
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
*           ����EEPROMд���ӳ���
*IN  PARAMETER: AX9(��ַ),AL1(����)
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
*��ڲ���:AL9(8bit��ַ)
*���ڲ���:AX1(16bit)
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
*   �״�������Ҫ�ѹ��������д���EEPROM�ӳ���
*   �������ܣ��ж��Ƿ�Ҫ�ָ��������ã�����ж��Ƿ�����Ȩ����
*   ����Ȩ������ȴ�����д��������д���������
*	�������ʣ��������Ȩ���룬���޴�������޷���E2PROM���в���
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER:
*===============================================*/
void INE2M()
{
	AX9 = XSAV;                       		//��װ��, XSAV=028H,��AX9=028H (��ʼ��ȡF20����ֵ����)
	AX1 = ERDE(AX9);						//����EEPROM������,��ȡ����ֵ�����AX1��
	
	if(AX1 != 888) {                        //888Ϊ�ָ��������ñ�־  6080 as DEFAULT PARAMETER RESTORE LABEL
		AX9 = XPWD;							//XPWD=C8H(200),AT93C66������   ��EEPROM C8H�������ֵ�ǲ���3FH?
		AX1 = ERDE(AX9);
		if(AX1 != PASS) {					//��������ֵ����AX1��,��������ĵ��ֽ�-->(AL1)
		//PASS=3FH(63),��AL1��������3FH�����ֽڱȽϿ��ǲ������(���EEPROM�Ƿ�����Ȩ��)�����������ת
		//���û�з�����Ȩ����,��תEEPROM��������д�봦�����
			KEYB();				//
			while(KEY != 0x7F) {			//����ǰ����״̬�ӳ���
				KEYB();						//������Ǿͼ�������ֵ,��ζ���EEPROMû����Ȩ����,�û�������д���,��һֱѭ���ȴ�!	
			}
		} else {
			return ;
		}	
	}
	
	CX1 = 0;						//�����д���,��ʼ����д��EEPROM������������
	DX1 = 0;
	BX1 = 0;
	DBUF[0] = 0x61;
	DBUF[1] = 0x99;					//E
	DBUF[2] = 0x03;					//4  RETORE FACTORY CODE OK LABEL->E200 #25H
	DBUF[3] = 0x03;					//0 
	DBUF[4] = 0xFF;					//0
	

	DISPLY1();						//��ʾE400(�ָ�����������־) ֱ��(ǿ��RT.2=0)��������ݷ�����ʾ�����ӳ�?
	BX1 = 100;						//��ʾ����Լ��ʱ�� 100xDelay TIME
	
	while(BX1 -- != 0) {			//��ʾE400(�ָ�����������־)
		DELAY();
		DISPLY1();					//ֱ��(ǿ��RT.2=0)��������ݷ�����ʾ�����ӳ���
	}
	
	AX9 = 0;                      	//��AX9,׼���ָ��������ò�����Ϣ
	
	while(AX9 != 159) {				//159?(160��) ˫�ֽھ���80������ (ʵ�ʿ�X8401����Ч��Ŀ��ȷ��160) �Ѳ���SINE��Ҳд��ȥ��
		AX1 = FACTRY[AX9];              //��ʼ�ָ�������Ϣ, �ѳ�������װ��AX1��(ƫ�Ƶ�ַ��AX9)
		E2M[AX9] = AX1;                 //���Ȱ�RAM��Ӧ��������ֵҲͬʱ�ָ��ɳ���������ֵ
	
		EWRE(AX1, AX9);					//����EEPROMд���?
	
		//DLY = 2;                     	//Add for AT93C66
		DELAY();                       	//����Ч��λ�ɳ�������
		AX9 ++;				 			//AX9=AX9+1 =��һ����Ԫ�ĵ�ַ
										//2011-10 Johnson �о��ٴμ�1�Ͷ��࣬��Ҳ��ʵ��EERPOM���ݳ��ֵ�ַ�����ԭ��ZDK HD508.ASMû����䣬���Կ���ȥ�����
	}

	AX1 = PASS;                         //����3FH(63)��Ȩ��д��EERPOM��(C8H)��ַ
	AX9 = XPWD;                        	//����д�����
	EWRE(AX1, AX9);				 		//����EEPROMд�����
	//DLY = 2;                         	//Add for AT93C66
	DELAY();                           //ʵ�ʱ������ȡAT93C66����ʵ���ݷ��ֵ�ַ�б�ͱ������в�𣬲�Ӱ��ʹ��
	
	return ;
}



/********************************************************
*         ��EEPROM��ȡ��������ֵ���浽��ӦRAM��
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
*  ��ָ�������������EEPROM ȡ��װ���ӦRAM��Ԫ�ӳ���
*IN  PARAMETER: FUN,AX9(��ǰ������)
*MID PARAMETER��AX9,AH1,AL1
*OUT PARAMETER: AX8(��Ӧ�������EEPROM���ò���)
*******************************************************/
void GETS()
{

}
/*GETS:	 CMP    FUN,#0
	 JNE    GETS1				;�����������F00�򷵻�
	 RET
GETS1:	 ADD    AX9,FUN,FUN			;AX9 = ��ǰ���ܶ�Ӧ��EEPROM�����ַ
	 INC    AX9
	 LCALL  ERDE
	 LDB    AH1,AL1
	 DEC    AX9
	 LCALL  ERDE
	 ST     AX1,E2M[AX9]			;��ȡEEPROM���浱ǰ����������ò��������浽�ڴ���ص�Ԫ����
	 ST     AX1,AX8
	 RET*/

/******************************************************************
*           ���λ���������ӳ��� (λ�ò���)
*IN  PARAMETER:	TIMER1	Ӧ���ǻ��TIMER1����ֵ
*OUT PARAMETER: TIMER1
*******************************************************************/
/*void GETPOS()
{
	NUMBER = (unsigned int)EQep1Regs.QPOSCNT / 4;		//��ȡ��ǰ������������
	if(NUMBER <= 0) {
		NUMBER = 0;
		//����������������0
	}
	
	if(SD_FLG != 2) {			//��ѧϰû�н���
		if(NUMBER > NMAX) {
			NUMBER = NMAX; 		//��ǰ������������Ϊ���������
			//�ѱ����������������Ϊ���������
		}
	} else {					//��ѧϰ����
		if(NUMBER >= EPULSE[0]) {
			NUMBER = EPULSE[0];
			///����������ΪEPULSE[0]
		}
	}
}*/


/*************************************************************
*    (ֻ�����ϵ����������ʾģʽ)������ӳ��� COMMAND ROUTINE
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER: COMD
*		TERMAL =# 1111 IN4 IN3 AUTO(IN2) GND(IN1)
*         ����Ч!       IN4:	       ����
*			IN3:           ����
*			IN2(��ΪAUTO): ���� GBV10Ӳ������AUTO��
*			IN1(��ΪGND):  ֹͣ GBV10Ӳ������GND�ˣ���ԶʧЧ
*         TERMAL�ĵ���λ�Ǿ�������ȡ���ģ�����Ч������Ч
*	COMD��־�����ֵ�ǰ�Ǽ��̻��ǵ㶯�����ⲿ���ӣ�ͳһ��COMD�ĵ���λ
**************************************************************/
/*void COMMAND()
{
	AL1 = APPL[0];									//��F58��AL1  (�ɵ�Ƶ��ģʽ 1->�ϵ���ʾ���������Ŀ,0->�ϵ���ʾ���Ƶ��ֵ)
	if(AL1 & 0x01 == 1) {							//����� �ϵ���ʾ���������Ŀ ģʽ ����תCOMMD1  AL1.bit0 == 1
		AL1 = ECMD[0];								//AL1 = F11 (����ģʽ) 0->���̿��� 1-> �㶯  2->�ⲿ����
		if(AL1 == 2) {								//����ģʽ��2 �ⲿ���ӿ���ģʽ 
			if(TERMAL & 0x02 == 0) {                //�ⲿ���ӿ���ģʽ��TERMAL.1=1? (HAD �����ƣ�ʵ�ʻ���AUTO��)  ����IN1(���� ��֧��) ����ERLY2    NRDING
				COMD = 0xFD;						//COMD = # 1111 1101B ����ǿ�ȹ���
			} else if(TERMAL & 0x04 == 0) {         //���ǿ���IN3 ����ERLY33   OPEN
				COMD = 0xFB;						//COMD = # 1111 1011B ���ӿ���
			} else if(TERMAL & 0x01 == 0) {       	//���Ƕ���ֹͣ(GDV10��֧��)����ERLY3  STOP
				COMD = 0xFE;						//COMD = # 1111 1110B ����ֹͣ
			} else if(TERMAL & 0x08 == 0) {         //���Ƕ��ӹ�������ERLY4 CLOSE
				COMD = 0xF7;						//COMD = # 1111 0111B ���ӹ���
			} else {
				COMD = 0xFF;						//û�ж����źţ�����COMD = # 1111 1111B
			}
		} else if(AL1 == 1){						//����ģʽ��1 ���̵㶯���������֧��
			if(KEY & 0x01 == 0) {					//���Ǽ��̿��ż�����KJOG1
				COMD = 0xFB;						//COMD = # 1111 1011B ���̿���
			} else if(KEY & 0x02 == 0) {			//���Ǽ��̹��ż�����KJOG2
				COMD = 0xF7;						//COMD = # 1111 0111B ���̹���
			} else if(KEY & 0x04 == 0) {			//���Ǽ���ǿ�ȹ�������KJOG3
				COMD = 0xFD;						//COMD = # 1111 1101B ����ǿ�ȹ��� (GDV10û�������)
			} else if(KEY & 0x08 == 0) {			//���Ǽ���ֹͣ������KJOG4
				COMD = 0xFE;						//COMD = # 1111 1110B ����ֹͣ
			} else {
				COMD = 0xFF;						//û����ذ�������COMD = # 1111 1111B
			}
		} else if(AL1 == 0) {						//����ģʽ��0 ����ģʽ
			if(KEY & 0x04 == 0) {                   //���Ǽ���ǿ�ȹ��ż�����KBTM2	NRDING
				COMD = 0xFD;						//COMD = # 1111 1101B ����ǿ�ȹ��� (GDV10û�������)
			} else if(KEY & 0x01 == 0) {            //OPEN
				COMD = 0xFB;						//COMD = # 1111 1011B ���̿���
			} else if(KEY & 0x08 == 0) {            //STOP
				COMD = 0xFE;						//COMD = # 1111 1110B ����ֹͣ
			} else if(KEY & 0x02 == 0) {            //CLOSE
				COMD = 0xF7;					//COMD = # 1111 1110B ���̹���
			}
		}
	}// RET					;����� �ϵ���ʾ���Ƶ����ֵ ģʽ �򷵻�
}*/


/****************************************************
*           ֱ���ƶ������ӳ���
*IN  PARAMETER: F16(EEBK) LSP
*MID PARAMETER:
*OUT PARAMETER: RT
*****************************************************/
/*void DC_BK()
{
	AX1 = EFBK[0];								//AX1 = F16_EFBK (F16 ֱ���ƶ���ʼƵ�� �Ŵ�100��������45=0.45Hz)
	if(AX1 > #LSP) {
		FG = EFBK[0];							//FG = EFBK (F16 ֱ���ƶ�Ƶ�� �Ŵ�100��������45=0.45Hz)
		if(FOH <= EFBK[0]) {					//�Ƚ�FOH �� EFBK
			CONTROL |= 0x04;					//FOH <= EFBK ����CONTROL.2=1  CONTROL |= #00000100B;
			if(DCCT > RDCCT[0]) {				//�Ƚ�DCCT �� RDCCT? ֱ���ƶ�ʱ�� �� ֱ���ƶ�ʱ��ο��Ƚ�
				RT &= 0xFE;				//RT.0=0  RT &= #11111110B;
			}
		}
	} else {
		FG = #LSP;								//FG = LSP (������СƵ�ʸ�16bit HZ)
		
		if(FOH <= #LSP) {						//FOH��LSP�Ƚ�(������СƵ�ʸ�16bit Hz)
			RT &= 0xFE;					//RT.0=0  ��ֹͣ?  RT &= #11111110B;
		}
	}
}*/

/***************************************************
*        �̵�������ӳ���
*IN  PARAMETER: S  ��ǰ�ſ�λ�ðٷ���
*         HAD  �̵������ù�ϵ��
*		OUT1 = ����     �̵��� InRush Relay
*		OUT2 = �ſ���   �̵���
*		OUT3 = �Ź���   �̵���
*		OUT4 = ָ��λ�� �̵���
*��ѧϰ��ɺ���F41����ٷ���λ��ʱ�̵���ʹ�ܣ��Ͽ���е��ȫ����
*OUT PARAMETER: (RT)
****************************************************/
/*void J123()
{
	AX1 = S - EOPL[0];                						//AX1 = S - EOPL(F53 �ſ���̵����ź� 95% �Ŵ�100�� 950)  OPEN LOCATION
	
	if(S <= EOPL[0]) {
		AL1 = P2_REG[0];									//S < EOPL ��ҪOUT2�̵���ʧЧ
		AL1 &= 0xBF;               							//CLR 95%  OUT2 �̵���ʧЧ (�ſ��� ʧЧ) AL1 &= #10111111B;
		P2_REG[0] = AL1;
		AX1 = S - EPOS[0];                					//AX1 = S - EPOS(F41 λ�ü̵����趨30% �Ŵ�100�� 300��  OPEN OPS.
		if(S >= EPOS[0]) {
			AX1 = WG_OUT[0];								//��� S > EPOS ��׼��OUT4�̵���ʧЧ (ָ��λ�� ʧЧ)
			WG_OUT[0] = AX1;
		} else {
			AX1 = WG_OUT[0];
			AX1 |= 0x0080;									//��OUT4�̵��� (ָ��λ�� ��Ч) AX1 |= #0000000010000000B;
			WG_OUT[0] = AX1;
		}
		AX1 = S - ECLL[0];                					//AX1 = S - ECLL(F54 �Ź���̵����ź� 5% �Ŵ�100�� 50)  CLOSE LOCATION
		if(S >= ECLL[0]) {
			AL1 = P2_REG[0];								//S > ECLL ��׼����OUT3�̵���ʧЧ (�Ź��� ʧЧ)
			AL1 &= 0x7F;		     						//OUT3�̵���ʧЧ (�Ź��� ʧЧ) AL1 &= #01111111B;
			AL1 = P2_REG[0];
			RT &= 0xF7;										//�ر���ѧϰ�ƽ�?
		} else {
			if(COMD.bit1 == 0) {							//����ǿ�ȹ�����ת ON0B
				RT |= 0x08;									//��ǰ��ǿ�ȹ������� RT.3=1 �̵�����ʱ���ź�?	RT |= #00001000B;
				if(JDLY > RJDLY[0]) {						//�Ƚ�JDLY �� RJDLY(�̵�����ʱ�ο�)
					AL1 = P2_REG[0];
					AL1 |= 0x80;		      				//��OUT3�̵��� (�Ź��� ��Ч) AL1 |= #10000000B;	
					P2_REG[0] = AL1;
				}
			} else {
				AL1 = P2_REG[0];
				AL1 |= 0x80;		      					//��OUT3�̵��� (�Ź��� ��Ч) AL1 |= #10000000B;
				P2_REG[0] = AL1;
			}
		}
	} else {
		AL1 = P2_REG[0];
		AL1 |= 0x40;										//AL1 |= #01000000B;
	 	AL1 &= 0x70;	             						//(2003-5-13)  AL1 &= #01111111B;
		P2_REG[0] = AL1;		     						//OUT3�̵���ʧЧ (�Ź��� ʧЧ)

		AX1 = WG_OUT[0];
		AX1 &= 0xFF7F;							//OUT4 �̵���ʧЧ (ָ��λ�� ʧЧ) AX1 &= #1111111101111111B;
		WG_OUT[0] = AX1;
	}
}*/


/********************************************************
*          ���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���  TRQUE CHECK
*IN  PARAMETER: S (��ǰ��λ�ðٷֱȣ��Ŵ�1000��)
*MID PARAMETER:
*OUT PARAMETER: FG (��ȷ��Ƶ��Hz)
* �öγ�������ϸ���ֲ�ͬ��λ�������������ͬ�׶ε��ٶ�
* Ŀ���ǳ���S�� �󻬵��ٶ����ߣ����Ż��������ȶ�
*********************************************************/
/*void OPEN()
{
	if(S <= DOS0[0]) {									//�Ƚϵ�ǰ��λ��S�Ϳ��Ż���λ��#0 F25 (%,�Ŵ�1000��)
		FG = DOV0[0];									//��ǰ���� < DOS0���䣬 ����FG  =���Ż��ٵ�#0���ٶ� F33_DOV0(400) (Hz,�Ŵ�100���ı�ʾ)
	} else if(S <= DOS01[0]) {							//�����Ƚϵ�ǰλ��S�Ϳ��Ż���λ��#01 (DOS01) �Ƚ�{
														//��ǰ����DOS0 �� DOS01֮�䰴���´���:
		AX1 = S - DOS0[0];								//AX1 = S - ���Ż���λ��#0 (F25_DOS0)
		CX1 = AX1 * DOVA[0];							//CX1 = AX1 * ����ƽ�����Բ���DOVA  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
		CX1 = CX1 / DOSA[0];             				//CX1 = CX1 / ����ƽ�����Բ���DOSA  CX1 = �������� DX1 = ����������

		CX1 = CX1 * AX1;								//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
		CX1 = CX1 / DOSA[0];							//CX1 = CX1 / ����ƽ�����Բ���DOSA CX1 = �������� DX1 = ����������
		FG = CX1 + DOV0[0];								//FG(��ǰ�������?) =CX1 + ���Ż��ٵ�#0�ٶ� F33(Hz,�Ŵ�100���ı�ʾ)
														//FG = F25_DOV0 +[(S - DOS0)^2 * (DOV1 - DOV0) *2] / (DOS1 - DOS0 )^2
	} else {
		AX1 = S - DOS1[0];								//AX1 = S -���Ż���λ��#1 F26 (%,�Ŵ�1000��)
		if(S <= DOS1[0]) {
			AX1 = -AX1;									//��AX1 ÿλȡ�� +1���ı�������ķ��ţ�����ֵ����
			CX1 = AX1 * DOVA[0];						//CX1 = CX1 * ����ƽ�����Բ���DOVA  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
			CX1 = CX1 / DOSA[0];						//CX1 = CX1 / ����ƽ�����Բ���DOSA  CX1 = �������� DX1 = ����������

			CX1 = CX1 * AX1;							//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
			CX1 = CX1 / DOSA[0];						//CX1 = CX1 / ����ƽ�����Բ�ʳDOSA CX1 = �������� DX1 = ����������
			AX1 = DOV1[0];								//AX1 = AX1 + ���Ż��ٵ�#1���ٶ� F34(Hz,�Ŵ�100���ı�ʾ)
			FG = AX1 - CX1;								//FG  = DOV1 -[(|S - DOS1|^2 * (DOV1 - DOV0) *2] / (DOS1 - DOS0)^2
		} else if(S <= DOS2[0]) {
														//S > ���Ż���λ��#2 F27(%),����תOPEN4
														//��ǰ����DOS1 - DOS2֮�䰴���´���:
			FG = DOV1[0];								//����FG = ���Ż��ٵ�#1���ٶ� F34(Hz,�Ŵ�100���ı�ʾ)
		} else if(S <= DOS23[0]) {						//�Ƚϵ�ǰ��λ�úͿ��Ż���λ��#23
														//��ǰ����DOS2 - DOS23֮�䰴���´���
			AX1 = S - DOS2[0];							//AX1 = S - ���Ż���λ��#2 F27(%)
			CX1 = AX1 * DOVB[0];						//CX1 = CX1 * ����ƽ�����Բ���DOVB ��DOS23�׶�����DOVB���о��Ǿ������ٶȡ�
			CX1 = CX1 / DOSB[0];						//CX1 = CX1 / ����ƽ�����Բ���DOSB CX1 = �������� DX1 = ����������

			CX1 = CX1 * AX1;							//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
			CX1 = CX1 / DOSB[0];						//CX1 = CX1 / ����ƽ�����Բ���DOSB   CX1 = �������� DX1 = ����������
			AX1 = DOV1[0];								//AX1 = ���Ż��ٵ�#1���ٶ� F34(Hz,�Ŵ�100���ı�ʾ)
			FG = AX1 - CX1;								//FG  = AX1 - CX1
		} else {
			AX1 = S - DOS3[0];							//AX1 = S - ���Ż���λ��#3 F28(%,�Ŵ�1000��)
			if(S <= DOS3[0]) {
				AX1 = -AX1;								//��AX1 ÿλȡ�� +1���ı�������ķ��ţ�����ֵ����
				CX1 = AX1 * DOVB[0];					//CX1 = CX1 * ����ƽ�����Բ���DOVB  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
				CX1 = CX1 / DOSB[0];					//CX1 = CX1 / ����ƽ�����Բ���DOSB CX1 = �������� DX1 = ����������

				CX1 = CX1 / AX1;						//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
				CX1 = CX1 / DOSB[0];					//CX1 = CX1 / ����ƽ�����Բ���DOSB CX1 = �������� DX1 = ����������
				FG = CX1 + DOV2[0];						//FG  = CX1 + ���Ż��ٵ�#2���ٶ� F35(Hz,�Ŵ�100���ı�ʾ)
			} else {
				FG = DOV2[0];							//��ǰ����DOS3 ֮����FG  = ���Ż��ٵ�#2���ٶ� F35(Hz,�Ŵ�100���ı�ʾ)
			}
		}
	}
}*/

	 
/********************************************************
*     ���Ž׶�λ�ö�Ӧ�ٶȼ����ӳ���  TRQUE CHECK
*IN  PARAMETER: S (��ǰ��λ�ðٷֱȣ��Ŵ�1000��)
*MID PARAMETER:
*OUT PARAMETER: FG (��ȷ��Ƶ��Hz)
* �öγ�������ϸ���ֲ�ͬ��λ�������������ͬ�׶ε��ٶ�
* Ŀ���ǳ���S�� �󻬵��ٶ����ߣ����Ż��������ȶ�
*********************************************************/
/*void CLOSE()
{
	if(S <= DCS0[0]) {												//�Ƚϵ�ǰ��λ��S �͹��Ż���λ��#0 F29_DCS0(980)(%,�Ŵ�1000��) 98%
		if(S <= DCS01[0]) {											//�Ƚϵ�ǰ��λ�ú͹��Ż���λ��#1 F30(%,�Ŵ�1000��)
			AX1 = S - DCS1[0];										//AX1 = S - ���Ż���λ��#1 F30 (%,�Ŵ�1000��)
			
			if(S >= DCS1[0]) {
				CX1 = AX1 * DCVA[0];								//CX1 = CX1 * ����ƽ�����Բ���DCVA  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
				CX1 = CX1 / DCSA[0];								//CX1 = CX1 / ����ƽ�����Բ���DCSA CX1 = �������� DX1 = ����������
				CX1 = CX1 * AX1;									//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
				CX1 = CX1 / DCSA[0];								//CX1 = CX1 / ����ƽ������DCSA CX1 = �������� DX1 = ����������
				AX1 = DCV1[0];										//AX1 = ���Ż���λ��#1���ٶ� F38 (Hz,�Ŵ�100��)
				FG = AX1 - CX1;										//FG  = AX1 - CX1
			} else {
				if(S > DCS2[0]) {									//�Ƚϵ�ǰ��λ��S�͹��Ż���λ��#2 F31(%,�Ŵ�1000��)
					FG = DCV1[0];									//FG = ���Ż���λ��#1���ٶ� F38 (Hz,�Ŵ�100��)
				} else {
					if(S > DCS23[0]) {								//�Ƚϵ�ǰ��λ�ú͹��Ż���λ��#23
						AX1 = S - DCS2[0];							//AX1 = S - ���Ż���λ��#2 F31(%,�Ŵ�1000��)
						AX1 = -AX1;									//��AX1 ÿλȡ�� +1���ı�������ķ��ţ�����ֵ����
						CX1 = AX1 * DCVB[0];						//CX1 = AX1 - ����ƽ�����Բ���DCVB �����ٿ�ʼ�� CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
						CX1 = CX1 / DCSB[0];						//CX1 = CX1 - ����ƽ�����Բ���DCSB CX1 = �������� DX1 = ����������
						CX1 = CX1 * AX1;							//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
						CX1 = CX1 / DCSB[0];						//CX1 = CX1 / ����ƽ�����Բ���DCSB CX1 = �������� DX1 = ����������
						AX1 = DCV1[0];								//AX1 = ���Ż���λ��#1���ٶ� F38 (Hz,�Ŵ�100��)
						FG = AX1 - CX1;								//FG  = AX1 - CX1
					} else {
						if(S > DCS3[0]) {							//AX1 = S - ���Ż���λ��#3 F32(%,�Ŵ�1000��)
							CX1 = AX1 * DCVB[0];					//CX1 = AX1 * ����ƽ�����Բ���DCVB  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
							CX1 = CX1 / DCSB[0];					//CX1 = CX1 / ����ƽ�����Բ���DCSB CX1 = �������� DX1 = ����������
							CX1 = CX1 * AX1;						//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
							CX1 = CX1 / DCSB[0];					//CX1 = CX1 / ����ƽ�����Բ���DCSB CX1 = �������� DX1 = ����������
							FG = CX1 + DCV2[0];						//FG  = CX1 + ���Ż���λ��#2���ٶ� F39 (Hz,�Ŵ�100��)
						} else {
							FG = DCV2[0];							//��ǰ���� < DCS3 λ�ã� ��FG  = ���Ż���λ��#2���ٶ� F39 (Hz,�Ŵ�100��)
						}
					}
				}
			}
			
		} else {													//S<= ���Ż���λ��#1 F30(%),�����CLOSE2
																	//��ǰ����DCS01 - DCS0֮�䰴���´���:
			AX1 = S - DCS0[0];										//AX1 = S - ���Ż���λ��#0 F29 (%,�Ŵ�1000��)
			AX1	= -AX1;												//��AX1 ÿλȡ�� +1���ı�������ķ��ţ�ȡ����ֵ
			CX1 = AX1 * DCVA[0];									//CX1 = AX1 * ����ƽ�����Բ���DCVA ��Ӧ���Ǿ����١�CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
			CX1 = CX1 / DCSA[0];									//CX1 = CX1 / ����ƽ�����Բ���DCSA CX1 = �������� DX1 = ����������
			CX1 = CX1 / AX1;										//CX1 = CX1 * AX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
			CX1 = CX1 / DCSA[0];									//CX1 = CX1 / ����ƽ�����Բ���DCSA CX1 = �������� DX1 = ����������
			FG = CX1 + DCV0[0];										//FG  = F37_DCV0 + CX1	
		}
	} else {
		FG = DCV0[0];												//��ǰ���� > DCS0 ���� ����FG = ���Ż���λ��#0���ٶ� F37 (Hz,�Ŵ�100��)
	}
}*/


/********************************************
*         ���ſ���ѧϰ�����ӳ���
*IN  PARAMETER:
*MID PARAMETER:
*OUT PARAMETER:
*********************************************/
/*void STUDY()
{
	 ADCL0();									//Ƶ�������Ԥ����--��ѧϰ����Ӽ��ٴ����ӳ���
	 
	 if(SD_FLG == 0) {							//����ѧϰ��ʼ�׶�?
		//******��ѧϰ��һ����****���ſ���ѧϰ��ʼ�׶�SD_FLG=0***

		if(RT & 0x10 == 0) {	    	        //��ѧϰ�ƽ���Ч  RT.bit4 != 1
			FG = EFG0[0];						//��ѧϰ�ƽ���Ч������FG = EFG0 (��ѧϰ�����ٶ�F46 ����12Hz �Ŵ�100�� 1200) SELF_STUDY CLOSE SPEED
			
			if(FOH == FG && DTS ==0) {			//FOH ������ FG,  ��ת����
												//(�����������ϰ���,DTS=0), FOH = FG,���DTS  ������0 ��ת����
				RT |= 0x10;  	        		//FOH = FG,  �����ϰ��� ����RT.4=1(ZERO is found)
				NUMBER = 0;                 	//�ѵ�ǰ��õı�������������Ϊ0  29/1-2000
				TIMER1[0] = NUMBER;
				S = 0;							//û�������ϰ������ſ�λ�ðٷ����Ĵ���S
			}
		} else {
			FG = EFG1[0];						//RT.0=1, ��FG = EFG1 (F47 ��ѧϰ�ƽ��ٶ�Hz ����3Hz �Ŵ�100�� 300) SELF_STUDY PUSH SPEED
			if(SPT > RSPT[0]) {					//SPT = RSPT ?(��ѧϰʱ��ο�)
				NUMBER = 0;						//�ѵ�ǰ��õı�������������Ϊ0
				TIMER1[0] = NUMBER;
				FSSK = 0;						//���FSSK ��������ڱ�������û���ر����⣬Ӧ����ZDK֮ǰ����CAPTURE��ʽ���������ʱʹ�õ�
				S = 0;							//����ſ�λ�ðٷ����Ĵ���S
				J123();							//���ü̵�������ӳ���
				if(COMD & 0x04 == 0) {          //�ǿ������� COMD.bit2 == 0
					SD_FLG = 1;	        		//�Ź��������һ�������
					RT &= 0xEF;					//�ر���ѧϰ�ƽ�? RT.4=1û���ҵ����?
				}
			}
		}
	 } else {
		/*** ���ſ���ѧϰ�ڶ����� SD_FLG=1 ֱ����� ��ѧϰ SD_FLG=2****
		CL1 = ESDY[0];							//CL1 = ESDY(F56 ��ѧϰ����ģʽ0,1,2������0 ��ѧϰ����������F50 EEPROM��Ԫ 2 ������ 1 ��������)
		if(CL1 != 2) {							//F56 ��ѧϰ����ģʽ����2 ��תSD_B0
			if(MST & 0x01 != 0) {				//���ŷ��� ����תSD_B2   MST.bit0 != 0
				FG = #LSP;						//���ŷ��� ��  FG = LSP (������СƵ�ʸ�16bit HZ)
				if(FOH <= #LSP) {				//FOH��LSP�Ƚ�
					MST &= 0xFE;         		//�ÿ��ŷ���
				}								//FOH > LSP �򷵻�
			} else {
				if(RT & 0x10 == 0) {         	//���ŷ��� RT.4=1? ��תSD_B4 (��ѧϰ�ƽ���Ч?)   RT.bit4 == 0
					FG = EFG0[0];              	//RT.4=0, ����FG = EFG0 (��ѧϰ�����ٶ�F46 ����12Hz �Ŵ�100�� 1200)
					if(FOH == FG && DTS == 0) {
						RT |= 0x10;				//�����ϰ��� ������RT.4=1 (��ѧϰ�ƽ���Ч?) �󷵻�
					}
				} else {
					FG = EFG1[0];				//FG = EFG1 (F47 ��ѧϰ�ƽ��ٶ�Hz ����3Hz �Ŵ�100�� 300)
					
					if(SPT > RSPT[0]) {			//SPT = RSPT ?(��ѧϰʱ��ο�)
						if(NUMBER <= 10) {		//��ǰ��õı����������� �� 10�Ƚ�
							NUMBER = 8;  		//�����ǰ��õı���������<10�����õ�ǰ��õı�����������Ϊ8   NO-PULSE
							TIMER1[0] = NUMBER;
							AX1 = NUMBER;		//AX1 = NUMBER (��ǰ��õı�����������)
						} else {
							CX1 = #NMAX;		//CX1 = ����֧�ֵ����������
							DX1 = ENULL[0];		//DX1 = ENULL (F14 �ſ����%) ����2% �Ŵ�1000�� 2
							AX1 = DX1 * 5;		//AX1 = ENULL(%) * 5  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
							DX1 = CX1 - AX1;	//??? DX1 = ����֧�ֵ���������� - ENULL(%)*5    16/3-2003
	 
							AX1 = NUMBER * DX1;			//AX1 = NUMBER * DX1  ��ǰ�Ļ�õı����������� * DX1  CX1=�˷��ĵ�16bit DX1=�˷��ĸ�16bit
							AX1 = AX1 / CX1;			//AX1 =(NUMBER * DX1) / #NMAX AX1 = �������� BX1 = ����������
							NUMBER = AX1;				//����F14�ſ��������������ø�TIMER1
							TIMER1[0] = NUMBER;
						}
						
						CL1 = ESDY[0];					//CL1 = ESDY(F56 ��ѧϰ����ģʽ0,1,2������0)
						
						if(CL1 == 0) {
							EPULSE[0] = NUMBER;			//F56=0(��ѧϰ����ģʽ)������  �ѵ�ǰ��õı������������ø�F50 EPULSE(��ѧϰ��������)
							AX9 = #XPULSE;				//XPULSE����F50��EEPROM�ĵ�ַ��׼���ѵ�ǰ��õı��������������浽F50��
							EWRE();						//�ѵ�ǰ��ı��������������浽F50 EEPROM��Ԫ��
							DELAY2(2);					//��ʱ2.4 * 2= 4.8ms
							SD_FLG = 2;               	//������ѧϰΪ����״̬  END  SELF-STUDY
							RT &= 0xEF;      		//�ر���ѧϰ�ƽ���־λ  END  AUTO-STUDY PUSH
						} else if(CL1 == 1) {
							CX1 = EPULSE[0];			//CX1 = F50 (������ѧϰ��������,������1000)
							DX1 = NUMBER - CX1;			//DX1 = ��ǰ��õı����������� - EPULSE
							if(NUMBER < CX1) {
								DX1 = -DX1;
							}
							
							if(DX1 > 20) {
								SD_FLG = 0;				//(NUMBER - F50) >  20, ��ǰ��Ϊ��ѧϰ��ʼ�׶�?
								RT &= 0xFE;		//RT.0=0 ֹͣ?
							} else {
								SEPULSE[0] = NUMBER;			//F56=0(��ѧϰ����ģʽ)������  �ѵ�ǰ��õı������������ø�F50 EPULSE(��ѧϰ��������)
								AX9 = #XPULSE;				//XPULSE����F50��EEPROM�ĵ�ַ��׼���ѵ�ǰ��õı��������������浽F50��
								EWRE();						//�ѵ�ǰ��ı��������������浽F50 EEPROM��Ԫ��
								DELAY2(2);					//��ʱ2.4 * 2= 4.8ms
								SD_FLG = 2;               	//������ѧϰΪ����״̬  END  SELF-STUDY
								RT &= 0xEF;      		//�ر���ѧϰ�ƽ���־λ  END  AUTO-STUDY PUSH
							}
						} else {
							SD_FLG = 2;               					//������ѧϰΪ����״̬  END  SELF-STUDY
							RT &= 0xEF;      						//�ر���ѧϰ�ƽ���־λ  END  AUTO-STUDY PUSH
						}	
					} 
				}
			}
		} else {								
			SD_FLG = 2;               					//������ѧϰΪ����״̬  END  SELF-STUDY
			RT &= 0xEF;      						//�ر���ѧϰ�ƽ���־λ  END  AUTO-STUDY PUSH
		}
	 }
}*/





/**************************************************************
*         ���ؼ�ؼ���ӳ���    DOOR TRQUE CHECK
*IN  PARAMETER: S COMD MST RT
*MID PARAMETER: AL1
*OUT PARAMETER: RT
*   ��γ���Ŀ�Ĳ����!
***************************************************************/
/*void OBSTCL()
{
	AL1 = EOBSTCL[0];						//AL1 = ��ǰ���ؼ�ر�־F24 (0��Ч 1��Ч)
	
	//if(AL1.bit0 != 0 && S > 80 && S <= 800 && COMD.bit1 == 1 && MST.bit0 == 1 && DTS != 0) {		//F24=1���ؼ����Ч	
	if(AL1 & 0x01 != 0 && S > 80 && S <= 800 && COMD & 0x02 == 1 && MST & 0x01 == 1 && DTS != 0) {		//F24=1���ؼ����Ч					
	//���AL1.0=1 F24=1���ؼ����Ч�����鵱ǰ�ſ�ٷֱ� 8% < S <= 80%��COMD����ǿ�ȹ������MST���ڹ��ŷ���DTS������û��ͣ��
		RT |= 0x02;					//�����ϰ���  ����RT.1=1  ���ؼ�ر�־��Ч? (���ϰ��������ͣ��)  RT |= #00000010B;

	} else {
		RT &= 0xFD;					//û�������ϰ��� ����RT.1=0  ���ؼ����Ч?  RT &= #11111101B;
	}
}*/

/*************************************************************
*        ��Ƶ���ز����ӳ���  LOW FRQ. TRQ BOOST
*IN  PARAMETER: SD_FLG��MST, TZ
*MID PARAMETER:
*OUT PARAMETER: VTRQ
* �ѿ����ŵĵ�Ƶת�ز������F04_EOTRQ( F52_ECTRQ) * TZ = VTRQ
* ��ʹ�ͳ��V-F���߸о��벻ͬ
* ��ѹ���ת����TZ�ı仯���, TZ�����䣬Ӧ���Ǹı�PWMռ�ձ�
**************************************************************/
/*void BOOST()
{
	if(SD_FLG != 2) {							//��ѧϰû�н���  (��ζ�������ѧϰ�׶�ֻ�����ŵ�Ƶת�ز���)
		CX1 = EOTRQ[0];							//�ѵ�ǰ���ŵ�Ƶת�ز���F04 ����CX1 (%) (�Ŵ�1000��������80 8%)	
	} else {									//�Զ����н׶�Ҫ�����ǿ��ŷ��� ���� ���ŷ��� ����ͬ�� ��Ƶת�ز�������
		if(MST.bit0 == 1) {
			CX1 = ECTRQ[0];						//�Զ����й��ŷ��� CX1 = ���ŵ�Ƶת�ز���F52 (%)(�Ŵ�1000��������80 8%)
		} else {
			CX1 = EOTRQ[0];						//�ѵ�ǰ���ŵ�Ƶת�ز���F04 ����CX1 (%) (�Ŵ�1000��������80 8%)
		}
	}

	AX1 = CX1 * TZ;								//AX1 = CX1 * TZ ( % * S )  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
	AX1 = AX1 / 1000;							//AX1 = AX1 /1000               AX1 = �������� BX1 = ����������
	VTRQ = AX1;									//VTRQ = AX1  �ѵ�VTRQ���뵥λ?
												//VTRQ���� = (F04_EOTRQ(80) * TZ ) /1000 ����
												//VTRQ���� = (F52_ECTRQ(80) * TZ ) /1000 ���� 
}*/

/**************************************************************
*            V/F���Դ����ӳ���  V/F CHARACTISTICS
*IN  PARAMETER: TZ,VTRQ(ת�ز���) SD_FLG
*MID PARAMETER:
*OUT PARAMETER: KV1 V-F����б��*T  (����V-Fϵ��)
*��Ҫע����ǣ����V-F���������ٳ���Ƶ���ز����������б��
*������Կ��ʼƱ��������൱�ڴ�ͳ��V-F����*T(�ز�����)
***************************************************************/
/*void VANDF()
{
	AX1 = TZ - VTRQ;						//AX1 = TZ - VTRQ
	AX1 = AX1 * 16384;						//AX1 = ( TZ - VTRQ ) * 16384  AX1=�˷��ĵ�16bit BX1=�˷��ĸ�16bit
											//16384 = 65536 /4
											
	if(SD_FLG == 2) {						//�����ѧϰ�Ƿ���� ��ѧϰ����, ��ζ���Զ����н׶� ����תVANDF1
											//��ѧϰ�׶ε�V-F���ߴ���ʽ�в�ͬ!
		if(MST.bit0 == 1) {					//MST.0=1 ���ŷ��� 
			CX1 = ECF1M[0];					//CX1 = ��ǰ���Ż���ת��Ƶ��(Hz) F51 (�Ŵ�100��65HZ 6500)
		} else {
			CX1 = EOF1M[0];					//MST.0=0 ���ŷ��� ��CX1 = ��ǰ���Ż���ת��Ƶ��F03 (�Ŵ�100��55HZ 5500)����CX1
		}
	} else {
		CX1 = EF1M0[0];						//CX1 = �ѵ�ǰ��ѧϰ����Ƶ��F49 (Hz ����50Hz �Ŵ�100�� 5000)
	}
	
	AX1 = AX1 / CX1;						//AX1 = AX1 / CV1  AX1 = �������� BX1 = ����������de
	KV1 = AX1;								//KV1 = (( TZ - VTRQ ) * 16384 ) / F51_ECF1M (or F03_EOF1M or F49_EF1M) [���Ǿ����Ŵ�100��]����
}*/

/**************************************************************
*    ���(RT.2=1�ж���Ч)��������ݷ�����ʾ�����ӳ���
*IN  PARAMETER:
*OUT PARAMETER:
***************************************************************/
void DISPLY() 
{
	//if(RT & 0x04 != 0) {
		DISPLY1();										//RT.2=1 ��ת������ʾ���ʹ���
	//}
}

/**************************************************************
*      ֱ��(ǿ��RT.2=0)��������ݷ�����ʾ�����ӳ���
*IN  PARAMETER: DBUF1,DBUF2,DBUF3,DBUF4,DDBUG5
*OUT PARAMETER: QX,QZ,
*OUT PARAMETER: DBUF1,DBUF2,DBUF3,DBUF4,DDBUG5
***************************************************************/
void DISPLY1()
{ 
	char i;
	
	GpioDataRegs.GPASET.bit.GPIO28 = 1;   		//���GPIO28
	
	for(i = 4; i >= 0; i --) {
		PUT8(DBUF[i]);                     		//���÷���8bit�Ӻ���
	}
	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;   	//���GPIO28
}

/**************************************************************
*            ��ʾ����8�������ֽ��ӳ���
*IN  PARAMETER: QZ(��Ҫ��ʾ��BUFF����)
*MID PARAMETER: QX,QY
*OUT PARAMETER:
***************************************************************/
void PUT8(unsigned char data)
{
	unsigned char num;
	unsigned char i;

	num = data;
	
	for(i = 0; i < 8; i ++) {
		GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;				//���͵�ƽ
		delay_loop(1);
		num = data & (1 << i); 								//1����iλ��������iλ��ֵ

		if(num) {    										//�����iλ��1��GPIO30��1�������0������
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
*          VDC MEASUREMENT ֱ��ĸ�ߵ�ѹ����
*IN  PARAMETER: ���ݺ˰´�˵����֧�����뽻��180V - 265V
*MID PARAMETER:
*OUT PARAMETER: VDC  ��ʽ=(1023*VDC)/5(Ϊ�����ο���ѹ)10bitAD�����
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
*          AUTOģʽ�л�����
*IN   PARAMETER: ECMD
*MID  PARAMETER: 
*OUT  PARAMETER: ECMD
*			ECMD=1 �㶯Key_JOG(���������?)
*			ECMD=2 �ⲿ���Ӽ̵�������
*			ECMD=0 ���̿���
*******************************************************/
void AUTO()
{
														//��AUTO����(�߱���ԭ��F11�Ĳ�������,�����F11��Ӧ��RAM��ԪECMD��Ϊ2)
														//P0.5=AUTO
    if(GpioDataRegs.GPADAT.bit.GPIO7 == 1) {           	//���AUTO����û�а���?
		E2M[ECMD] = 0;									//AUTOƽʱΪ�ߣ���ѿ���ģʽECMD����Ϊ0, ���̿���ģʽ
	} else {
		E2M[ECMD] = 2;									//ECMD ( F11 ����ģʽ),ע�Ⲣ����д��F11 ��Ӧ��EEPROM��Ԫ�����粻�ָ�
	}
}




