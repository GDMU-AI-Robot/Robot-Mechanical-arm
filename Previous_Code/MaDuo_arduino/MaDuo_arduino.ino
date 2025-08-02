/*
 * Robot.cpp
 *
 *  Created on: 2013-7-9
 *      Author: Administrator
 */

#define		TCS230			    0x01
#define		HC_SR04			    0x02
#define		ZEOR			      0x08
#define		TOP			        0x04 
#define		MID			        0x05 
#define		BOTTOM			    0x06 
#define   QTI             0x07
#define		RUN			        0xAA

#define		DEBUG			      ZEOR  
/***********************************************************************************************************************/
/*                                                  需要调整的参数                                                     */
/***********************************************************************************************************************/
/*直线上色块之间的距离*/
#define    LINE_BLACK_DIS_A  90
#define    LINE_BLACK_DIS_C  70
#define    LINE_BLACK_DIS_E  50
/*旋转中心补偿*/
#define    TRUN_CENTER_OFFSET     22//30

/*机械手同步脉冲*/
#define    SYNC_PULS      5884

#define   INIT_0    3000//底
#define   INIT_1    SYNC_PULS-INIT_0
#define   INIT_2    4000//臂
#define   INIT_3    0//手
#define   INIT_4    3700//抓


/*机械手动作数据3层*/
#define    LAYER_3_1      3000
#define    LAYER_3_2      1500//大是向上
#define    LAYER_3_3      1800//手
/*机械手动作数据2层*/
#define    LAYER_2_1      2700
#define    LAYER_2_2      1200
#define    LAYER_2_3      1300
/*机械手动作数据1层*/
#define    LAYER_1_1      2000
#define    LAYER_1_2      1500
#define    LAYER_1_3      200
/*机械手抓*/
#define    HAND_CLAW_ON       600  //往大调是开
#define    HAND_CLAW_OFF      1120   

#define DownDis 170  //200mm
#define min3v(v1, v2, v3)  ((v1)>(v2)? ((v2)>(v3)?(v3):(v2)):((v1)>(v3)?(v3):(v2)))
#define max3v(v1, v2, v3) ((v1)<(v2)? ((v2)<(v3)?(v3):(v2)):((v1)<(v3)?(v3):(v1)))

/***********************************************************************************************************************/
/*                                                  参数声明                                                           */
/***********************************************************************************************************************/
#define    POINT_A        0
#define    POINT_B        1
#define    POINT_C        2
#define    POINT_D        3
#define    POINT_E        4
/*颜色定义*/
#define    YELLOW         0
#define    WHITE          1
#define    BLACK          2
#define    RED            3
#define    BLUE           4
#define    OTHER          6

/*电机动作参数*/
#define    FRONT      0
#define    BACK       1
#define    LEFT       2
#define    RIGHT      3
#define    LEFT_90    4
#define    RIGHT_90   5
#define    LEFT_45    6
#define    RIGHT_45   7
#define    TRUN_180   8
#define    STOP       9

/*机械手动作参数*/
#define    TOP_LAYER        1
#define    MID_LAYER        2
#define    BOTTOM_LAYER     3
#define    INIT_HAND        4
#define    HAND_ON          5
#define    HAND_OFF         6
/***********************************************************************************************************************/
/*                                                控制端口声明                                                         */
/***********************************************************************************************************************/
/*电机接口*/
#define    LEFT_MOTOR      13
#define    RIGHT_MOTOR     12
/*QTI接口*/
#define    QTI_1           14
#define    QTI_2           15
#define    QTI_3           16
#define    QTI_4           17
#define    QTI_5           18
/*超声波传感器接口*/
#define    TRIG            11
#define    ECHO            10
/*颜色传感器接口*/
#define    S0              7
#define    S1              6
#define    S2              5
#define    S3              4
#define    LED             8
#define    OUT             3

typedef struct{

unsigned int  R;
unsigned int  G;
unsigned int  B;    
}Color_Typedef;

typedef struct{
unsigned int  H;
unsigned int  S;
unsigned int  L;
}HslTypedef;

Color_Typedef RGB;
HslTypedef    HSL;

unsigned long DIS=0;
int qtis=0;
char HeapCount = 0;
char PointCount = 1;
char BlockCount[3] = {1,1,1};
unsigned int Dis[3] = {100,100,100};
void InitUltrasonic(uint8_t trig,uint8_t echo);//初始化引脚
unsigned int DistanceDetection(void);//超声波测距函数int
void InitSingleQti(const int pin);//配置QTI信号脚
int GetQtiStatus(const int pin);//获取QTI的状态
void InitOpenMultiChannelServo(void);
bool SetJointsAngle(uint16_t *joint);
void UpdateMotion(int time);
void InitHc595port(void);
void InitTimer1(void);
void InitServo(void);
void SetServo(uint8_t servonum,uint16_t Angle);
void CommandSet(void);//命令执行函数
void InitContinMotorPin(uint8_t pin);
void PulseOut(uint8_t pin,int speed);
void InitColorSenor(const int pin[6]);//pin[]存放传感器引脚S0,S1,S2,S3,LED,OUT
bool ColorreCognt(int timestd[3],int value[3]);//timestd[]存放三色时间基准（单位为200us）.数组value[]存放三色通道脉冲数，顺序均为红蓝绿
bool WhiteBalance(int refertime[3]);//白平衡函数，得到三色时间基准，顺序为红蓝绿
void SetTimer2(uint8_t microsec,void (*f)());//设置定时器2的定时时间
void start(void);//开启定时器2
void stops(void);//关闭定时器2
void _Overflow(void);//定时器2溢出中断服务程序中要执行的函数
void Init_INT1(void);
void Init_Timer0(void);


uint8_t yellownum=0;
uint8_t whitenum=0;//白区当前色块个数
uint8_t rednum=0;
uint8_t blacknum=0;
uint8_t bluenum=0;

uint8_t APnum=3;// A点叠放的色块个数
uint8_t BPnum=3;
uint8_t CPnum=3;
uint8_t DPnum=3;
uint8_t EPnum=3;

uint8_t remainsum=8;

//传感器引脚定义
const int colorpin[6]={7,6,5,4,8,3};//颜色传感器引脚连接
const int qtipin[6] = {14,15,16,17,18};//前4个是前方QTI，后面是最后紧挨一起的QTI
const int Trigpin = 11;//超声波Trig引脚trig
const int Echopin = 10;//超声波Echo引脚echo

char func=1;
//颜色传感器变量
int refer_time[3]={0,0,0};//白平衡得到的基准时间
int clrpulses[3] = {0,0,0};
int currentcolor=0;

//超声波变量
unsigned int dis=0,mindis=65535;
char cnts1=0,cnts2=0;
char i=0;

char recog_times=0;

uint16_t Init_servoangle[7]={INIT_0,INIT_2,INIT_3,INIT_4,3000,3000,3000};//存放用户设置的舵机角度，servoangle[0]存放并联舵机角度，单位增量为27
uint16_t servoangle[7]={INIT_0,INIT_2,INIT_3,INIT_4,3000,3000,3000};//存放用户设置的舵机角度，servoangle[0]存放并联舵机角度

//***********************************************************************************
unsigned long pulsewidth=0;
unsigned long millimetre=0;
//uint8_t Trigpin;
//uint8_t Echopin;

#define ClkPin     		1  //PB1    移位时钟比锁存时钟早一个脉冲
#define DataPin         3  //PB3  串行数据输入端

#define Clk_High   		PORTB |= (1<<ClkPin)
#define Clk_Low    		PORTB &= ~(1<<ClkPin)
#define Data_High       PORTB |= (1<<DataPin)
#define Data_Low        PORTB &= ~(1<<DataPin)

volatile const long PwmPro = 40000;
/*********************************************************************************************
 *变量定义
 *********************************************************************************************/
uint16_t counter2;//计数器2
uint16_t servotime;//每隔servotime转一度
uint16_t servoLastAngle[8]={2946,3000,3000,3000,3000,3000,3000,3000};//备份当前位置
uint16_t servoDstAngle[8]={2946,3000,3000,3000,3000,3000,3000,3000};//存储舵机目标位置
uint16_t Parallel_HighWidSum=6000;//并联电机正脉宽之和

/* 中断服务函数中用到的变量 */
volatile long low_width;//总低电平时间
volatile unsigned int servoNowAngle[8]={2946,3000,3000,3000,3000,3000,3000,3000};//存储舵机当前位置
volatile uint16_t servoId;//舵机编号
volatile long high_widthsum;//多路舵机脉宽总和
volatile bool done=false;//完成一次目标角度标志

volatile int pulses=0;//脉冲数
volatile int stdtime=0;//时间计数器
int s0,s1,s2,s3,led,out;//颜色传感器引脚
volatile bool flag = false;

uint16_t msecs;
void (*func2)();
volatile uint8_t count;
volatile uint8_t Vofing;
volatile uint16_t Tcnt2;//计数初值


void InitialHand(void)//初始化
{
	SetJointsAngle(Init_servoangle);//1
	UpdateMotion(20);
}

void InitUltrasonic(uint8_t trig,uint8_t echo)//
{
	//Trigpin = trig;
	//Echopin = echo;
	pinMode(Trigpin,OUTPUT);//PB0置为输出，Trig
	pinMode(Echopin,INPUT);//PB2置为输入,Echo
	digitalWrite(Echopin,LOW);
	digitalWrite(Trigpin,LOW);
}
void InitSingleQti(const int pin)
{
	pinMode(pin,INPUT);
}

int GetQtiStatus(const int pin)
{
	int value;
	value = digitalRead(pin);

	return value;
}


/*********************************************************************************************
 *函数功能：初始化多路舵机(构造函数)
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void InitOpenMultiChannelServo(void)
{
	InitHc595port();//初始化595驱动端口
	InitTimer1();//初始化16位定时器1
    InitServo();//初始化舵机

	int i=0;
	for(i=0;i<8;i++)
	{
		SetServo(i,servoDstAngle[i]);
	}
	TIMSK1 |= (1<<OCIE1A);//使能比较器A匹配中断
}

/*********************************************************************************************
 *函数功能：74HC595驱动IO初始化函数
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void InitHc595port(void)
{
	DDRB |= (1<<ClkPin)|(1<<DataPin);//配置PB1和PB3为输出
	PORTB &= ~((1<<ClkPin)|(1<<DataPin));//PB1和PB3输出低电平
}

/*********************************************************************************************
 *函数功能：定时器1初始化函数
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void InitTimer1(void)
{
	TCCR1A=0;
	TCCR1B=(1<<CS11);//将晶振8分频后作为定时时钟
	TCNT1=0;//计数初值
	OCR1A=2000;//比较匹配初值,对16MHz晶振进行8分频后，定时时钟为0.5us，0.5*2000=1ms
	sei();//使能全局中断
}
/*********************************************************************************************
 *函数功能：初始化多路舵机函数
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void InitServo(void)
{
	high_widthsum=0;//多路舵机脉宽总和
	servoId=0;//舵机编号
	Parallel_HighWidSum = servoNowAngle[0]+servoNowAngle[1];//存放用户设置的舵机角度，servoangle[0]存放并联舵机角度

	int i=0;
	for(i=0;i<8;i++)
	{
		high_widthsum += servoNowAngle[i];//多路舵机当前位置对应的正脉宽之和
	}
	for(i=0;i<8;i++)
	{
		servoLastAngle[i] = servoNowAngle[i];//保存各路舵机当前角度
	}
	low_width = PwmPro - high_widthsum;//舵机控制周期（20ms）- 正脉宽之和 == 总的低电平
}
/*********************************************************************************************
 *定时器1比较器匹配中断服务函数
 *********************************************************************************************/

SIGNAL (TIMER1_COMPA_vect)
{
	if(!servoId)
	{
		//servoId=0
		Data_High;//数据线置“1”
		Clk_High;//时钟线置“1”,上升沿，将“1”移入移位寄存器，在下一个上升沿时被锁存到锁存寄存器
		Clk_Low;//时钟线置“0”
		Data_Low;//数据线置低,在下一个时钟被移入移位寄存器
	}
		Clk_High;//时钟线置"1"
		Clk_Low;//时钟线置"0"
	if(servoId==8)
	{
		OCR1A += low_width;//剩下的低电平时间
		servoId=0;
	}
	else
	{
		OCR1A += servoNowAngle[servoId];
		servoId++;
	}
}
/*********************************************************************************************
 *函数功能：初始化多路舵机函数
 *输入参数：servonum - 舵机编号
 *输出参数：   Angle      - 脉冲宽度
 *********************************************************************************************/
void SetServo(uint8_t servonum,uint16_t Angle)
{
	high_widthsum -= servoLastAngle[servonum];//将舵机的旧脉宽从总脉宽中去掉
	high_widthsum += Angle;//将新的脉宽加入
	servoLastAngle[servonum]=Angle;//更新舵机的旧脉宽
}
/*********************************************************************************************
 *函数功能：更新舵机当前角度函数
 *输入参数：无
 *输出参数：无

 *********************************************************************************************/
void CommandSet(void)
{

	uint8_t j=0;
	if(servoNowAngle[0]>servoDstAngle[0])	servoNowAngle[0] -= 27;
	if(servoNowAngle[0]<servoDstAngle[0])	servoNowAngle[0] += 27;
	if(servoNowAngle[1]>servoDstAngle[1])	servoNowAngle[1] -= 27;
	if(servoNowAngle[1]<servoDstAngle[1])	servoNowAngle[1] += 27;
	if(servoNowAngle[2]>servoDstAngle[2])	servoNowAngle[2] -= 27;
	if(servoNowAngle[2]<servoDstAngle[2])	servoNowAngle[2] += 27;
	if(servoNowAngle[3]>servoDstAngle[3])	servoNowAngle[3] -= 27;
	if(servoNowAngle[3]<servoDstAngle[3])	servoNowAngle[3] += 27;
	if(servoNowAngle[4]>servoDstAngle[4])	servoNowAngle[4] -= 27;
	if(servoNowAngle[4]<servoDstAngle[4])	servoNowAngle[4] += 27;
	if(servoNowAngle[5]>servoDstAngle[5])	servoNowAngle[5] -= 27;
	if(servoNowAngle[5]<servoDstAngle[5])	servoNowAngle[5] += 27;
	if(servoNowAngle[6]>servoDstAngle[6])	servoNowAngle[6] -= 27;
	if(servoNowAngle[6]<servoDstAngle[6])	servoNowAngle[6] += 27;
	if(servoNowAngle[7]>servoDstAngle[7])	servoNowAngle[7] -= 27;
	if(servoNowAngle[7]<servoDstAngle[7])	servoNowAngle[7] += 27;

    if( (abs(servoNowAngle[0] - servoDstAngle[0])<27)&&
		(abs(servoNowAngle[1] - servoDstAngle[1])<27)&&
		(abs(servoNowAngle[2] - servoDstAngle[2])<27)&&
		(abs(servoNowAngle[3] - servoDstAngle[3])<27)&&
		(abs(servoNowAngle[4] - servoDstAngle[4])<27)&&
		(abs(servoNowAngle[5] - servoDstAngle[5])<27)&&
		(abs(servoNowAngle[6] - servoDstAngle[6])<27)&&
		(abs(servoNowAngle[7] - servoDstAngle[7])<27))
	{
		servoNowAngle[0]=servoDstAngle[0];
		servoNowAngle[1]=servoDstAngle[1];
		servoNowAngle[2]=servoDstAngle[2];
		servoNowAngle[3]=servoDstAngle[3];
		servoNowAngle[4]=servoDstAngle[4];
		servoNowAngle[5]=servoDstAngle[5];
		servoNowAngle[6]=servoDstAngle[6];
		servoNowAngle[7]=servoDstAngle[7];
		stops();//关闭定时器2
		done = true;
	}
	for(j=0;j<8;j++)
	{
		SetServo(j,servoNowAngle[j]);
	}
	low_width = PwmPro-high_widthsum;
}
/*********************************************************************************************
 *函数功能：将输入的角度映射为脉宽函数
 *输入参数：*joint -  指向8路舵机角度的数组指针
 *输出参数： runtime   -  转到目标角度所用时间
 *假设1：0.5ms是0度，对应脉宽为1000，2.5ms是180度，对应脉宽为5000，则（5000-1000）/（180-0）=20
 *假设2：0.544ms是0度，对应脉宽为1088，2.4ms是180度，对应脉宽为4800，则（4800-1088）/（180-0）=20.6
 *********************************************************************************************/
bool SetJointsAngle(uint16_t *joint)
{
	int i;
	for(i=0;i<7;i++)
	{
		if(joint[i]<1000) joint[i]=1000;
		if(joint[i]>5000) joint[i]=5000;
	}
	if((joint[0]!=0)&&(joint[0]!=servoDstAngle[0]))
	{
		servoDstAngle[0] = joint[0];
		servoDstAngle[1] = SYNC_PULS-joint[0];
	}
	for(i=1;i<7;i++)
	{
		if((joint[i]!=0)&&(joint[i]!=servoDstAngle[i+1]))
			servoDstAngle[i+1] = joint[i];
	}
	return true;
}
/*********************************************************************************************
 *函数功能：更新多路舵机位置函数
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void UpdateMotion(int time)
{
	done = false;
	SetTimer2(time,CommandSet);
	start();//启动定时器2
	while(!(done&0x1));//等待转到位
}
void InitContinMotorPin(uint8_t pin)
{
	pinMode(pin,OUTPUT);
}
void PulseOut(uint8_t pin,int speed)
{
	digitalWrite(pin,HIGH);
	delayMicroseconds(speed);
	digitalWrite(pin,LOW);
}


void OpenRedFilter(void){//红色滤波器
	digitalWrite(s2,LOW);//S2--PD6
	digitalWrite(s3,LOW);//S3--PD7
}
void OpenBlueFilter(void){//蓝色滤波器
	digitalWrite(s2,LOW);
	digitalWrite(s3,HIGH);
}
void OpenGreenFilter(void){//绿色滤波器
	digitalWrite(s2,HIGH);
	digitalWrite(s3,HIGH);
}
void ClosePower(void){//关闭电源
	digitalWrite(s0,LOW);
	digitalWrite(s1,LOW);
}
void Out1than50(void){//输出比例1:50
	digitalWrite(s0,LOW);
	digitalWrite(s1,HIGH);
}
void Out1than5(void){//输出比例1:5
	digitalWrite(s0,HIGH);
	digitalWrite(s1,LOW);
}
void Out1than1(void){//输出比例1:1
	digitalWrite(s0,HIGH);
	digitalWrite(s1,HIGH);
}
bool ColorreCognt(int timestd[3],int value[3])
{
	digitalWrite(led,HIGH);//开灯
	delay(100);

	//红色
	pulses=0;
	stdtime=0;
	TCNT0 = 0;
	TIMSK0 |=(1<<OCIE0A);//定时器1溢出中断使能,执行中断服务函数
	Out1than1();
	OpenRedFilter();
	EIMSK |= (1<<INT1);      /*使能外部中断1请求*/
	while(stdtime != timestd[0]);
	EIMSK &= ~(1<<INT1);
	TIMSK0 &= ~(1<<OCIE0A);
	ClosePower();
	if(pulses > 255){
		pulses = 255;
	}
	value[0] = pulses;


	//蓝色
	pulses=0;
	stdtime=0;
	TCNT0 = 0;
	TIMSK0 |=(1<<OCIE0A);//定时器1溢出中断使能,执行中断服务函数
	Out1than1();
	OpenBlueFilter();
	EIMSK |= (1<<INT1);      /*使能外部中断1请求*/
	while(stdtime != timestd[1]);
	EIMSK &= ~(1<<INT1);
	TIMSK0 &= ~(1<<OCIE0A);
	ClosePower();
	if(pulses > 255){
		pulses = 255;
	}
	value[1] = pulses;

	//绿色
	pulses=0;
	stdtime=0;
	TCNT0 = 0;
	TIMSK0 |=(1<<OCIE0A);//定时器1溢出中断使能,执行中断服务函数
	Out1than1();
	OpenGreenFilter();
	EIMSK |= (1<<INT1);      /*使能外部中断1请求*/
	while(stdtime != timestd[2]);
	EIMSK &= ~(1<<INT1);
	TIMSK0 &= ~(1<<OCIE0A);
	ClosePower();
	if(pulses > 255){
		pulses = 255;
	}
	value[2] = pulses;

	digitalWrite(s0,LOW);
	digitalWrite(s1,LOW);
	digitalWrite(led,LOW);

	return true;
}
bool WhiteBalance(int refertime[3])
{
	digitalWrite(led,HIGH);//LED
	delay(100);

	//红色滤波器
	stdtime=0;
	pulses=0;	//计数清零
	TCNT0 = 0;
	TIMSK0 |=(1<<OCIE0A);//定时器1溢出中断使能,执行中断服务函数
	Out1than1();
	OpenRedFilter();
	EIMSK |= (1<<INT1);      /*使能外部中断1请求*/
	while(!(flag&0x1));//等待定时到
	TIMSK0 &= ~(1<<OCIE0A);
	ClosePower();
	EIMSK &= ~(1<<INT1);

	refertime[0] = stdtime;//保存脉冲数
	flag=false;

	//蓝色
	stdtime=0;
	pulses=0;	//计数清零
	TCNT0 = 0;
	TIMSK0 |=(1<<OCIE0A);//定时器1溢出中断使能,执行中断服务函数
	Out1than1();
	OpenBlueFilter();
	EIMSK |= (1<<INT1);      /*使能外部中断1请求*/
	while(!(flag&0x1));
	TIMSK0 &= ~(1<<OCIE0A);
	ClosePower();
	EIMSK &= ~(1<<INT1);

	refertime[1] = stdtime;
	flag=false;

	//绿色
	stdtime=0;
	pulses=0;	//计数清零
	TCNT0 = 0;
	TIMSK0 |=(1<<OCIE0A);//定时器1溢出中断使能,执行中断服务函数
	Out1than1();
	OpenGreenFilter();
	EIMSK |= (1<<INT1);      /*使能外部中断1请求*/
	while(!(flag&0x1));
	TIMSK0 &= ~(1<<OCIE0A);
	ClosePower();
	EIMSK &= ~(1<<INT1);

	refertime[2] = stdtime;
	flag=false;

	digitalWrite(s0,LOW);//S0,关闭电源
	digitalWrite(s1,LOW);//S1
	digitalWrite(led,LOW);//LED

	return true;
}
void InitColorSenor(const int pin[6])
{
	s0=pin[0];//7
	s1=pin[1];//6
	s2=pin[2];//5
	s3=pin[3];//4
	led=pin[4];//8
	out=pin[5];//3
	pinMode(s0,OUTPUT);
	pinMode(s1,OUTPUT);
	pinMode(s2,OUTPUT);
	pinMode(s3,OUTPUT);
	pinMode(led,OUTPUT);
	pinMode(out,INPUT);

	Init_INT1();//外部中断1
	Init_Timer0();//16位定时器1
}
void Init_INT1(void)
{
	cli();//屏蔽所有中断
	EICRA |= (1<<ISC10)|(1<<ISC11); //外部中断1上升沿触发中断
}
void Init_Timer0(void)
{
    TCCR0A |= (1<<WGM01);//CTC功能
    TCCR0B |= (1<<CS01); //8分频，时钟周期16/8=2MHz，0.5us
    TCNT0 = 0;
    OCR0A = 10;
    sei();//开启总中断
}
SIGNAL(INT1_vect)
{
	pulses++;//脉冲计数器
	if(pulses == 255)//白色光源时，三色滤波的值都默认是255
	{
		flag = true;
	}
}
SIGNAL(TIMER0_COMPA_vect)//TCNT0与OCR0A发生匹配时，TCNT0自动清零并发生中断
{
	stdtime++;
}


/*********************************************************************************************
 *函数功能：设置定时器2
 *输入参数：microsec  -  定时时间(ms)
 *输出参数：执行函数指针
 *********************************************************************************************/
void SetTimer2(uint8_t microsec,void (*f)())//设置定时器2的定时时间
{
	float prescaler=0.0;//分频系数
#if defined (__AVR_ATmega328P__)
	TIMSK2 &= ~(_BV(TOIE2));//关闭定时器2溢出中断使能
	TCCR2A &= ~(_BV(WGM21)|_BV(WGM20));//普通模式
	TCCR2B &= ~(_BV(WGM22));//与TCCR2A关联起来
	ASSR &= ~(_BV(AS2));//定时器2由I\O时钟驱动
	TIMSK2 &= ~(_BV(OCIE2A));//关闭输出比较A匹配中断使能
	if((F_CPU >= 1000000UL)&&(F_CPU <= 16000000UL))
	{
		TCCR2B |= _BV(CS22);//64分频
		TCCR2B &= ~(_BV(CS21)|_BV(CS20));
		prescaler = 64.0;
	}
#endif
	Tcnt2 = 256-(int)((float)F_CPU*0.001/prescaler);// 定时器2计数初值
													//8位定时器，最大值是256，,对16MHz进行64分频，16000000/64=250000
													//250000*0.001=250KHz,16/64=0.25,1/0.25=4us,4*250=1000us=1ms
													//所以是1ms产生一次溢出中断
	if(microsec == 0)
	{
		msecs = 1;//至少定时1ms
	}
	else
	{
		msecs = microsec;
	}
	func2 = f;//指针赋值

}

/*********************************************************************************************
 *函数功能：开启定时器2
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void start(void)//开启定时器2
{
	count = 0;
	Vofing = 0;//溢出标志变量
#if defined (__AVR_ATmega328P__)
	TCNT2 = Tcnt2;//赋计数初值
	TIMSK2 |= _BV(TOIE2);//开启定时器2溢出中断使能
#endif
}

/*********************************************************************************************
 *函数功能：关闭定时器2
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void stops(void)//关闭定时器2
{
#if defined (__AVR_ATmega328P__)
	TIMSK2 &= ~(_BV(TOIE2));//关闭定时器2溢出中断使能
#endif
}

/*********************************************************************************************
 *函数功能：溢出中断执行函数
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
void _Overflow(void)//定时器2溢出中断服务程序中的执行函数
{
	count += 1;//其他计数器
	if((count >= msecs*2) && (!Vofing))//10ms
	{
		Vofing = 1;
		count = 0;
		(*func2)();
		Vofing = 0;
	}
}

/*********************************************************************************************
 *函数功能：定时器2溢出中断服务函数
 *输入参数：无
 *输出参数：无
 *********************************************************************************************/
ISR(TIMER2_OVF_vect)
{
#if defined (__AVR_ATmega328P__)
	TCNT2 = Tcnt2;//重新赋计数初值
#endif
	_Overflow();//调用函数
}
   

/***********************************************************************************************************************/
/*                                                 底层驱动                                                            */
/***********************************************************************************************************************/
/*
函数名：MotorControl
功能：电机底层驱动
参数：leftSpeed：左电机的速度
      rightSpeed：右电机的速度
返回：void
*/
void MotorControl(unsigned int leftSpeed,unsigned int rightSpeed)
{
      digitalWrite(RIGHT_MOTOR,HIGH);
      delayMicroseconds(rightSpeed);
      digitalWrite(RIGHT_MOTOR,LOW);
      digitalWrite(LEFT_MOTOR,HIGH);
      delayMicroseconds(leftSpeed);
      digitalWrite(LEFT_MOTOR,LOW); 
      delay(20);
}
/*
函数名：MotorControl
功能：电机动作接口(转速)
参数：action：
          FRONT
          BACK
          LEFT
          LEFT_45
          LEFT_90
          RIGHT
          RIGHT_45
          RIGHT_90
          STOP
          TRUN_180
返回：void
*/
void MotorAction(unsigned char action)
{
  unsigned int i = 0;
  switch (action)
  {
    case FRONT:     {MotorControl(1700,1270);}break;
    case BACK:      {do{MotorControl(1270,1700);delay(1);i++;}while(i < 180);}break;
    case LEFT:      {MotorControl(1460,1160);}break;
    case LEFT_45:   {do{MotorControl(1460,1160);delay(1);i++;}while(i < 60);}break;
    case LEFT_90:   {do{MotorControl(1260,1160);delay(1);i++;}while(i < 60);}break;
    case RIGHT:     {MotorControl(1740,1600);}break;
    case RIGHT_45:  {do{MotorControl(1740,1600);delay(1);i++;}while(i < 60);}break;
    case RIGHT_90:  {do{MotorControl(1460,1760);delay(1);i++;}while(i < 60);}break;
    case STOP:      {MotorControl(1500,1500);}break;
    case TRUN_180:  {do{MotorControl(1460,1760);delay(1);i++;}while(i < 100);}break;
    default:        {MotorControl(1500,1500);}break;  
  }
}

/*
函数名：HandAction
功能：  机械手控制接口
参数：  void
返回：  void
*/
void HandAction(unsigned char action)
{
  switch(action)//往小改机械臂低
  {
    case 3:{
      servoangle[0]=1800;
      servoangle[1]=1500;
      servoangle[2]=200;
    }break;
  
    case 2:{
      servoangle[0]=1500;
      servoangle[1]=1500;
      servoangle[2]=200;
    }break;

    case 1:{
      servoangle[0]=1300;
      servoangle[1]=LAYER_1_2;
      servoangle[2]=LAYER_1_3;    
    }break;

    case INIT_HAND:{
      servoangle[0]=Init_servoangle[0];
      servoangle[1]=Init_servoangle[1];
      servoangle[2]=Init_servoangle[2];
    }break;
    
    case HAND_ON:{
      servoangle[3]=Init_servoangle[3] - HAND_CLAW_ON;
    }break; 

    case HAND_OFF:{
      servoangle[3]=Init_servoangle[3] + HAND_CLAW_OFF;
    }break;   
    default :{
      
      
    }break;
  }
  SetJointsAngle(servoangle);
  UpdateMotion(15);
}



/*
函数名：TCS230_GetColor
功能：  获取颜色
参数：  void
返回：  void
*/
char TCS230_GetColor(void)
{
  char color;
  HslTypedef * hsl;
      ColorreCognt(refer_time,clrpulses);
      if(clrpulses[0] >= 255)
      {
         RGB.R = 255;
      }else{
         RGB.R = clrpulses[0];
      }
      if(clrpulses[2] >= 255)
      {
         RGB.G = 255;
      }else{
         RGB.G = clrpulses[2];
      }      
      if(clrpulses[1] >= 255)
      {
         RGB.B = 255;
      }else{
         RGB.B = clrpulses[1];
      }
      hsl = RGBtoHSL(&RGB);
      Serial.print("H:");
      Serial.print(hsl->H,DEC);
      Serial.print("S:");
      Serial.print(hsl->S,DEC);
      Serial.print("L:");
      Serial.println(hsl->L,DEC);       
      if((hsl->H > 210) && (hsl->H < 275) && (hsl->L < 20))
     {
      Serial.println("blue");
       color = BLUE;
     }else{
       if((hsl->H > 320) && (hsl->H < 360) && (hsl->S > 45))
       {
         Serial.println("red");
         color = RED;
       }else{
         if((hsl->H > 20) && (hsl->H < 50) && (hsl->S > 40))
         {
              Serial.println("yellow");
           color = YELLOW;
         }else{
           if((hsl->L < 15) && ((hsl->S < 15)))
           {
               Serial.println("black");
              color = BLACK;
           }else{
             if((hsl->L > 80))// && (hsl->S < 40)
             {
                 Serial.println("white");
               color = WHITE;
             }else{
                Serial.println("other");
              color = OTHER;
             }  
           }
         }
       }     
     }
     return color;
}
/*
函数名：RGBtoHSL
功能：  获取颜色
参数： TCS_Handle
返回： void
*/
extern HslTypedef *RGBtoHSL(Color_Typedef *rgb)
{
  float h = 0,s = 0,l = 0;
  float r = rgb->R / 255.0;
  float g = rgb->G / 255.0;
  float b = rgb->B / 255.0;
  float maxVal = max3v(r,g,b);
  float minVal = min3v(r,g,b);
  if(maxVal == minVal)
  {
    h = 0;
  }else{
    if((maxVal == r) && (g >= b))
    {
      h = 60.0 * (g - b) / (maxVal - minVal);
    }else{
      if((maxVal == r) && (g < b))
      {
        h = 60.0 * (g - b) / (maxVal - minVal) + 360.0;
      }else{
        if(maxVal == g)
        {
          h = 60.0 * (b - r) / (maxVal - minVal) + 120.0;
        }else{
          if(maxVal == b)
          {
            h = 60.0 * (r - g) / (maxVal - minVal) + 240.0;
          }
        }
      }
    }
  }
  l = (maxVal + minVal) / 2.0;
  
  if((l == 0 )|| (maxVal == minVal))
  {
    s = 0;
  }else{
    if((l > 0) && (l <= 0.5))
    {
      s = (maxVal - minVal) / (maxVal + minVal);
    }else{
      if(l > 0.5)
      {
        s = (maxVal - minVal) / (2 - (maxVal + minVal));
      }
    }
  }
  HSL.H = (h > 360)? 360 : ((h < 0)? 0 : h);
  HSL.S = ((s>1)? 1 : ((s<0)?0:s))*100;
  HSL.L = ((l>1)? 1 : ((l<0)?0:l))*100;
  return &(HSL);
}

/***********************************************************************************************************************/
/*                                                中间控制层                                                           */
/***********************************************************************************************************************/


/*
函数名：CarryToTargetArea1
功能： 搬运到目标点
参数：  void
返回：  void
*/
void CarryToTargetArea1(void)
{
  int st0 = 0; 
  uint16_t color = 0;
  uint8_t state = 0;
  unsigned int i = 0;
  static uint8_t flag = 0;
  static uint8_t clocrCount[5] = {1,1,1,1,1};
    /*走到第二个路口*/
    /*旋转中心补偿*/
    //do{MotorAction(FRONT);delay(1);i++;}while(i < TRUN_CENTER_OFFSET);
    //HandAction(BOTTOM_LAYER);//INIT_HAND
    //HandAction(HAND_ON);
    
    for(i=0;i<3;i++)
    {
     WhiteBalance(refer_time);//白平衡
     delay(200);
     }
    do{color = TCS230_GetColor();}while(color == OTHER);
    //HandAction(HAND_OFF);
    //HandAction(INIT_HAND);
    i = 0;
    do{
    if(state == 31)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
     
  }while(state != 31);
  i = 0;
 do{MotorAction(FRONT);delay(1);i++;}while(i < 15);
  switch(color)
  {
    case YELLOW:  {MotorAction(LEFT_90);}break;
    case WHITE:   {MotorAction(LEFT_45);}break; 
    case RED:     {MotorAction(STOP);}break;
    case BLACK:   {MotorAction(RIGHT_45);}break;
    case BLUE:    {MotorAction(RIGHT_90);}break;      
  }
    /*等待第5个QTI进入黑线*/
    do{
    if(state == 4)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
     
  }while(state != 4);
    
    do{
    if(state == 0)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
     
  }while(state != 0);
     switch(color)
  {
    case YELLOW:  {do{MotorAction(FRONT);delay(1);i++;}while(i < 30);}break;
    case WHITE:   {do{MotorAction(FRONT);delay(1);i++;}while(i < 120);}break; 
    case RED:     {do{MotorAction(FRONT);delay(1);i++;}while(i < 140);}break;
    case BLACK:   {do{MotorAction(FRONT);delay(1);i++;}while(i < 120);}break;
    case BLUE:    {do{MotorAction(FRONT);delay(1);i++;}while(i < 30);}break;      
  }
    /*判断第五个QTI是否到达白色圆圈*/
  switch(clocrCount[color])
  {
    case 3:{HandAction(TOP_LAYER);}break;
    case 2:{HandAction(MID_LAYER);}break;
    case 1:{HandAction(BOTTOM_LAYER);}break;    
  }
  clocrCount[color] ++;
  BlockCount[HeapCount] --;
  HandAction(HAND_ON);
  HandAction(INIT_HAND);
  MotorAction(TRUN_180);
  //过白点
  do{
    if(state == 0)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
  }while(state != 0);
//回到中心
  switch(color)
  {
    case BLUE:
    do{
    if(state == 31)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
  }while(state != 31);
  break;
  case BLACK:
  break;
  case RED:
  do{
    if(state == 31)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
  }while(state != 31);
  break;
  case WHITE:
  do{
    if(state == 29)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
  }while(state != 29);
  break;
  break;
  case YELLOW:
  do{
    if(state == 31)
    {
      st0 = 1;
      }
      else
      {
        st0 = 0;
        }
  switch(st0)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
  }while(state != 31);
  break;
  }

i = 0;
 do{MotorAction(FRONT);delay(1);i++;}while(i < 15);

    /*旋转中心补偿*/
  switch(color)
  {
    case BLUE: {MotorAction(LEFT_90);}break;
    case BLACK: {MotorAction(LEFT_45);}break;  
    case RED: {MotorAction(STOP);}break;break;
    case WHITE: {MotorAction(RIGHT_45);}break;
    case YELLOW: {MotorAction(RIGHT_90);}break;
  }
  int DST = 0;
   do{
    dis = DistanceDetection();
    if(dis == 8)
    {
      DST = 1;
      }
      else if(dis != 8)
      {
        DST = 0;
        }
  switch(DST)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
     
  }while(dis != 8);
  
  switch(BlockCount[HeapCount])
  {
    case 3:{HandAction(TOP_LAYER);}break;
    case 2:{HandAction(MID_LAYER);}break;
    case 1:{HandAction(BOTTOM_LAYER);}break;
  }
  HandAction(HAND_OFF);
  HandAction(INIT_HAND);
  do
  {
  dis = DistanceDetection();
  if(dis < 15 )
  {
    MotorAction(BACK);
  }
  if(dis > 15 )//
  {
    MotorAction(FRONT);
  }
  if(dis == 15)
  {
    MotorAction(STOP);
  }
  }while(dis != 15);
  if(BlockCount[HeapCount] != 0)
   MotorAction(TRUN_180);
  if(BlockCount[HeapCount] < 0)
    BlockCount[HeapCount] = 0;
  if(BlockCount[HeapCount] == 0)
  {
    if(HeapCount == 0)
    { 
      if(flag)
        PointCount = 5;
      flag = 1;
    }
    HeapCount --;
    if(HeapCount < 0)
      HeapCount = 0;
  } 
}
/*
函数名：EndRun
功能：  结束运行
参数：  void
返回：  void
*/
void EndRun(void)
{
  unsigned int i = 0; 
  char state = 0;
}
/*
函数名：StartRun
功能：  开始运行
参数：  void
返回：  void
*/
void StartRun(void)
{
  int state = 0,ii = 0;
  for(int i = 3;i >= 1;i--)//搬左三至原点
  {
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 120);//直走
    ii = 0;
    MotorAction(LEFT_90);//左转90度
    MotorAction(BACK);//后退一段距离
  HandAction(i);//放下机械臂（第i层）
  HandAction(HAND_OFF);
  HandAction(INIT_HAND);
    MotorAction(LEFT_90);//左转90度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
  HandAction(4-i);//放下机械臂（第4-i层）
  HandAction(HAND_ON);
  HandAction(INIT_HAND);
    MotorAction(TRUN_180);//转180度

  }
/*
  for(int i = 3;i >= 1;i--)//搬中三至原点
  {
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
  HandAction(i);//放下机械臂（第i层）
  HandAction(HAND_OFF);
  HandAction(INIT_HAND);
    MotorAction(TRUN_180);//转180度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
  HandAction(4-i);//放下机械臂（第4-i层）
  HandAction(HAND_ON);
  HandAction(INIT_HAND);
    MotorAction(TRUN_180);//转180度

  }

  for(int i = 3;i >= 1;i--)//搬右三至原点
  {
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
    MotorAction(RIGHT_90);//右转90度
  HandAction(i);//放下机械臂（第i层）
  HandAction(HAND_OFF);
  HandAction(INIT_HAND);
    if(i == 1) continue;
    MotorAction(RIGHT_90);//右转90度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
  HandAction(BOTTOM_LAYER);//放下机械臂（第4-i层）
    HandAction(HAND_ON);//打开机械手的手抓
    MotorAction(TRUN_180);//转180度

  }
  
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
  HandAction(1);//放下机械臂（第1层）
    HandAction(HAND_ON);//打开机械手的手抓
    MotorAction(RIGHT_90);//右转90度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
  HandAction(2);//放下机械臂（第i层）
  HandAction(HAND_OFF);
  HandAction(INIT_HAND);
    MotorAction(TRUN_180);//转180度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
    MotorAction(LEFT_90);//左转90度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;


    HandAction(1);//放下机械臂
    HandAction(HAND_OFF);//关闭机械手的手抓
    MotorAction(TRUN_180);//转180度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;
    HandAction(1);//放下机械臂
    HandAction(HAND_ON);//打开机械手的手抓
    MotorAction(TRUN_180);//转180度
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
    ii = 0;

    
    for(int i = 1;i<3;i++)
    {
      //搬物至中
        HandAction(3);//放下机械臂
        HandAction(HAND_OFF);//关闭机械手的手抓
          HandAction(INIT_HAND);
        MotorAction(TRUN_180);//转180度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        HandAction(2);//放下机械臂
        HandAction(HAND_ON);//打开机械手的手抓
          HandAction(INIT_HAND);
        MotorAction(TRUN_180);//转180度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;

      //搬物至左
        HandAction(2);//放下机械臂
        HandAction(HAND_OFF);//关闭机械手的手抓
          HandAction(INIT_HAND);
        MotorAction(TRUN_180);//转180度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        MotorAction(LEFT_90);//左转90度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        HandAction(2);//放下机械臂
        HandAction(HAND_ON);//打开机械手的手抓
          HandAction(INIT_HAND);
        MotorAction(TRUN_180);//转180度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        MotorAction(RIGHT_90);//右转90度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;

      //搬物至右
        HandAction(1);//放下机械臂
        HandAction(HAND_OFF);//关闭机械手的手抓
          HandAction(INIT_HAND);
        MotorAction(TRUN_180);//转180度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        MotorAction(RIGHT_90);//右转90度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        HandAction(2);//放下机械臂
        HandAction(HAND_ON);//打开机械手的手抓
          HandAction(INIT_HAND);
        MotorAction(TRUN_180);//转180度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;
        MotorAction(LEFT_90);//左转90度
        do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走
        ii = 0;

    }
    do{MotorAction(FRONT);delay(1);ii++;}while(ii < 70);//直走（回到出发点）
    ii = 0;*/
    
   //MotorAction(STOP);
          /*FRONT
          BACK
          LEFT
          LEFT_45
          LEFT_90
          RIGHT
          RIGHT_45
          RIGHT_90
          STOP
          TRUN_180*/
  /*do{
    dis = DistanceDetection();
    if(dis == 5)
    {
      flag = 1;
      }
      else
      {
        flag = 0;
        }
  switch(flag)
  {
    case 0:
    MotorAction(FRONT);
    break;
    case 1:
    MotorAction(STOP);
    break;
    }
  }while(dis != 5);*/

  
}
/***********************************************************************************************************************/
/*                                                  用户调用接口                                                       */
/***********************************************************************************************************************/
void CarryRun()
{
  switch(PointCount)
  {
    case 4:{CarryToTargetArea1();}break;
    case 5:{EndRun();while(1);}break; 
  }
}
//初始化
void setup(void){
    Serial.begin(115200); 
    
    pinMode(LEFT_MOTOR,OUTPUT);
    pinMode(RIGHT_MOTOR,OUTPUT);
   
    pinMode(QTI_1,INPUT);
    pinMode(QTI_2,INPUT);
    pinMode(QTI_3,INPUT);
    pinMode(QTI_4,INPUT);
    pinMode(QTI_5,INPUT);
    
    pinMode(Trigpin,OUTPUT);//,TRIG
    pinMode(Echopin,INPUT);//Echopin,ECHO
    InitOpenMultiChannelServo();//初始化595端口
    //HandAction(INIT_HAND);//INIT_HAND，BOTTOM_LAYER，MID_LAYER，TOP_LAYER
    //HandAction(BOTTOM_LAYER);
    InitColorSenor(colorpin);//
	  
	  delay(2000);
    
    for(i=0;i<3;i++)
    {
     //WhiteBalance(refer_time);//白平衡
     delay(200);
     }

  StartRun();//
//MotorAction(BACK);
//MotorAction(TRUN_180);
}
//循环
void loop(void){
do{
  
  //HandAction(BOTTOM_LAYER);//TOP_LAYER(直抬起),MID_LAYER（平抬起抖动）,BOTTOM_LAYER（平抬起）,INIT_HAND（初始化动作）
  //HandAction(HAND_OFF);
  //HandAction(INIT_HAND);
//JiaoZhun();
//HandAction(HAND_OFF);/*关闭机械手的手抓*///yes
//HandAction(INIT_HAND);/*将机械臂抬起*///yes
/*
HandAction(HAND_ON);
delay(1000);
HandAction(HAND_OFF);
delay(1000);
*/
//char a = TCS230_GetColor();
//Serial.println(a);
//RGBtoHSL(TCS230_GetColor());
//待测carry系列函数
}while(1);
}


/*
 * 主函数
 */
/*int main(void)
{

  setup();
  delay(200);
  

  while(1)
  {
  loop();
  }
  return 0;
}*/

/*
函数名：MotorControl
功能：电机底层驱动
参数：leftSpeed：左电机的速度
      rightSpeed：右电机的速度
返回：void
*/
/*
函数名：MotorAction(FRONT);yes
功能：电机动作接口(转速)
参数：action：
          FRONT
          BACK
          LEFT
          LEFT_45
          LEFT_90
          RIGHT
          RIGHT_45
          RIGHT_90
          STOP
          TRUN_180
返回：void
*/
/*
函数名：HandAction/yes
功能：  机械手控制接口
参数：  TOP_LAYER(直抬起),MID_LAYER（平抬起抖动）,BOTTOM_LAYER（平抬起）,INIT_HAND（初始化动作）,HAND_ON,HAND_OFF
返回：  void
*/
/*
函数名：TCS230_GetColor
功能：  获取颜色
参数：  void
返回：  void
*/
/*
函数名：RGBtoHSL
功能：  获取颜色
参数： TCS_Handle
返回： void
*/
/*
函数名：MoveColorBlack
功能：  搬运色块
参数：  point:点(A-E)
        layerNum:层数
返回：  void
*/
/*
函数名：  Tracking
功能：    循迹
参数：    qtiStatus：QTI的状态字
                    
返回：    QTI的状态字
*/
/*
函数名：MoveColorBlack
功能：  搬运色块
参数：  point:点(A-E)
        layerNum:层数
返回：  void
*/
/*
函数名：CarryAtoO
功能：  搬运A点的色块到直线
参数：  void
返回：  void
*/
/*
函数名：CarryToTargetArea1
功能： 搬运到目标点
参数：  void
返回：  void
*/


