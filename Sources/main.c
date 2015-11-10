#include <hidef.h>          /* common defines and macros */
#include "derivative.h"     /* derivative-specific definitions */
#include <MC9S12XS128.h>


                                                                    

#define ROW     40          //定义图像采集行数：40行
#define COLUMN  75         //定义图像采集列数：75列
#define CENTER  37         //数组中心

unsigned char Image_Data[ROW][COLUMN];    //图像数组
unsigned char Line_C=0;          //行数统计值
unsigned char l,r;
unsigned char m = 0;
unsigned char t=0;
unsigned char  Interval;        //采集有效行数间隔


unsigned char  THRESHOLD=80;  //黑白二值化图像阈值(经验值，随环境光变换而变化)


unsigned char ctr_tiaobian [6]={37,37,37,37,37,37};            //各行黑线
unsigned char tiaobian_L[6];                         //存储左跳变数组
unsigned char tiaobian_R[6];                         //存储右跳变数组
int caiji[6]={14,15,16,19,20,21};                    //采集行



unsigned char ctr=37; //黑线
                                          
  float K=32 ;   //【比例系数】
  int Y=40;      //【车速】
  float Error ;                                                                    
  float Last_Error=0 ;                                                               
  float Pre_Error=0 ;                                                               
  float This_Output;   
  float Last_Output=0;                                           
  float Learn_P=10 ;
  float Learn_D=1 ;
  float tempWeight_P=1;
  float tempWeight_D=1;
/***************************************************
** 函数名称: PLL_Init
** 功能描述: 时钟初始化函数
** 说明:     总线时钟选定40M
****************************************************/
void PLL_Init(void)
{
  CLKSEL=0x00; //40mhz
  SYNR =0xc0 | 0x04;                        
  REFDV=0x80 | 0x01; 
  PLLCTL_PLLON=1;
  POSTDIV=0X00;
  asm(nop);
  asm(nop);
  while(0==CRGFLG_LOCK); //锁相环锁定
  CLKSEL_PLLSEL=1; //选定PLL时钟
}

/***************************************************
** 函数名称: TIM_Init
** 功能描述: 行场中断初始化函数
** 说明:     行中断上升沿触发  场中断下降沿触发
****************************************************/
void TIM_Init(void) 
{
TIOS =0x00;        //定时器通道0，1 为输入捕捉
TSCR1=0x80;        //定时器使能
TCTL4=0x09;        //通道0 捕捉上升沿通道1 捕捉下降沿      3 2 1 0 通道
TIE=0x03;          //通道0，1 中断使能
TFLG1=0xFF;        //清中断标志位
}
 
/***************************************************
** 函数名称: PWM_Init
** 功能描述: 舵机控制初始化程序
** 说明:     频率50HZ  
****************************************************/ 
 
 void PWM_Init()   // 4,5级联
{     
    PWME=0x00;        // 00000000   disable PWM45
    PWMCAE=0; //对齐方式,左
    PWMCLK=0xff;    //选择A或B时钟通道
    PWMPOL=0xff; //极性,
    PWMPRCLK=0x00;    // clkA=clkB=40M
    PWMSCLA=20;        // clkSA=A/(2*20)=1M=0.001ms
    PWMSCLB=20;
    PWMCTL_CON23= 1;
    PWMCTL_CON45=1;   // connect channel 4,5 
    PWMCTL_CON67= 1;
    PWMPOL_PPOL3=0;
    PWMPOL_PPOL5=1;   // output low level after high level   duty=high level
    PWMPOL_PPOL7=0;    
    PWMCAE_CAE5=0;    // left aligned
    PWMPER23=200;
    PWMPER45=20000;   // period=PWMPER*SA=20000*0.001ms=20ms   frequency=1/period=50Hz
    PWMPER67=200;
    PWMDTY45=1500;       // pwm_duty=PWMDTY/PWMPER
    PWME=0x30;        // 00001010   enable PWM45
}



/************************************************** 
** 函数名称: 二值化+动态阀值+滤波
** 功能描述: 图像预处理
***************************************************/

void erzhi() 
{
  unsigned char i,j;
  unsigned int y=0;
 for(i = 0;i< ROW;i++) 
  {
    for(j=0;j<COLUMN;j++) 
    {y=y+Image_Data[i][j];
    }
    y=y/COLUMN-7;
    for(j = 0;j < COLUMN ;j++) 
    { 
    
          if(Image_Data[i][j]>y)  
                Image_Data[i][j]=1;        //白线
                 else  Image_Data[i][j]=0;    //黑线
    }
    y=0;
  }
} 



/************************************************** 
** 函数名称: 采集跳变
** 功能描述: 
***************************************************/
void tiaobian() // 【跳变采集】
 { 
   int i; 
   int n;  //数组里第n个数
   int a;  //行所在的黑线
   int b;   //决定第几行
   
   for(n=0;n<6;n++)         // 6行循环
   {
	   b=caiji[n];            // 对应所在的行
	   a=ctr_tiaobian[n];     // 对应的中心黑线
	   
	   for(i=a;i<75-2;i++)   
	   {
		    if((Image_Data[b][i]==1)&&(Image_Data[b][i-1]==1)&&(Image_Data[b][i+1]==0)&&(Image_Data[b][i+2]==0))    //第19行 白白黑 中间白为右边线
			{
              tiaobian_R[n]=i+1; break;                                                                     //保存右边线列数r
			} 
          else tiaobian_R[n]=COLUMN;                                                                        //否则右边线在中间
	   }
       for(i=a;i>2;i--)                                                                 //从ctr开始向做寻找，初始化为屏幕中间
	   {
	        if((Image_Data[b][i]==1)&&(Image_Data[b][i+1]==1)&&(Image_Data[b][i-1]==0)&&(Image_Data[b][i-2]==0))    //第19行 黑白白 中间白为左边线
			{
              tiaobian_L[n]=i+1; break;                                                                      //保存左边线列数l
			}  
           else tiaobian_L[n]=0;                                                                         //否则左边线在中间
	   }
	   ctr_tiaobian[n]=(tiaobian_L[n]+tiaobian_R[n])/2;

   }
}

/************************************************** 
** 函数名称: 偏差计算函数 （偏差小于0应向左转，偏差大于零应向右拐）
** 功能描述: 将每行白道的中心与数组的中心求偏差
***************************************************/
 void piancha()  //【偏差】
 {
	 if((tiaobian_R[3]=COLUMN)&&(tiaobian_L[3]=0)&&(tiaobian_R[4]=COLUMN)&&(tiaobian_L[4]=0))	 
	 ctr=(ctr_tiaobian[0]+ctr_tiaobian[1]+ctr_tiaobian[2])/4;	 
	 else	 
	 ctr=(ctr_tiaobian[3]+ctr_tiaobian[4]+ctr_tiaobian[5])/3;
 }
      
           
 /************************************************** 
** 函数名称: 舵机
** 功能描述: 
***************************************************/
void duoji()
{
	int a,b;                                                                                                                                                
                 
    float Weight_P;                                               
    float Weight_D;                                                                                                        
                       
    float X_P;
    float X_D;

	Error=CENTER-ctr;                         //【算偏差】

	if(Error<3&&Error>-3)       //【偏差小时】
	{   
			PWMDTY45=1460;
      Last_Output=0;
	}
	else                                      //【偏差大时】
	{
    X_P=Error - Last_Error;
		X_D=Error - 2*Last_Error + Pre_Error;

		if(Last_Error<3&&Last_Error>-3) Last_Output=0;  //【如果上个偏差小，偏差重新计算】
        //【上次无偏差，不更新权重】
		if(Last_Error>0)                //【上次正偏差，更新权重】          
		{
		tempWeight_P = tempWeight_P + Learn_P*This_Output*Error*(2*Error-Last_Error);      //更新暂量权重【P】
        tempWeight_D = tempWeight_D + Learn_D*This_Output*Error*(2*Error-Last_Error);      //更新暂量权重【D】
		}
		else if(Last_Error<0)                //【上次负偏差，更新权重】 
		{
		tempWeight_P = tempWeight_P + Learn_P*This_Output*Error*(Last_Error-2*Error);      //更新暂量权重【P】
        tempWeight_D = tempWeight_D + Learn_D*This_Output*Error*(Last_Error-2*Error);      //更新暂量权重【D】		
		}

        Weight_P= tempWeight_P /(tempWeight_P+tempWeight_D) ;
        Weight_D= tempWeight_D /(tempWeight_P+tempWeight_D) ;

		This_Output = Last_Output+ K*(Weight_P*X_P+Weight_D*X_D);

        b=(int)This_Output;   
        a=b+1460 ;
        if(a>1815)          //最大转向角限制350
        a=1815;   
        else 
        if(a<1135)          //最大转向角限制300
        a=1135;
        PWMDTY45=a;
       
		Last_Output=This_Output;
	}

	Pre_Error=Last_Error;
    Last_Error=Error;

}




/***************************************************
** 函数名称: 电机
** 功能描述: 
** 说明: 
****************************************************/  
void motor_f()  //电机驱动
{
  PWME_PWME3=1;
  PWME_PWME7=0;
  PWMDTY23=200-Y;                    //速度
}


/***************************************************
** 函数名称: main
** 功能描述: 主函数
** 说明: 
****************************************************/  
void main(void)
{
  DisableInterrupts;
  DDRA = 0X00;
  DDRB = 0xFF;
  PORTB= 0xFF; 
  PLL_Init();
    
  TIM_Init();
  PWM_Init() ;
  motor_f();      //【电机】
  EnableInterrupts;
  for(;;)
  { 
    if(t==40) 
    {
      t=0;
      TIE=0x00; 
   //////////////////////////////////////////////////
     
   
    erzhi();     //【二值化】
  
	tiaobian();  //【跳变采集】
 piancha();   //【偏差】
   
	 duoji();     //【舵机】
   
    /////////////////////////////////////////////////
      TIE=0x03; 
	 
    }
  } 
}

//---------------------中断定义---------------------
#pragma CODE_SEG NON_BANKED

/**************************************************       
** 函数名称: 中断处理函数
** 功能描述: 行中断处理函数
** 输    入: 无 
** 输    出: 无 
** 说明：  
***************************************************/ 
interrupt 8 void HREF_Count(void) 
{
  TFLG1_C0F = 1;
  m++;
  if ( m<6 || m>240 )       
  {
    return;//判断是否从新的一场开始
  } 

  
  Interval=6;
  if(m%Interval==0)
  {
   t++;
    
Image_Data[Line_C][0] = PORTA; 
  asm(nop);
Image_Data[Line_C][1] = PORTA;
  asm(nop);
Image_Data[Line_C][2] = PORTA;
  asm(nop);
Image_Data[Line_C][3] = PORTA;
  asm(nop);
Image_Data[Line_C][4] = PORTA;
  asm(nop);
Image_Data[Line_C][5] = PORTA;
  asm(nop);
Image_Data[Line_C][6] = PORTA;
  asm(nop);
Image_Data[Line_C][7] = PORTA;
  asm(nop);
Image_Data[Line_C][8] = PORTA;
  asm(nop);
Image_Data[Line_C][9] = PORTA;
  asm(nop);
Image_Data[Line_C][10] = PORTA;
  asm(nop);
Image_Data[Line_C][11] = PORTA;
  asm(nop);
Image_Data[Line_C][12] = PORTA;
  asm(nop);
Image_Data[Line_C][13] = PORTA;
  asm(nop);
Image_Data[Line_C][14] = PORTA;
  asm(nop);
Image_Data[Line_C][15] = PORTA;
  asm(nop);
Image_Data[Line_C][16] = PORTA;
  asm(nop);
Image_Data[Line_C][17] = PORTA;
  asm(nop);
Image_Data[Line_C][18] = PORTA;
  asm(nop);
Image_Data[Line_C][19] = PORTA;
  asm(nop);
Image_Data[Line_C][20] = PORTA;
  asm(nop);
Image_Data[Line_C][21] = PORTA;
  asm(nop);
Image_Data[Line_C][22] = PORTA;
  asm(nop);
Image_Data[Line_C][23] = PORTA;
  asm(nop);
Image_Data[Line_C][24] = PORTA;
  asm(nop);
Image_Data[Line_C][25] = PORTA;
  asm(nop);
Image_Data[Line_C][26] = PORTA;
  asm(nop);
Image_Data[Line_C][27] = PORTA;
  asm(nop);
Image_Data[Line_C][28] = PORTA;
  asm(nop);
Image_Data[Line_C][29] = PORTA;
  asm(nop);
Image_Data[Line_C][30] = PORTA;
  asm(nop);
Image_Data[Line_C][31] = PORTA;
  asm(nop);
Image_Data[Line_C][32] = PORTA;
  asm(nop);
Image_Data[Line_C][33] = PORTA;
  asm(nop);
Image_Data[Line_C][34] = PORTA;
  asm(nop);
Image_Data[Line_C][35] = PORTA;
  asm(nop);
Image_Data[Line_C][36] = PORTA;
  asm(nop);
Image_Data[Line_C][37] = PORTA;
  asm(nop);
Image_Data[Line_C][38] = PORTA;
  asm(nop);
Image_Data[Line_C][39] = PORTA;
  asm(nop);
Image_Data[Line_C][40] = PORTA;
  asm(nop);
Image_Data[Line_C][41] = PORTA;
  asm(nop);
Image_Data[Line_C][42] = PORTA;
  asm(nop);
Image_Data[Line_C][43] = PORTA;
  asm(nop);
Image_Data[Line_C][44] = PORTA;
  asm(nop);
Image_Data[Line_C][45] = PORTA;
  asm(nop);
Image_Data[Line_C][46] = PORTA;
  asm(nop);
Image_Data[Line_C][47] = PORTA;
  asm(nop);
Image_Data[Line_C][48] = PORTA;
  asm(nop);
Image_Data[Line_C][49] = PORTA;
  asm(nop);
Image_Data[Line_C][50] = PORTA;
  asm(nop);
Image_Data[Line_C][51] = PORTA;
  asm(nop);
Image_Data[Line_C][52] = PORTA;
  asm(nop);
Image_Data[Line_C][53] = PORTA;
  asm(nop);
Image_Data[Line_C][54] = PORTA;
  asm(nop);
Image_Data[Line_C][55] = PORTA;
  asm(nop);
Image_Data[Line_C][56] = PORTA;
  asm(nop);
Image_Data[Line_C][57] = PORTA;
  asm(nop);
Image_Data[Line_C][58] = PORTA;
  asm(nop);
Image_Data[Line_C][59] = PORTA;
  asm(nop);
Image_Data[Line_C][60] = PORTA;
  asm(nop);
Image_Data[Line_C][61] = PORTA;
  asm(nop);
Image_Data[Line_C][62] = PORTA;
  asm(nop);
Image_Data[Line_C][63] = PORTA;
  asm(nop);
Image_Data[Line_C][64] = PORTA;
  asm(nop);
Image_Data[Line_C][65] = PORTA;
  asm(nop);
Image_Data[Line_C][66] = PORTA;
  asm(nop);
Image_Data[Line_C][67] = PORTA;
  asm(nop);
Image_Data[Line_C][68] = PORTA;
  asm(nop);
Image_Data[Line_C][69] = PORTA;
  asm(nop);
Image_Data[Line_C][70] = PORTA;
  asm(nop);
Image_Data[Line_C][71] = PORTA;
  asm(nop);
Image_Data[Line_C][72] = PORTA;
  asm(nop);
Image_Data[Line_C][73] = PORTA;
  asm(nop);
Image_Data[Line_C][74] = PORTA;
  asm(nop);

    
    Line_C++;
  }
  

}

/************************************************** 
** 函数名称: 中断处理函数
** 功能描述: 场中断处理函数
** 输    入: 无 
** 输    出: 无 
** 说明：  
***************************************************/
interrupt 9 void VSYN_Interrupt(void)
{
  TFLG1_C1F = 1; //清场中断
  TFLG1_C0F = 1; //清行中断

  Line_C = 0; //行计数器
  
    
  
}


#pragma CODE_SEG DEFAULT