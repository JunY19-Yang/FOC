//改霍尔
//*****************************************************************************************************
#include "DSP28x_Project.h"     //   Device Headerfile and Examples Include File
#include "IQmathLib.h"
interrupt void cpu_timer0_isr(void);
interrupt void EPWM_1_INT(void);                         //epwm增强型脉冲宽度调制器
interrupt void SCIBRX_ISR(void);
interrupt void INT3_ISR(void);                          //中断服务例程声明
void Init_SiShu(void);
//*****************************************************************************************************
//全局变量定义与初始化
//***************************************************************************************************** 
float32 i=0;
float32 j=0;
float32 k=0;
Uint16 IsrTicker = 0;
Uint16 BackTicker = 0; //用于次数计数
Uint16 T1Period=0;     // T1定时器周期(Q0)
Uint16 T3Period = 0;   
float32 Modulation=0.25;    // 调制比
int16 MPeriod=0;
int32 Tmp=0;
_iq PolePairs=_IQ(5);      //极对数

//:::::::::::::::::::::::::::位置环变量定义:::::::::::::::::::::::::::
long PlaceError=0,Place_now=0, Now_P=0,//圈数
              OutPreSat_Place=0;//位置变量值定义
Uint16 PlaceSetBit=0;  //位置设定标志位
int32 	PosCount = 0;
float32 MfuncF1=0;
float32 MfuncF2=0;
float32 MfuncF3=0; 
//===============转子初始位置定位=============================  
Uint16 LocationFlag=1;
Uint16 LocationEnd=0; 
Uint16 Position=1;
Uint16 PositionPhase60=1;
Uint16 PositionPhase120=2;
Uint16 PositionPhase180=3; 
Uint16 PositionPhase240=4;
Uint16 PositionPhase300=5;
Uint16 PositionPhase360=6;      
//===============DAC模拟===================================== 
_iq DACTemp0=0;
_iq DACTemp1=0;
_iq DACTemp2=0; 
_iq MfuncC1=0;
_iq MfuncC2=0;
_iq MfuncC3=0;
Uint16 ZhengFan=1;                                      
//===============转子速度计算===================================== 
typedef struct                  //======ThreeHall.h====//        
    {
	    Uint16   HallUVW[3];      // 读取三个霍尔的对应状态
	    Uint16   Hall_State;      // 当前霍尔状态
	    Uint16   OldHall_State;   // 历史霍尔状态
	    Uint16   HallLX_State;    // 当前和历史霍尔状态联接一个字节数据
	    Uint16   Hall_num[8];     // 八种霍尔状态
	    _iq      Hall_angle[8];   // 八个霍尔角度根据不同状态
	    _iq      step_angle ;     //步进角度
	    _iq      step_angleFitter; // 步进角度滤波
	    Uint16   Poles;                 //电机极对数
	    _iq      Step_coeff;           //步进角系数
	    Uint16   Move_State;            //电机旋转状态
      _iq initial_angle;              // 电机初始角度            补偿角，试试（不同转速）
      _iq angleIQ;                    // 霍尔计算电机角度
      int16 Speed_RPM;                // 电机旋转速度
      Uint16 Speed_count;             // 判断停止计算和速度计算
      Uint16 Speed_countFitter;       // 速度计算的滤波
      Uint16 Speed_count_old;         // 速度计算历史的滤波
      _iq Speed_ele_angleIQ;          // 速度电角度值（计算速度）
      _iq Speed_ele_angleIQFitter;    // 速度电角度值（计算速度）
      _iq old_ele_angleIQ;            // 电机历史电角度
      _iq ele_angleIQ;                // 电机电角度
      _iq speed_coeff;
	   } Hall;
#define  Hall_DEFAULTS {0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0}  // 初始化参数
extern   Hall   Hall_Three;

//======ThreeHall.c ====//     
Hall         Hall_Three=Hall_DEFAULTS;
  _iq   HallK1=_IQ10(0.278);
  _iq   HallK2=_IQ10(0.722);

Uint16 SpeedLoopPrescaler = 10;     // 速度环判断周期
Uint16 SpeedLoopCount = 1;          // 速度环计数                              
_iq NewRawTheta=0;
_iq OldRawTheta=0; 
_iq SpeedRpm=0;                     //速度，单位：转/每分钟
Uint16 Hall_Fault=0;
_iq RawThetaTmp=0;
float32 SpeedRef=0;
_iq Speed=0;                        //速度，标幺值

// //===============转子角度计算===================================

Uint16 DirectionQep=0;               //转子旋转方向
_iq RawTheta=0;
_iq OldRawThetaPos = 0;
_iq TotalPulse=0; 
_iq MechTheta = 0;                   //机械角度，单位：度
_iq ElecTheta = 0;                   //电气角度，单位：度
_iq	AnglePU=0;                       //角度标幺化
_iq	Cosine=0;
_iq	Sine=0;

void  ThreeHallPara_init(void )
{
   Hall_Three.Hall_num[0]=6;                                 //513264  
   Hall_Three.Hall_num[1]=4;
   Hall_Three.Hall_num[2]=5;
   Hall_Three.Hall_num[3]=1;
   Hall_Three.Hall_num[4]=3;
   Hall_Three.Hall_num[5]=2;

   Hall_Three.Hall_angle[0] = _IQ(0.166666667);                    // 1/6      60         6
   Hall_Three.Hall_angle[1] = _IQ(0.333333333);                    // 2/6      120        4      正转
   Hall_Three.Hall_angle[2]=_IQ(0.5);                              //3/6      180         5
   Hall_Three.Hall_angle[3] = _IQ(0.666666667);                    // 4/6      240        1
   Hall_Three.Hall_angle[4] = _IQ(0.833333333);                    // 5/6      300        3
   Hall_Three.Hall_angle[5] = _IQ(1);                              // 6/6         360     2

   Hall_Three.Step_coeff=_IQ(0.0);                         //Hall_Three.Step_coeff步进系数
   Hall_Three.Poles=5;
   Hall_Three.initial_angle= _IQ(0.102);                //初始角度    0.102*360=36.72
   Hall_Three.speed_coeff=_IQ(0.002/Hall_Three.Poles);  // 2毫秒计算一次角度差值        速度系数= 0.002/极对数
}

//===============控制绕组电流计算============================ 
_iq ia=0;
_iq ib=0;
_iq ic=0;
_iq ialfa=0;
_iq ibeta=0; 
_iq id=0;
_iq iq=0; 

//===============PI控制器参数计算============================ 
_iq ID_Given=0;                                                     //给定d轴电流目标值
_iq ID_Ref=0;                                                       //处理过的、用于控制器的D轴电流参考值       
_iq ID_Fdb=0;                                                       //反馈值，通常来自电流传感器,从系统实际测量得到
_iq ID_Error=0;                                                     //ID_Ref - ID_Fdb

_iq ID_Up=0;                                                        //比例项,基于误差计算得到
_iq ID_Up1=0;                                                       //前一周期的比例项（可用于实现比例滤波器或其他逻辑）
_iq ID_Ui=0;                                                        //积分项,累积误差的长期影响
_iq ID_OutPreSat=0;                                                 //未考虑输出限制前的控制器输出。
_iq ID_SatError=0;                                                  //饱和误差，用于实现积分抗饱和控制
_iq ID_OutMax=_IQ(1);
_iq ID_OutMin=_IQ(-1);                                              //输出的最大和最小限制
_iq ID_Out=0;                                                       //最终的控制器输出，经过饱和处理。
 
_iq IQ_Given=0;
_iq IQ_Ref=0;
_iq IQ_Fdb=0;
_iq IQ_Error=0; 
 
_iq IQ_Up=0;
_iq IQ_Up1=0;
_iq IQ_Ui=0;
_iq IQ_OutPreSat=0;
_iq IQ_SatError=0;
_iq IQ_OutMax=_IQ(1);
_iq IQ_OutMin=_IQ(-1); 
_iq IQ_Out=0; 

_iq Speed_Given=_IQ(0.1); //速度给定    标幺值 0.2==>600RPM，最高转速1.0==>3000RPM
_iq Speed_Ref=0;                                                  //处理过的速度参考值
_iq Speed_Fdb=0;                                                  //速度反馈值，从系统实际测量得到
_iq Speed_Error=0;                                                //误差，计算为Speed_Ref - Speed_Fdb
 
_iq Speed_Up=0;                                                   //比例项
_iq Speed_Up1=0;                                                  //前一周期的比例项
_iq Speed_Ui=0;                                                   //积分项                 
_iq Speed_OutPreSat=0;                                            //未考虑输出限制前的控制器输出
_iq Speed_SatError=0;                                             //饱和误差
_iq Speed_OutMax=_IQ(0.99999);
_iq Speed_OutMin=-_IQ(0.99999);
_iq Speed_Out=0;  
Uint16 Speed_run=0;                                                //标志位，可能用来指示速度控制是否激活或在运行

//===============SVPWM计算==================================== 
Uint16 Sector = 0;                                                //扇区
_iq	Ualfa=0;  		                                              //静止坐标系列Ualfa, Ubeta         
_iq	Ubeta=0;		
_iq	Ud=0;		                                                   
_iq	Uq=0;	                                                      //旋转坐标系（d-q轴）  		
_iq	B0=0;			
_iq	B1=0;
_iq	B2=0;                                                         //用于计算在当前扇区内，电压矢量与相邻基本矢量的比较结果
_iq	X=0;
_iq	Y=0;
_iq	Z=0;                                                          //计算时的中间变量
_iq	t1=0;
_iq	t2=0;                                                         //在当前PWM周期内，两个主矢量（相邻的非零矢量）作用的时间
_iq	Ta=0;
_iq	Tb=0;
_iq	Tc=0;                                                         //三相电机中每一相的开关时间
_iq	MfuncD1=0;
_iq	MfuncD2=0;
_iq	MfuncD3=0;                                                    //表示三相中每一相调制函数的额外参数或调整值。
//===================================================================
Uint16 Run_PMSM=2;                                                //标识永磁同步电机（PMSM）是否在运行状态
float32 TEMP2=0;
_iq MechScaler=_IQ(0.0);                                           
_iq SpeedScaler=_IQ(0.00);                                        //机械角度和速度的标幺化
Uint16 speed_give=0;                                              //预设的速度给定值
Uint16 HallAngle=0;                                               //基于霍尔传感器测量的转子角度
Uint16 BuChang=416;
int16 TotalCnt=0;                                                 //总计数器
_iq RawCnt1=0;
_iq RawCnt2=0;                                                    //存储原始计数或其他传感器读数
Uint16 ShangDian_Err=0;                                           //可能表示上电错误或故障标志

//========================速度环PI参数=================================   simulink   _iq Speed_Kp = _IQ(300);    _iq Speed_Ki = _IQ(9000);
_iq Speed_Kp=_IQ(8);   
_iq Speed_Ki=_IQ(0.005);
//=====================================================================

//========================Q轴电流环PI参数==============================
_iq IQ_Kp=_IQ(0.3);
_iq IQ_Ki=_IQ(0.002);
// _iq IQ_Kp=_IQ(2.79999);    _iq IQ_Ki=_IQ(0.023);    0.093333 / 6 * 180 = 2.79999   0.00076767 / 6 * 180 = 0.023     20A/30A 打在示波器上
//=====================================================================

//========================D轴电流环PI参数==============================
_iq ID_Kp=_IQ(0.3);
_iq ID_Ki=_IQ(0.002);
// _iq ID_Kp = _IQ(2);    _iq ID_Ki = _IQ(0.023);       0.066667 / 6 * 180 = 2        0.00076767 / 6 * 180 = 0.023
    //=====================================================================

    //=====================参数设置========================================
float32 E_Ding_DianLiu = 4.2;       // 设置电机的额定电流的有效值  单位A
Uint16 BaseSpeed=3000;              //设置电机额定转速
Uint32 Udc_zero=2518;
                
void main(void)
{
   InitSysCtrl();
   InitGpio(); 
   Pwm_EN_1;
   DINT;                                                                            //DINT用于禁用中断
   InitPieCtrl(); 
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();                                                              //初始化中断处理系统和中断向量表
   EALLOW;  //写入受 EALLOW 保护的寄存器时需要使用此功能
  // PieVectTable.TINT0 = &cpu_timer0_isr; 
   PieVectTable.EPWM1_INT=&EPWM_1_INT;
   PieVectTable.SCIRXINTB= &SCIBRX_ISR;   //设置串口B接受中断的中断向量
   PieVectTable.XINT3=&INT3_ISR;
   EDIS;    //禁止写入受 EALLOW 保护的寄存器时需要使用此功能
 
 // InitCpuTimers(); 
   InitSci_C();
   InitSci_B();
   InitSpi();                                                                      //初始化串行通信接口和SPI接口
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);               //从FLASH复制函数到RAM，通常用于提高执行速度
   InitFlash();                                                                    //初始化闪存
   InitEPwm_1_2_3();//epwm初始化

   ThreeHallPara_init();// 根据电机霍尔计算位置角度参数初始化
  //  QEP_Init(); 
   Init_SiShu();
   ADC_Soc_Init();                                                                 //初始化ADC采样,用于读取传感器数据。
   eva_close();                                                                  //可能用于关闭或禁用电机驱动或其他硬件接口。
   Ad_CaiJi();                                  
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi();                                                                     //重复采样来稳定ADC读取的数据
   
   if(AD_BUF[7]<150)                                                               //条件性PWM使能:在通过ADC采集到的数据基础上决定是否使能PWM
    {
	     Pwm_EN_0;//允许PWM使能
    }
   else
   {
       Pwm_EN_1;//禁止PWM使能
	     ShangDian_Err=1;                                                               //设置上电错误标志
   }
    
    DELAY_US(1000000);
    
   IER |= M_INT3;                                                                 //中断使能寄存器,控制是否启用特定的中断
   IER |= M_INT9;                                       
   IER |= M_INT12;
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int                     //PieCtrlRegs是片上中断扩展的寄存器结构体，PIEIER是控制特定中断组使能的寄存器
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3                      //确保了如PWM、SCI、以及其他外部中断（如XINT3）能够被系统检测并响应。
    
   Init_lcd();
   EINT;   // Enable Global interrupt INTM         启用全局中断
   ERTM;   // Enable Global realtime interrupt DBGM    启用全局实时中断 
   for(;;)                          //无限循环用于持续执行一组特定的函数或任务，直到系统关闭或重启
     {     
        CPU_RUN();                  //CPU核心的维护操作
        DC_Link();                  //直流电源链路
        deal_key();                 //处理来自用户界面的按键输入
        LCD_DIS();                  //更新液晶显示屏（LCD）的显示内容
	    TX232_ChuLi();              //处理通过RS-232接口的数据传输
     }
}

interrupt void EPWM_1_INT(void)                              //中断服务例程的函数，专门响应与第一个EPWM通道相关的中断
{
    _iq t_01,t_02;
    IPM_BaoHu();                                          //电机保护逻辑的函数
    Show_time++;
    Show_time2++;                                            //时间追踪，计数器每次中断时自增
       if(Show_time2==1000)//1秒
    {
        Show_time2=0;
        lcd_dis_flag=1;
    }                                                        //重置计数器和设置lcd_dis_flag=1，后者可能用于触发LCD显示更新
                                                      
   Read_key();                                                //读取按键输入
   Ad_CaiJi();  
   JiSuan_Dl();                                              //计算电流
   JiSuan_AvgSpeed();                                        //计算平均转速
   JiSuan_AvgUDC();                                          //计算直流母线电压

      if(Run_PMSM==1&&IPM_Fault==0)                                //PMSM正在运行且没有逆变器模块（IPM）故障时
    {
        LuBo(ia, ib, ic,Hall_Three.Speed_RPM);  
        if(LocationFlag!=LocationEnd)                             //检查转子是否已经完成定位
          { 
            Modulation=0.95;
            HallAngle=0;
            if(GpioDataRegs.GPBDAT.bit.GPIO58) //W相       C
                  {
                              HallAngle+=1;  
                  }                                
            if(GpioDataRegs.GPBDAT.bit.GPIO52)//V相        B
                  {
                              HallAngle+=2;                               
                  }
            if(GpioDataRegs.GPBDAT.bit.GPIO49)//U相        A
                  {
                              HallAngle+=4;                              
                  }
              switch(HallAngle) 
                    {                          
                      case 6:
                        Position=PositionPhase60;
                        LocationFlag=LocationEnd;    
                        EQep1Regs.QPOSCNT =BuChang*0+BuChang/2;      
                        OldRawTheta=_IQ(EQep1Regs.QPOSCNT);         
                      break;
                      case 4:
                        Position=PositionPhase120;
                        LocationFlag=LocationEnd;
                        EQep1Regs.QPOSCNT =BuChang*5+BuChang/2; 
                        OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                      break;
                      case 5:
                        Position=PositionPhase180;
                        LocationFlag=LocationEnd;
                        EQep1Regs.QPOSCNT =BuChang*4+BuChang/2; 
                        OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                      break;
                      case 1:
                        Position=PositionPhase240;
                        LocationFlag=LocationEnd;
                        EQep1Regs.QPOSCNT =BuChang*3+BuChang/2; 
                        OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                      break;
                      case 3:
                        Position=PositionPhase300;
                        LocationFlag=LocationEnd;
                        EQep1Regs.QPOSCNT =BuChang*2+BuChang/2; 
                        OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                      break;
                      case 2:
                        Position=PositionPhase360;
                        LocationFlag=LocationEnd;
                        EQep1Regs.QPOSCNT =BuChang*1+BuChang/2;  
                        OldRawTheta=_IQ(EQep1Regs.QPOSCNT);
                      break;
                      default:
                              DC_ON_1;
                              Run_PMSM=2;
                              eva_close();
                              Hall_Fault=1;
                      break;
                    }   //switch  
          }     //定位结束
    //=====================================================================================================
    //初始位置定位结束，开始闭环控制
      else if(LocationFlag==LocationEnd)
         {  
    //霍尔换相计算电角度
            Hall_Three.HallUVW[0]=GpioDataRegs.GPBDAT.bit.GPIO58 ;   // C  W
            Hall_Three.HallUVW[1]=GpioDataRegs.GPBDAT.bit.GPIO52;     // B  v 
            Hall_Three.HallUVW[2]=GpioDataRegs.GPBDAT.bit.GPIO49 ;    //A  U

            Hall_Three.Hall_State = Hall_Three.HallUVW[0] +(Hall_Three.HallUVW[1]<<1) +(Hall_Three.HallUVW[2]<<2);        

             if ( Hall_Three.Hall_State!=Hall_Three.OldHall_State )                 
               {
                  Hall_Three.HallLX_State=Hall_Three.Hall_State + (Hall_Three.OldHall_State<<4) ;        
                      switch (Hall_Three.HallLX_State )
                            {
                            case 0x64:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.Hall_angle[1] - Hall_Three.angleIQ);
                                  // 60度+换向点的误差值，放大100倍       Hall_Three.angleIQ 转子的当前电角度      正传误差：  5的角度-当前
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 1;
                              }
                            break;

                            case 0x46:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.angleIQ - Hall_Three.Hall_angle[1]);
                                  // 反转误差：  当前-1的角度
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 2;
                              }
                            break;

                            case 0x45:                                                    
                              {
                                  Hall_Three.Step_coeff=( Hall_Three.Hall_angle[1]+Hall_Three.Hall_angle[2]- Hall_Three.angleIQ);                 
                                 
                                  if (Hall_Three.Step_coeff<_IQ(0.0))
                                    Hall_Three.Step_coeff+=_IQ(1.0);                                                                                 
                                  if (Hall_Three.Step_coeff>_IQ(1.0))                                                                          
                                      Hall_Three.Step_coeff-=_IQ(1.0);
                                  Hall_Three.Move_State=1;            //(正传)
                              }
                            break;

                            case 0x54:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.angleIQ - Hall_Three.Hall_angle[2]);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 2;
                              }
                            break;

                            case 0x51:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.Hall_angle[3] - Hall_Three.angleIQ);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 1; //(正传)
                              }
                            break;

                            case 0x15:           
                              {
                                  Hall_Three.Step_coeff=( Hall_Three.Hall_angle[1]+ Hall_Three.angleIQ-Hall_Three.Hall_angle[3]);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))                                        
                                    Hall_Three.Step_coeff+=_IQ(1.0);
                                  if (Hall_Three.Step_coeff>_IQ(1.0))
                                    Hall_Three.Step_coeff-=_IQ(1.0);
                                  Hall_Three.Move_State=2;          //反转
                              }
                            break;

                            case 0x13:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.Hall_angle[4] - Hall_Three.angleIQ);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 1;
                              }
                            break;

                            case  0x31:        
                              {
                                  Hall_Three.Step_coeff= (  Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[4] );                
                                  if (Hall_Three.Step_coeff<_IQ(0.0))
                                      Hall_Three.Step_coeff+=_IQ(1.0);
                                  if (Hall_Three.Step_coeff>_IQ(1.0))
                                      Hall_Three.Step_coeff-=_IQ(1.0);
                                  Hall_Three.Move_State=2;       // (反传)
                              }
                            break;

                            case 0x32:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.Hall_angle[5] - Hall_Three.angleIQ);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 1;
                              }
                            break;

                            case 0x23:
                              {
                                  Hall_Three.Step_coeff= (  Hall_Three.Hall_angle[1]+Hall_Three.angleIQ- Hall_Three.Hall_angle[5] );
                                  if (Hall_Three.Step_coeff<_IQ(0.0))
                                    Hall_Three.Step_coeff+=_IQ(1.0);
                                  if (Hall_Three.Step_coeff>_IQ(1.0))
                                    Hall_Three.Step_coeff-=_IQ(1.0);
                                  Hall_Three.Move_State=2;
                              }
                              break;

                              case 0x26:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.Hall_angle[0] - Hall_Three.angleIQ);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 1;
                              }
                              break;

                              case 0x62:
                              {
                                  Hall_Three.Step_coeff = (Hall_Three.Hall_angle[1] + Hall_Three.angleIQ - Hall_Three.Hall_angle[0]);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))
                                    Hall_Three.Step_coeff += _IQ(1.0);
                                  if (Hall_Three.Step_coeff > _IQ(1.0))
                                    Hall_Three.Step_coeff -= _IQ(1.0);
                                  Hall_Three.Move_State = 2;
                              }
                              break;
                                 
                              default:
                              {
                                  Hall_Three.ele_angleIQ=0;                       
                              }
                              break;
                            } // switch  
              // 使用一阶低通滤波器更新滤波后的PWM计数
                Hall_Three.Speed_countFitter = _IQ10mpy(HallK2, Hall_Three.Speed_countFitter) + _IQ10mpy(HallK1, Hall_Three.Speed_count);
              // 存储滤波后的pmw计数为旧值
                Hall_Three.Speed_count_old = Hall_Three.Speed_countFitter;
              // 计算电机的步进角用二个连续霍尔状态间角度变化除此之间时间，就是每一个 PWM 的角度变化量   w=60°/（NT）
                Hall_Three.step_angle = Hall_Three.Step_coeff / Hall_Three.Speed_count_old;              
              // 使用一阶低通滤波器滤波步进角
                Hall_Three.step_angleFitter = _IQ10mpy(HallK2, Hall_Three.step_angleFitter) + _IQ10mpy(HallK1, Hall_Three.step_angle);
              // 重置速度计数器为0
                Hall_Three.Speed_count = 0;
                           
               }//霍尔状态变化
               
            else  if ( Hall_Three.Hall_State==Hall_Three.OldHall_State )               
              {
                Hall_Three.Speed_count++;
                if( Hall_Three.Speed_count>=2000 )  // 超时阈值
                    {
                      Hall_Three.Move_State=1;                                         //正转
                      Hall_Three.Speed_count=0;                                        //重置速度计数器
                      Hall_Three.Speed_RPM= 0 ;                                        //将转速设置为0，因为没有检测到运动。
                      Hall_Three.step_angle=0 ;                                        //角度变化量和其滤波值均重置，缺少新的角度变化数据。
                      Hall_Three.step_angleFitter=0;
                      switch (Hall_Three.Hall_State )
                            {
                                case 0x6:
                                {
                                Hall_Three.angleIQ = Hall_Three.Hall_angle[0]+_IQ(0.083333333);                 //_IQ(0.083333333)提供一个小的、固定的偏移量   0.083333333=1/12   为30°        
                                }
                                 break;
                                case 0x4:
                                {
                                  Hall_Three.angleIQ = Hall_Three.Hall_angle[1]+_IQ(0.083333333);
                                }
                                 break;
                                case 0x5:
                                {
                                Hall_Three.angleIQ = Hall_Three.Hall_angle[2]+_IQ(0.083333333);
                                }
                                break;
                                case 0x1:
                                {
                                  Hall_Three.angleIQ = Hall_Three.Hall_angle[3]+_IQ(0.083333333);
                                }
                                break;
                                case 0x3:
                                {
                                Hall_Three.angleIQ = Hall_Three.Hall_angle[4]+_IQ(0.083333333);
                                }
                                break;
                                 case 0x2:
                                {
                                  Hall_Three.angleIQ = Hall_Three.Hall_angle[5]+_IQ(0.083333333);
                                }
                                break;
                                default:
                                {
                                  Hall_Three.ele_angleIQ=0;
                                }
                                break;
                            } //switch
                     }	//计数器重置
              }   //霍尔状态未变化 

        if ( Hall_Three.Move_State==1 )              //正转
        {
          Hall_Three.angleIQ = Hall_Three.angleIQ + Hall_Three.step_angleFitter ;
        }
        else if ( Hall_Three.Move_State==2 )       //反转
        {
          Hall_Three.angleIQ = Hall_Three.angleIQ - Hall_Three.step_angleFitter;
        }

      if( Hall_Three.angleIQ>_IQ(1.0))
        Hall_Three.angleIQ-=_IQ(1.0);
      else if( Hall_Three.angleIQ<_IQ(0.0))
        Hall_Three.angleIQ+=_IQ(1.0);

  // d轴电机磁极位置和电机位置,A相霍尔上升沿与A相反电动势下降0点的时间角度差
      Hall_Three.ele_angleIQ = Hall_Three.angleIQ - Hall_Three.initial_angle ;        
        if( Hall_Three.ele_angleIQ>_IQ(1.0))
          Hall_Three.ele_angleIQ-=_IQ(1.0);
        else if( Hall_Three.ele_angleIQ<_IQ(0.0))
          Hall_Three.ele_angleIQ+=_IQ(1.0);
      Hall_Three.OldHall_State=Hall_Three.Hall_State ;
              
      Sine = _IQsinPU(Hall_Three.ele_angleIQ);
      Cosine = _IQcosPU(Hall_Three.ele_angleIQ);

//======================================================================================================
//霍尔速度计算
//======================================================================================================
    if( Hall_Three.Move_State==1)    // 判断计算正转的2ms的角度变化量
    {
        Hall_Three.Speed_ele_angleIQ =Hall_Three.ele_angleIQ -Hall_Three.old_ele_angleIQ ;       //电机在2ms时间计算角度变化量。 
        if( Hall_Three.Speed_ele_angleIQ <_IQ(0.0) )
          Hall_Three.Speed_ele_angleIQ+= _IQ(1.0);                                                 // 防止超过16777216.0和小于0，   16777216=2^24 
    }
    else  if( Hall_Three.Move_State==2) //  判断计算反转的2ms的角度变化量，
    {
        Hall_Three.Speed_ele_angleIQ =Hall_Three.old_ele_angleIQ -Hall_Three.ele_angleIQ;
        if( Hall_Three.Speed_ele_angleIQ <_IQ(0.0) )
          Hall_Three.Speed_ele_angleIQ+= _IQ(1.0);
    }
    Hall_Three.Speed_ele_angleIQFitter= _IQ10mpy(HallK1, Hall_Three.Speed_ele_angleIQFitter)+_IQ10mpy(HallK2,  Hall_Three.Speed_ele_angleIQ);  //差值一阶滤波   
    Hall_Three.Speed_RPM = _IQmpy(Hall_Three.Speed_ele_angleIQFitter,Hall_Three.speed_coeff); // 插值变化量*系数=速度。   

//=================速度环PI===================================
    			Speed_Ref=_IQ(SpeedRef);                                            //转换为定点数格式
    			Speed_Fdb=Hall_Three.Speed_RPM;                                                    //将速度赋给Speed_Fdb
    			Speed_Error=Speed_Ref - Speed_Fdb;                                  //速度参考值和反馈值之间的差值，即速度误差。
    			Speed_Up=_IQmpy(Speed_Kp,Speed_Error);                              //计算比例项，使用比例增益Speed_Kp乘以速度误差
    			Speed_Ui=Speed_Ui + _IQmpy(Speed_Ki,Speed_Up) + _IQmpy(Speed_Ki,Speed_SatError);         //计算积分项，累加之前的积分值、当前周期的比例项和饱和误差的调整。
    			Speed_OutPreSat=Speed_Up+Speed_Ui;                                  //计算未进行饱和处理的控制输出
    			if(Speed_OutPreSat>Speed_OutMax)                                    //饱和处理确保控制输出不会超过设定的最大和最小值
    				Speed_Out=Speed_OutMax;
    			else if(Speed_OutPreSat<Speed_OutMin)
    	 			Speed_Out=Speed_OutMin;
    			else
    				Speed_Out=Speed_OutPreSat;
    			Speed_SatError=Speed_Out-Speed_OutPreSat;                            //计算饱和误差，用于调整下一个控制周期的积分项，防止积分饱和。
    			IQ_Given=Speed_Out;
    
    //Clark变换计算
    ialfa=ia;                                                                //直接取相电流ia作为α轴电流。
    ibeta=_IQmpy(ia,_IQ(0.57735026918963))+_IQmpy(ib,_IQ(1.15470053837926)); //计算β轴电流，基于Clark变换公式
    //Park变换计算
    id = _IQmpy(ialfa,Cosine) +_IQmpy(ibeta,Sine);
    iq = _IQmpy(ibeta,Cosine)- _IQmpy(ialfa,Sine);

//======================================================================================================
//IQ电流PID调节控制, 对比simulink仿真
//======================================================================================================  
        IQ_Ref=IQ_Given;                                                          //将速度环输出的电流指令值赋给Q轴电流的参考值
        IQ_Fdb=iq;                                                                //从系统反馈中获取实际的Q轴电流值
        IQ_Error=IQ_Ref-IQ_Fdb;                                                   //计算Q轴电流的控制误差，即参考值和反馈值之间的差。

        IQ_Up=_IQmpy(IQ_Kp,IQ_Error);                                             //计算比例控制项，使用比例增益IQ_Kp乘以误差。
        IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up) + _IQmpy(IQ_Ki,IQ_SatError);            //更新积分项，其中包括基于误差的积分更新和考虑前一周期的饱和误差（IQ_SatError）来调整积分项，防止积分饱和。

        IQ_OutPreSat=IQ_Up+IQ_Ui;                                                 //计算未进行饱和处理前的PID输出。

        if(IQ_OutPreSat>IQ_OutMax)
          IQ_Out=IQ_OutMax;
        else if(IQ_OutPreSat<IQ_OutMin)
          IQ_Out=IQ_OutMin;
        else 
          IQ_Out=IQ_OutPreSat;  

        IQ_SatError=IQ_Out-IQ_OutPreSat;                                           //计算由于饱和限制导致的误差，用于下一个控制周期的积分调整
        Uq=IQ_Out;                                                                 //将计算得到的Q轴电流控制输出赋值给逆变器的Q轴电压指令Uq
//======================================================================================================
//ID电流PID调节控制
//======================================================================================================  
        ID_Ref=ID_Given;                                                          
        ID_Fdb=id;
        ID_Error=ID_Ref-ID_Fdb;

        ID_Up=_IQmpy(ID_Kp,ID_Error);    
        ID_Ui=ID_Ui+_IQmpy(ID_Ki,ID_Up)+_IQmpy(ID_Ki,ID_SatError);   

        ID_OutPreSat=ID_Up+ID_Ui;    
        if(ID_OutPreSat>ID_OutMax)   
          ID_Out=ID_OutMax;
        else if(ID_OutPreSat<ID_OutMin)
          ID_Out=ID_OutMin;
        else 
          ID_Out=ID_OutPreSat;  

        ID_SatError=ID_Out-ID_OutPreSat;     
        Ud=ID_Out;           
//======================================================================================================
//IPark变换（逆Park变换）
//====================================================================================================== 
        Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
        Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine);        
//======================================================================================================
//SVPWM实现
//====================================================================================================== 
        B0=Ubeta;
        B1=_IQmpy(_IQ(0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2 
        B2=_IQmpy(_IQ(-0.8660254),Ualfa)- _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2

        Sector=0;
        if(B0>_IQ(0)) Sector =1;
        if(B1>_IQ(0)) Sector =Sector +2;
        if(B2>_IQ(0)) Sector =Sector +4; 

        X=Ubeta;//va
        Y=_IQmpy(_IQ(0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta);// 0.8660254 = sqrt(3)/2 vb
        Z=_IQmpy(_IQ(-0.8660254),Ualfa)+ _IQmpy(_IQ(0.5),Ubeta); // 0.8660254 = sqrt(3)/2 vc
      
      if(Sector==1)
        {
          t_01=Z;
          t_02=Y;

          if((t_01+t_02)>_IQ(1))
          {
            t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
          t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

          }
          else
          { t1=t_01;
          t2=t_02;
          }

          Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
          Ta=Tb+t1;
          Tc=Ta+t2;
        }
        else if(Sector==2)
        {
          t_01=Y;
          t_02=-X;
                if((t_01+t_02)>_IQ(1))
          {
            t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
          t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

          }
          else
          { t1=t_01;
          t2=t_02;
          }

          Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
          Tc=Ta+t1;
          Tb=Tc+t2;
        } 
        else if(Sector==3)
          {
          t_01=-Z;
          t_02=X;

                if((t_01+t_02)>_IQ(1))
          {
            t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
          t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

          }
          else
          { t1=t_01;
          t2=t_02;
          }

          Ta=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
          Tb=Ta+t1;
          Tc=Tb+t2;	
          } 
          else if(Sector==4)
          {
          t_01=-X;
          t_02=Z;
                if((t_01+t_02)>_IQ(1))
          {
            t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
          t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

          }
          else
          { t1=t_01;
          t2=t_02;
          }

          Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
          Tb=Tc+t1;
          Ta=Tb+t2;
          } 
          else if(Sector==5)
          {
          t_01=X;
          t_02=-Y;
                if((t_01+t_02)>_IQ(1))
          {
            t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
          t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

          }
          else
          { t1=t_01;
          t2=t_02;
          }

          Tb=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
          Tc=Tb+t1;
          Ta=Tc+t2;
        }
        else if(Sector==6)
        {
          t_01=-Y;
          t_02=-Z;
                if((t_01+t_02)>_IQ(1))
          {
            t1=_IQmpy(_IQdiv(t_01, (t_01+t_02)),_IQ(1));
          t2=_IQmpy(_IQdiv(t_02, (t_01+t_02)),_IQ(1));

          }
          else
          { t1=t_01;
          t2=t_02;
          }

          Tc=_IQmpy(_IQ(0.5),(_IQ(1)-t1-t2));
          Ta=Tc+t1;
          Tb=Ta+t2;
        } 
        MfuncD1=_IQmpy(_IQ(2),(_IQ(0.5)-Ta));
        MfuncD2=_IQmpy(_IQ(2),(_IQ(0.5)-Tb));
        MfuncD3=_IQmpy(_IQ(2),(_IQ(0.5)-Tc)); 

    //======================================================================================================
    //EVA全比较器参数赋值，用于驱动电机
    //====================================================================================================== 
      MPeriod = (int16)(T1Period * Modulation);              // Q0 = (Q0 * Q0)

      Tmp = (int32)MPeriod * (int32)MfuncD1;                    // Q15 = Q0*Q15，计算全比较器CMPR1赋值
      EPwm1Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

      Tmp = (int32)MPeriod * (int32)MfuncD2;                    // Q15 = Q0*Q15，计算全比较器CMPR2赋值
      EPwm2Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

      Tmp = (int32)MPeriod * (int32)MfuncD3;                    // Q15 = Q0*Q15，计算全比较器CMPR3赋值
      EPwm3Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2) 
      
      }   //  闭环控制的
    }   //电机正常运行       
               
    if(DC_ON_flag==1)                                              //控制标志，用来指示系统电源是否在正常供电状态。
    {
            if(U_dc_dis<10)//执行停机命令          //如果直流电压低于10伏，可能需要执行停机命令。       if(U_dc_dis>10)//去掉上电保护   Run_PMSM=2初值是2；
            {
            eva_close();
            Run_PMSM=2;
            DC_ON_flag=0;
            }  
    }

    EPwm1Regs.ETCLR.bit.INT=1;//清除中断标志位
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
                    
}    //EPWM中断结束


interrupt void SCIBRX_ISR(void)     // SCI-B
{
	PieCtrlRegs.PIEACK.bit.ACK9 = 1;

}
void Init_SiShu(void)
{ 
 GuoliuZhi=7*E_Ding_DianLiu;
 E_Ding_DianLiu=1.414*E_Ding_DianLiu;
}

interrupt void INT3_ISR(void)
{ 
PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
}

