//�Ļ���
//*****************************************************************************************************
#include "DSP28x_Project.h"     //   Device Headerfile and Examples Include File
#include "IQmathLib.h"
interrupt void cpu_timer0_isr(void);
interrupt void EPWM_1_INT(void);                         //epwm��ǿ�������ȵ�����
interrupt void SCIBRX_ISR(void);
interrupt void INT3_ISR(void);                          //�жϷ�����������
void Init_SiShu(void);
//*****************************************************************************************************
//ȫ�ֱ����������ʼ��
//***************************************************************************************************** 
float32 i=0;
float32 j=0;
float32 k=0;
Uint16 IsrTicker = 0;
Uint16 BackTicker = 0; //���ڴ�������
Uint16 T1Period=0;     // T1��ʱ������(Q0)
Uint16 T3Period = 0;   
float32 Modulation=0.25;    // ���Ʊ�
int16 MPeriod=0;
int32 Tmp=0;
_iq PolePairs=_IQ(5);      //������

//:::::::::::::::::::::::::::λ�û���������:::::::::::::::::::::::::::
long PlaceError=0,Place_now=0, Now_P=0,//Ȧ��
              OutPreSat_Place=0;//λ�ñ���ֵ����
Uint16 PlaceSetBit=0;  //λ���趨��־λ
int32 	PosCount = 0;
float32 MfuncF1=0;
float32 MfuncF2=0;
float32 MfuncF3=0; 
//===============ת�ӳ�ʼλ�ö�λ=============================  
Uint16 LocationFlag=1;
Uint16 LocationEnd=0; 
Uint16 Position=1;
Uint16 PositionPhase60=1;
Uint16 PositionPhase120=2;
Uint16 PositionPhase180=3; 
Uint16 PositionPhase240=4;
Uint16 PositionPhase300=5;
Uint16 PositionPhase360=6;      
//===============DACģ��===================================== 
_iq DACTemp0=0;
_iq DACTemp1=0;
_iq DACTemp2=0; 
_iq MfuncC1=0;
_iq MfuncC2=0;
_iq MfuncC3=0;
Uint16 ZhengFan=1;                                      
//===============ת���ٶȼ���===================================== 
typedef struct                  //======ThreeHall.h====//        
    {
	    Uint16   HallUVW[3];      // ��ȡ���������Ķ�Ӧ״̬
	    Uint16   Hall_State;      // ��ǰ����״̬
	    Uint16   OldHall_State;   // ��ʷ����״̬
	    Uint16   HallLX_State;    // ��ǰ����ʷ����״̬����һ���ֽ�����
	    Uint16   Hall_num[8];     // ���ֻ���״̬
	    _iq      Hall_angle[8];   // �˸������Ƕȸ��ݲ�ͬ״̬
	    _iq      step_angle ;     //�����Ƕ�
	    _iq      step_angleFitter; // �����Ƕ��˲�
	    Uint16   Poles;                 //���������
	    _iq      Step_coeff;           //������ϵ��
	    Uint16   Move_State;            //�����ת״̬
      _iq initial_angle;              // �����ʼ�Ƕ�            �����ǣ����ԣ���ͬת�٣�
      _iq angleIQ;                    // �����������Ƕ�
      int16 Speed_RPM;                // �����ת�ٶ�
      Uint16 Speed_count;             // �ж�ֹͣ������ٶȼ���
      Uint16 Speed_countFitter;       // �ٶȼ�����˲�
      Uint16 Speed_count_old;         // �ٶȼ�����ʷ���˲�
      _iq Speed_ele_angleIQ;          // �ٶȵ�Ƕ�ֵ�������ٶȣ�
      _iq Speed_ele_angleIQFitter;    // �ٶȵ�Ƕ�ֵ�������ٶȣ�
      _iq old_ele_angleIQ;            // �����ʷ��Ƕ�
      _iq ele_angleIQ;                // �����Ƕ�
      _iq speed_coeff;
	   } Hall;
#define  Hall_DEFAULTS {0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0}  // ��ʼ������
extern   Hall   Hall_Three;

//======ThreeHall.c ====//     
Hall         Hall_Three=Hall_DEFAULTS;
  _iq   HallK1=_IQ10(0.278);
  _iq   HallK2=_IQ10(0.722);

Uint16 SpeedLoopPrescaler = 10;     // �ٶȻ��ж�����
Uint16 SpeedLoopCount = 1;          // �ٶȻ�����                              
_iq NewRawTheta=0;
_iq OldRawTheta=0; 
_iq SpeedRpm=0;                     //�ٶȣ���λ��ת/ÿ����
Uint16 Hall_Fault=0;
_iq RawThetaTmp=0;
float32 SpeedRef=0;
_iq Speed=0;                        //�ٶȣ�����ֵ

// //===============ת�ӽǶȼ���===================================

Uint16 DirectionQep=0;               //ת����ת����
_iq RawTheta=0;
_iq OldRawThetaPos = 0;
_iq TotalPulse=0; 
_iq MechTheta = 0;                   //��е�Ƕȣ���λ����
_iq ElecTheta = 0;                   //�����Ƕȣ���λ����
_iq	AnglePU=0;                       //�Ƕȱ��ۻ�
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
   Hall_Three.Hall_angle[1] = _IQ(0.333333333);                    // 2/6      120        4      ��ת
   Hall_Three.Hall_angle[2]=_IQ(0.5);                              //3/6      180         5
   Hall_Three.Hall_angle[3] = _IQ(0.666666667);                    // 4/6      240        1
   Hall_Three.Hall_angle[4] = _IQ(0.833333333);                    // 5/6      300        3
   Hall_Three.Hall_angle[5] = _IQ(1);                              // 6/6         360     2

   Hall_Three.Step_coeff=_IQ(0.0);                         //Hall_Three.Step_coeff����ϵ��
   Hall_Three.Poles=5;
   Hall_Three.initial_angle= _IQ(0.102);                //��ʼ�Ƕ�    0.102*360=36.72
   Hall_Three.speed_coeff=_IQ(0.002/Hall_Three.Poles);  // 2�������һ�νǶȲ�ֵ        �ٶ�ϵ��= 0.002/������
}

//===============���������������============================ 
_iq ia=0;
_iq ib=0;
_iq ic=0;
_iq ialfa=0;
_iq ibeta=0; 
_iq id=0;
_iq iq=0; 

//===============PI��������������============================ 
_iq ID_Given=0;                                                     //����d�����Ŀ��ֵ
_iq ID_Ref=0;                                                       //������ġ����ڿ�������D������ο�ֵ       
_iq ID_Fdb=0;                                                       //����ֵ��ͨ�����Ե���������,��ϵͳʵ�ʲ����õ�
_iq ID_Error=0;                                                     //ID_Ref - ID_Fdb

_iq ID_Up=0;                                                        //������,����������õ�
_iq ID_Up1=0;                                                       //ǰһ���ڵı����������ʵ�ֱ����˲����������߼���
_iq ID_Ui=0;                                                        //������,�ۻ����ĳ���Ӱ��
_iq ID_OutPreSat=0;                                                 //δ�����������ǰ�Ŀ����������
_iq ID_SatError=0;                                                  //����������ʵ�ֻ��ֿ����Ϳ���
_iq ID_OutMax=_IQ(1);
_iq ID_OutMin=_IQ(-1);                                              //�����������С����
_iq ID_Out=0;                                                       //���յĿ�����������������ʹ���
 
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

_iq Speed_Given=_IQ(0.1); //�ٶȸ���    ����ֵ 0.2==>600RPM�����ת��1.0==>3000RPM
_iq Speed_Ref=0;                                                  //��������ٶȲο�ֵ
_iq Speed_Fdb=0;                                                  //�ٶȷ���ֵ����ϵͳʵ�ʲ����õ�
_iq Speed_Error=0;                                                //������ΪSpeed_Ref - Speed_Fdb
 
_iq Speed_Up=0;                                                   //������
_iq Speed_Up1=0;                                                  //ǰһ���ڵı�����
_iq Speed_Ui=0;                                                   //������                 
_iq Speed_OutPreSat=0;                                            //δ�����������ǰ�Ŀ��������
_iq Speed_SatError=0;                                             //�������
_iq Speed_OutMax=_IQ(0.99999);
_iq Speed_OutMin=-_IQ(0.99999);
_iq Speed_Out=0;  
Uint16 Speed_run=0;                                                //��־λ����������ָʾ�ٶȿ����Ƿ񼤻��������

//===============SVPWM����==================================== 
Uint16 Sector = 0;                                                //����
_iq	Ualfa=0;  		                                              //��ֹ����ϵ��Ualfa, Ubeta         
_iq	Ubeta=0;		
_iq	Ud=0;		                                                   
_iq	Uq=0;	                                                      //��ת����ϵ��d-q�ᣩ  		
_iq	B0=0;			
_iq	B1=0;
_iq	B2=0;                                                         //���ڼ����ڵ�ǰ�����ڣ���ѹʸ�������ڻ���ʸ���ıȽϽ��
_iq	X=0;
_iq	Y=0;
_iq	Z=0;                                                          //����ʱ���м����
_iq	t1=0;
_iq	t2=0;                                                         //�ڵ�ǰPWM�����ڣ�������ʸ�������ڵķ���ʸ�������õ�ʱ��
_iq	Ta=0;
_iq	Tb=0;
_iq	Tc=0;                                                         //��������ÿһ��Ŀ���ʱ��
_iq	MfuncD1=0;
_iq	MfuncD2=0;
_iq	MfuncD3=0;                                                    //��ʾ������ÿһ����ƺ����Ķ�����������ֵ��
//===================================================================
Uint16 Run_PMSM=2;                                                //��ʶ����ͬ�������PMSM���Ƿ�������״̬
float32 TEMP2=0;
_iq MechScaler=_IQ(0.0);                                           
_iq SpeedScaler=_IQ(0.00);                                        //��е�ǶȺ��ٶȵı��ۻ�
Uint16 speed_give=0;                                              //Ԥ����ٶȸ���ֵ
Uint16 HallAngle=0;                                               //���ڻ���������������ת�ӽǶ�
Uint16 BuChang=416;
int16 TotalCnt=0;                                                 //�ܼ�����
_iq RawCnt1=0;
_iq RawCnt2=0;                                                    //�洢ԭʼ��������������������
Uint16 ShangDian_Err=0;                                           //���ܱ�ʾ�ϵ�������ϱ�־

//========================�ٶȻ�PI����=================================   simulink   _iq Speed_Kp = _IQ(300);    _iq Speed_Ki = _IQ(9000);
_iq Speed_Kp=_IQ(8);   
_iq Speed_Ki=_IQ(0.005);
//=====================================================================

//========================Q�������PI����==============================
_iq IQ_Kp=_IQ(0.3);
_iq IQ_Ki=_IQ(0.002);
// _iq IQ_Kp=_IQ(2.79999);    _iq IQ_Ki=_IQ(0.023);    0.093333 / 6 * 180 = 2.79999   0.00076767 / 6 * 180 = 0.023     20A/30A ����ʾ������
//=====================================================================

//========================D�������PI����==============================
_iq ID_Kp=_IQ(0.3);
_iq ID_Ki=_IQ(0.002);
// _iq ID_Kp = _IQ(2);    _iq ID_Ki = _IQ(0.023);       0.066667 / 6 * 180 = 2        0.00076767 / 6 * 180 = 0.023
    //=====================================================================

    //=====================��������========================================
float32 E_Ding_DianLiu = 4.2;       // ���õ���Ķ��������Чֵ  ��λA
Uint16 BaseSpeed=3000;              //���õ���ת��
Uint32 Udc_zero=2518;
                
void main(void)
{
   InitSysCtrl();
   InitGpio(); 
   Pwm_EN_1;
   DINT;                                                                            //DINT���ڽ����ж�
   InitPieCtrl(); 
   IER = 0x0000;
   IFR = 0x0000;
   InitPieVectTable();                                                              //��ʼ���жϴ���ϵͳ���ж�������
   EALLOW;  //д���� EALLOW �����ļĴ���ʱ��Ҫʹ�ô˹���
  // PieVectTable.TINT0 = &cpu_timer0_isr; 
   PieVectTable.EPWM1_INT=&EPWM_1_INT;
   PieVectTable.SCIRXINTB= &SCIBRX_ISR;   //���ô���B�����жϵ��ж�����
   PieVectTable.XINT3=&INT3_ISR;
   EDIS;    //��ֹд���� EALLOW �����ļĴ���ʱ��Ҫʹ�ô˹���
 
 // InitCpuTimers(); 
   InitSci_C();
   InitSci_B();
   InitSpi();                                                                      //��ʼ������ͨ�Žӿں�SPI�ӿ�
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);               //��FLASH���ƺ�����RAM��ͨ���������ִ���ٶ�
   InitFlash();                                                                    //��ʼ������
   InitEPwm_1_2_3();//epwm��ʼ��

   ThreeHallPara_init();// ���ݵ����������λ�ýǶȲ�����ʼ��
  //  QEP_Init(); 
   Init_SiShu();
   ADC_Soc_Init();                                                                 //��ʼ��ADC����,���ڶ�ȡ���������ݡ�
   eva_close();                                                                  //�������ڹرջ���õ������������Ӳ���ӿڡ�
   Ad_CaiJi();                                  
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi(); 
   Ad_CaiJi();                                                                     //�ظ��������ȶ�ADC��ȡ������
   
   if(AD_BUF[7]<150)                                                               //������PWMʹ��:��ͨ��ADC�ɼ��������ݻ����Ͼ����Ƿ�ʹ��PWM
    {
	     Pwm_EN_0;//����PWMʹ��
    }
   else
   {
       Pwm_EN_1;//��ֹPWMʹ��
	     ShangDian_Err=1;                                                               //�����ϵ�����־
   }
    
    DELAY_US(1000000);
    
   IER |= M_INT3;                                                                 //�ж�ʹ�ܼĴ���,�����Ƿ������ض����ж�
   IER |= M_INT9;                                       
   IER |= M_INT12;
   //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;//timer0
   PieCtrlRegs.PIEIER3.bit.INTx1=1;//epwm1int                     //PieCtrlRegs��Ƭ���ж���չ�ļĴ����ṹ�壬PIEIER�ǿ����ض��ж���ʹ�ܵļĴ���
   PieCtrlRegs.PIEIER9.bit.INTx3=1;//scib
   PieCtrlRegs.PIEIER12.bit.INTx1=1;//xint3                      //ȷ������PWM��SCI���Լ������ⲿ�жϣ���XINT3���ܹ���ϵͳ��Ⲣ��Ӧ��
    
   Init_lcd();
   EINT;   // Enable Global interrupt INTM         ����ȫ���ж�
   ERTM;   // Enable Global realtime interrupt DBGM    ����ȫ��ʵʱ�ж� 
   for(;;)                          //����ѭ�����ڳ���ִ��һ���ض��ĺ���������ֱ��ϵͳ�رջ�����
     {     
        CPU_RUN();                  //CPU���ĵ�ά������
        DC_Link();                  //ֱ����Դ��·
        deal_key();                 //���������û�����İ�������
        LCD_DIS();                  //����Һ����ʾ����LCD������ʾ����
	    TX232_ChuLi();              //����ͨ��RS-232�ӿڵ����ݴ���
     }
}

interrupt void EPWM_1_INT(void)                              //�жϷ������̵ĺ�����ר����Ӧ���һ��EPWMͨ����ص��ж�
{
    _iq t_01,t_02;
    IPM_BaoHu();                                          //��������߼��ĺ���
    Show_time++;
    Show_time2++;                                            //ʱ��׷�٣�������ÿ���ж�ʱ����
       if(Show_time2==1000)//1��
    {
        Show_time2=0;
        lcd_dis_flag=1;
    }                                                        //���ü�����������lcd_dis_flag=1�����߿������ڴ���LCD��ʾ����
                                                      
   Read_key();                                                //��ȡ��������
   Ad_CaiJi();  
   JiSuan_Dl();                                              //�������
   JiSuan_AvgSpeed();                                        //����ƽ��ת��
   JiSuan_AvgUDC();                                          //����ֱ��ĸ�ߵ�ѹ

      if(Run_PMSM==1&&IPM_Fault==0)                                //PMSM����������û�������ģ�飨IPM������ʱ
    {
        LuBo(ia, ib, ic,Hall_Three.Speed_RPM);  
        if(LocationFlag!=LocationEnd)                             //���ת���Ƿ��Ѿ���ɶ�λ
          { 
            Modulation=0.95;
            HallAngle=0;
            if(GpioDataRegs.GPBDAT.bit.GPIO58) //W��       C
                  {
                              HallAngle+=1;  
                  }                                
            if(GpioDataRegs.GPBDAT.bit.GPIO52)//V��        B
                  {
                              HallAngle+=2;                               
                  }
            if(GpioDataRegs.GPBDAT.bit.GPIO49)//U��        A
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
          }     //��λ����
    //=====================================================================================================
    //��ʼλ�ö�λ��������ʼ�ջ�����
      else if(LocationFlag==LocationEnd)
         {  
    //������������Ƕ�
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
                                  // 60��+���������ֵ���Ŵ�100��       Hall_Three.angleIQ ת�ӵĵ�ǰ��Ƕ�      ������  5�ĽǶ�-��ǰ
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
                                  // ��ת��  ��ǰ-1�ĽǶ�
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
                                  Hall_Three.Move_State=1;            //(����)
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
                                  Hall_Three.Move_State = 1; //(����)
                              }
                            break;

                            case 0x15:           
                              {
                                  Hall_Three.Step_coeff=( Hall_Three.Hall_angle[1]+ Hall_Three.angleIQ-Hall_Three.Hall_angle[3]);
                                  if (Hall_Three.Step_coeff < _IQ(0.0))                                        
                                    Hall_Three.Step_coeff+=_IQ(1.0);
                                  if (Hall_Three.Step_coeff>_IQ(1.0))
                                    Hall_Three.Step_coeff-=_IQ(1.0);
                                  Hall_Three.Move_State=2;          //��ת
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
                                  Hall_Three.Move_State=2;       // (����)
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
              // ʹ��һ�׵�ͨ�˲��������˲����PWM����
                Hall_Three.Speed_countFitter = _IQ10mpy(HallK2, Hall_Three.Speed_countFitter) + _IQ10mpy(HallK1, Hall_Three.Speed_count);
              // �洢�˲����pmw����Ϊ��ֵ
                Hall_Three.Speed_count_old = Hall_Three.Speed_countFitter;
              // �������Ĳ������ö�����������״̬��Ƕȱ仯����֮��ʱ�䣬����ÿһ�� PWM �ĽǶȱ仯��   w=60��/��NT��
                Hall_Three.step_angle = Hall_Three.Step_coeff / Hall_Three.Speed_count_old;              
              // ʹ��һ�׵�ͨ�˲����˲�������
                Hall_Three.step_angleFitter = _IQ10mpy(HallK2, Hall_Three.step_angleFitter) + _IQ10mpy(HallK1, Hall_Three.step_angle);
              // �����ٶȼ�����Ϊ0
                Hall_Three.Speed_count = 0;
                           
               }//����״̬�仯
               
            else  if ( Hall_Three.Hall_State==Hall_Three.OldHall_State )               
              {
                Hall_Three.Speed_count++;
                if( Hall_Three.Speed_count>=2000 )  // ��ʱ��ֵ
                    {
                      Hall_Three.Move_State=1;                                         //��ת
                      Hall_Three.Speed_count=0;                                        //�����ٶȼ�����
                      Hall_Three.Speed_RPM= 0 ;                                        //��ת������Ϊ0����Ϊû�м�⵽�˶���
                      Hall_Three.step_angle=0 ;                                        //�Ƕȱ仯�������˲�ֵ�����ã�ȱ���µĽǶȱ仯���ݡ�
                      Hall_Three.step_angleFitter=0;
                      switch (Hall_Three.Hall_State )
                            {
                                case 0x6:
                                {
                                Hall_Three.angleIQ = Hall_Three.Hall_angle[0]+_IQ(0.083333333);                 //_IQ(0.083333333)�ṩһ��С�ġ��̶���ƫ����   0.083333333=1/12   Ϊ30��        
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
                     }	//����������
              }   //����״̬δ�仯 

        if ( Hall_Three.Move_State==1 )              //��ת
        {
          Hall_Three.angleIQ = Hall_Three.angleIQ + Hall_Three.step_angleFitter ;
        }
        else if ( Hall_Three.Move_State==2 )       //��ת
        {
          Hall_Three.angleIQ = Hall_Three.angleIQ - Hall_Three.step_angleFitter;
        }

      if( Hall_Three.angleIQ>_IQ(1.0))
        Hall_Three.angleIQ-=_IQ(1.0);
      else if( Hall_Three.angleIQ<_IQ(0.0))
        Hall_Three.angleIQ+=_IQ(1.0);

  // d�����ż�λ�ú͵��λ��,A�������������A�෴�綯���½�0���ʱ��ǶȲ�
      Hall_Three.ele_angleIQ = Hall_Three.angleIQ - Hall_Three.initial_angle ;        
        if( Hall_Three.ele_angleIQ>_IQ(1.0))
          Hall_Three.ele_angleIQ-=_IQ(1.0);
        else if( Hall_Three.ele_angleIQ<_IQ(0.0))
          Hall_Three.ele_angleIQ+=_IQ(1.0);
      Hall_Three.OldHall_State=Hall_Three.Hall_State ;
              
      Sine = _IQsinPU(Hall_Three.ele_angleIQ);
      Cosine = _IQcosPU(Hall_Three.ele_angleIQ);

//======================================================================================================
//�����ٶȼ���
//======================================================================================================
    if( Hall_Three.Move_State==1)    // �жϼ�����ת��2ms�ĽǶȱ仯��
    {
        Hall_Three.Speed_ele_angleIQ =Hall_Three.ele_angleIQ -Hall_Three.old_ele_angleIQ ;       //�����2msʱ�����Ƕȱ仯���� 
        if( Hall_Three.Speed_ele_angleIQ <_IQ(0.0) )
          Hall_Three.Speed_ele_angleIQ+= _IQ(1.0);                                                 // ��ֹ����16777216.0��С��0��   16777216=2^24 
    }
    else  if( Hall_Three.Move_State==2) //  �жϼ��㷴ת��2ms�ĽǶȱ仯����
    {
        Hall_Three.Speed_ele_angleIQ =Hall_Three.old_ele_angleIQ -Hall_Three.ele_angleIQ;
        if( Hall_Three.Speed_ele_angleIQ <_IQ(0.0) )
          Hall_Three.Speed_ele_angleIQ+= _IQ(1.0);
    }
    Hall_Three.Speed_ele_angleIQFitter= _IQ10mpy(HallK1, Hall_Three.Speed_ele_angleIQFitter)+_IQ10mpy(HallK2,  Hall_Three.Speed_ele_angleIQ);  //��ֵһ���˲�   
    Hall_Three.Speed_RPM = _IQmpy(Hall_Three.Speed_ele_angleIQFitter,Hall_Three.speed_coeff); // ��ֵ�仯��*ϵ��=�ٶȡ�   

//=================�ٶȻ�PI===================================
    			Speed_Ref=_IQ(SpeedRef);                                            //ת��Ϊ��������ʽ
    			Speed_Fdb=Hall_Three.Speed_RPM;                                                    //���ٶȸ���Speed_Fdb
    			Speed_Error=Speed_Ref - Speed_Fdb;                                  //�ٶȲο�ֵ�ͷ���ֵ֮��Ĳ�ֵ�����ٶ���
    			Speed_Up=_IQmpy(Speed_Kp,Speed_Error);                              //��������ʹ�ñ�������Speed_Kp�����ٶ����
    			Speed_Ui=Speed_Ui + _IQmpy(Speed_Ki,Speed_Up) + _IQmpy(Speed_Ki,Speed_SatError);         //���������ۼ�֮ǰ�Ļ���ֵ����ǰ���ڵı�����ͱ������ĵ�����
    			Speed_OutPreSat=Speed_Up+Speed_Ui;                                  //����δ���б��ʹ���Ŀ������
    			if(Speed_OutPreSat>Speed_OutMax)                                    //���ʹ���ȷ������������ᳬ���趨��������Сֵ
    				Speed_Out=Speed_OutMax;
    			else if(Speed_OutPreSat<Speed_OutMin)
    	 			Speed_Out=Speed_OutMin;
    			else
    				Speed_Out=Speed_OutPreSat;
    			Speed_SatError=Speed_Out-Speed_OutPreSat;                            //���㱥�������ڵ�����һ���������ڵĻ������ֹ���ֱ��͡�
    			IQ_Given=Speed_Out;
    
    //Clark�任����
    ialfa=ia;                                                                //ֱ��ȡ�����ia��Ϊ���������
    ibeta=_IQmpy(ia,_IQ(0.57735026918963))+_IQmpy(ib,_IQ(1.15470053837926)); //����������������Clark�任��ʽ
    //Park�任����
    id = _IQmpy(ialfa,Cosine) +_IQmpy(ibeta,Sine);
    iq = _IQmpy(ibeta,Cosine)- _IQmpy(ialfa,Sine);

//======================================================================================================
//IQ����PID���ڿ���, �Ա�simulink����
//======================================================================================================  
        IQ_Ref=IQ_Given;                                                          //���ٶȻ�����ĵ���ָ��ֵ����Q������Ĳο�ֵ
        IQ_Fdb=iq;                                                                //��ϵͳ�����л�ȡʵ�ʵ�Q�����ֵ
        IQ_Error=IQ_Ref-IQ_Fdb;                                                   //����Q������Ŀ��������ο�ֵ�ͷ���ֵ֮��Ĳ

        IQ_Up=_IQmpy(IQ_Kp,IQ_Error);                                             //������������ʹ�ñ�������IQ_Kp������
        IQ_Ui=IQ_Ui + _IQmpy(IQ_Ki,IQ_Up) + _IQmpy(IQ_Ki,IQ_SatError);            //���»�������а����������Ļ��ָ��ºͿ���ǰһ���ڵı�����IQ_SatError���������������ֹ���ֱ��͡�

        IQ_OutPreSat=IQ_Up+IQ_Ui;                                                 //����δ���б��ʹ���ǰ��PID�����

        if(IQ_OutPreSat>IQ_OutMax)
          IQ_Out=IQ_OutMax;
        else if(IQ_OutPreSat<IQ_OutMin)
          IQ_Out=IQ_OutMin;
        else 
          IQ_Out=IQ_OutPreSat;  

        IQ_SatError=IQ_Out-IQ_OutPreSat;                                           //�������ڱ������Ƶ��µ���������һ���������ڵĻ��ֵ���
        Uq=IQ_Out;                                                                 //������õ���Q��������������ֵ���������Q���ѹָ��Uq
//======================================================================================================
//ID����PID���ڿ���
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
//IPark�任����Park�任��
//====================================================================================================== 
        Ualfa = _IQmpy(Ud,Cosine) - _IQmpy(Uq,Sine);
        Ubeta = _IQmpy(Uq,Cosine) + _IQmpy(Ud,Sine);        
//======================================================================================================
//SVPWMʵ��
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
    //EVAȫ�Ƚ���������ֵ�������������
    //====================================================================================================== 
      MPeriod = (int16)(T1Period * Modulation);              // Q0 = (Q0 * Q0)

      Tmp = (int32)MPeriod * (int32)MfuncD1;                    // Q15 = Q0*Q15������ȫ�Ƚ���CMPR1��ֵ
      EPwm1Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

      Tmp = (int32)MPeriod * (int32)MfuncD2;                    // Q15 = Q0*Q15������ȫ�Ƚ���CMPR2��ֵ
      EPwm2Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2)

      Tmp = (int32)MPeriod * (int32)MfuncD3;                    // Q15 = Q0*Q15������ȫ�Ƚ���CMPR3��ֵ
      EPwm3Regs.CMPA.half.CMPA = (int16)(Tmp>>16) + (int16)(T1Period>>1); // Q0 = (Q15->Q0)/2 + (Q0/2) 
      
      }   //  �ջ����Ƶ�
    }   //�����������       
               
    if(DC_ON_flag==1)                                              //���Ʊ�־������ָʾϵͳ��Դ�Ƿ�����������״̬��
    {
            if(U_dc_dis<10)//ִ��ͣ������          //���ֱ����ѹ����10����������Ҫִ��ͣ�����       if(U_dc_dis>10)//ȥ���ϵ籣��   Run_PMSM=2��ֵ��2��
            {
            eva_close();
            Run_PMSM=2;
            DC_ON_flag=0;
            }  
    }

    EPwm1Regs.ETCLR.bit.INT=1;//����жϱ�־λ
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
                    
}    //EPWM�жϽ���


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

