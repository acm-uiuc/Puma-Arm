/*--------------------------------------------------------------*/
//Revised February 16, 2013
//Description:
//This outputs ePWM1 and ePWM2 on GPIO00/GPIO01 and GPIO03/GPIO04 respectvely. 
//ePWMxA is used to control the motor in one direction, and ePWMxB is used for the opposite direction. 
//The position of the joint is determined by the poisition of potentiometers from ADCA0/ADCA1.
//An encoder is used to determine the position of the joint.
//When an LED is attached to the ePWM outputs, the result is controlling the intensity of the LED.
//This is setup for controlling two joints.
//The code is uploaded to the flash memory, so the GPIO84-87 jumpers need to be set high.
/*--------------------------------------------------------------*/ 

#include "DSP28x_Project.h"
#include "Example_posspeed.h"

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;

interrupt void PWM_isr(void);
interrupt void eQEP_isr(void);
void initialize(void);
void PWMinitialize(void);
void LEDinitialize(void); 
void ADCinitialize(void);
void update_compare(EPWM_INFO *epwm_info);
void potentiometer_compare_speed(void);
void potentiometer_compare_position(void);

// Global variables used in this example
EPWM_INFO epwm1_info;
#define EPWM_CMP_DOWN 0
#define EPWM_CMP_UP 1
// Configure the period for each timer
#define EPWM1A_TIMER_TBPRD  4096  // Period register
#define EPWM1A_MAX_CMPA     4095
#define EPWM1A_MIN_CMPA       5
#define EPWM1B_TIMER_TBPRD  4096  // Period register
#define EPWM1B_MAX_CMPB     4095
#define EPWM1B_MIN_CMPB       5

// ADC start parameters
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#define ADC_CKPS   0x0   // ADC module clock = HSPCLK/1      = 25.5MHz/(1)   = 25.0 MHz
#define ADC_SHCLK  0x1   // S/H width in ADC module periods                  = 2 ADC cycle
//#define AVG        1000  // Average sample limit
//#define ZOFFSET    0x00  // Average Zero offset
//#define BUF_SIZE   1024  // Sample buffer size

#define DEADZONE 50

// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped using
// the linker cmd file.
//#pragma CODE_SECTION(PWM_isr, "ramfuncs");

// These are defined by the linker (see F28335.cmd)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

// Global variables for eQEP
POSSPEED qep_posspeed=POSSPEED_DEFAULTS;
Uint16 Interrupt_Count = 0;
	int ADC_value_0 = 2048;
	int ADC_value_1 = 2048;

void main(void)
{
	InitSysCtrl();	
	
	// Specific clock setting for the ADC
   EALLOW;
   SysCtrlRegs.HISPCP.all = ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
   EDIS;
	
	// For this case just init GPIO pins for ePWM1, ePWM2, eQEP1 and eQEP2
   // These functions are in the DSP2833x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEQep1Gpio();
   InitEQep2Gpio();
	// Clear all interrupts and initialize PIE vector table:
	//Disable the interrupts
	DINT;
	//Initiate PIE control registers to default settings
	InitPieCtrl();
	//Reset CPU interrupt enable and flag
	IER = 0x0000;
	IFR = 0x0000;
	//Initialize PIE vector table
	InitPieVectTable();
	//Set pointers to ISR function
	EALLOW;
	PieVectTable.EPWM1_INT=&PWM_isr;
	PieVectTable.EQEP1_INT=&eQEP_isr;
	EDIS;
	
	// only initialize the ePWM
	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
//------------------------------------------------------------------------------------------
//	ePWM
    PWMinitialize();
   
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
//------------------------------------------------------------------------------------------
//	ADC
    ADCinitialize();
    
    // Step 5. User specific code, enable interrupts:
// Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;
   IER |= M_INT5;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER5.bit.INTx1 = 1;

   //Sets register values for output pins
   LEDinitialize();

   // Copy time critical code and Flash setup code to RAM
// This includes the following ISR functions: epwm1_timer_isr(), epwm2_timer_isr()
// epwm3_timer_isr and and InitFlash();
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the F28335.cmd file.
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
   InitFlash();
  
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

	qep_posspeed.init(&qep_posspeed);

// Start SEQ1
   AdcRegs.ADCTRL2.all = 0x2000;
//===============================================================================
//Infinite for loop that sets the PWM outputs by comparing the encoder counts and 
//potentiometer position	
	for(;;){
			if(EQep1Regs.QPOSCNT > (EQep1Regs.QPOSCMP+DEADZONE)){
				EPwm1Regs.CMPA.half.CMPA = 2048;
				EPwm1Regs.CMPB = 0;
			}
			else if(EQep1Regs.QPOSCNT < (EQep1Regs.QPOSCMP-DEADZONE)){
				EPwm1Regs.CMPA.half.CMPA = 0;
				EPwm1Regs.CMPB = 2048;
			}
		else{
				EPwm1Regs.CMPA.half.CMPA = 0;
				EPwm1Regs.CMPB = 0;
			}
   }
}
void ADCinitialize(void)
{
	// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   InitAdc();         // For this example, init the ADC

// Specific ADC setup for this example:
   AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;  // Sequential mode: Sample rate   = 1/[(2+ACQ_PS)*ADC clock in ns]
                        //                     = 1/(3*40ns) =8.3MHz (for 150 MHz SYSCLKOUT)
					    //                     = 1/(3*80ns) =4.17MHz (for 100 MHz SYSCLKOUT)
					    // If Simultaneous mode enabled: Sample rate = 1/[(3+ACQ_PS)*ADC clock in ns]
   AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // 1  Cascaded mode
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
   AdcRegs.ADCTRL1.bit.CONT_RUN = 1;       // Setup continuous run

   AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;       // Enable Sequencer override feature
   AdcRegs.ADCCHSELSEQ1.all = 0x0;
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0; 
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 = 0x1;       
   AdcRegs.ADCCHSELSEQ2.all = 0x0;
   AdcRegs.ADCCHSELSEQ3.all = 0x0;
   AdcRegs.ADCCHSELSEQ4.all = 0x0;
   AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0x7;  // convert and store in 8 results registers
    
}

//Not used in this version
interrupt void PWM_isr(void){
	// Update the CMPA values
//    update_compare(&epwm1_info);
//	GpioDataRegs.GPATOGGLE.bit.GPIO30 = 1;
//	DELAY_US(50000);
	
	potentiometer_compare_position();
	
	// Position and Speed measurement
   qep_posspeed.calc(&qep_posspeed);

	// Clear INT flag for the timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    
}

//ISR when QPOSCNT=QPOSCMP
interrupt void eQEP_isr(){
	//Clear eQEP gloabal interrupt and position-compare module interrupts
	EQep1Regs.QCLR.bit.INT = 1;
	EQep1Regs.QCLR.bit.PCM = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
	//Turn GPIO30 on then off when the encoder count equals the potentiometer value
	GpioDataRegs.GPASET.bit.GPIO30 = 1;
	DELAY_US(50000);
	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
	// Position and Speed measurement
   qep_posspeed.calc(&qep_posspeed);
}

void LEDinitialize(void){
	// Configure GPIO30 as a GPIO output pin
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
   EDIS;
   GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
   
//   /* Enable internal pull-up for the selected pins */
//// Pull-ups can be enabled or disabled by the user. 
//// This will enable the pullups for the specified pins.
//// Comment out other unwanted lines.
//
//    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
//    
//    /* Configure ePWM-1 pins using GPIO regs*/
//// This specifies which of the possible GPIO pins will be ePWM1 functional pins.
//// Comment out other unwanted lines.
//
//    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
}

void PWMinitialize(){
//==========================================================================
//Set up ePWM1	
	// Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = EPWM1A_TIMER_TBPRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadow register load on ZERO
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = EPWM1A_MIN_CMPA;    // Set compare A value
   EPwm1Regs.CMPB = EPWM1B_MIN_CMPB;    // Set compare B value
  
   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A high on Zero
   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
   EPwm1Regs.AQCTLA.bit.PRD = 1	;			     // Set PWM1A high when CTR=PRD
   EPwm1Regs.AQCTLB.bit.PRD = 1	;

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
   
//=============================================================================
//Set up ePWM2
// Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm2Regs.TBPRD = EPWM1A_TIMER_TBPRD;       // Set timer period
   EPwm2Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadow register load on ZERO
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = EPWM1A_MIN_CMPA;    // Set compare A value
   EPwm2Regs.CMPB = EPWM1B_MIN_CMPB;    // Set compare B value
  
   // Set actions
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A high on Zero
   EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count
   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
   EPwm2Regs.AQCTLA.bit.PRD = 1	;			     // Set PWM1A high when CTR=PRD
   EPwm2Regs.AQCTLB.bit.PRD = 1	;
   
//   // Information this example uses to keep track
//   // of the direction the CMPA/CMPB values are
//   // moving, the min and max allowed values and
//   // a pointer to the correct ePWM registers
//   epwm1_info.EPwm_CMPA_Direction = 1; // Start by increasing CMPA & CMPB
//   epwm1_info.EPwmTimerIntCount = 0;             // Zero the interrupt counter
//   epwm1_info.EPwmRegHandle = &EPwm1Regs;        // Set the pointer to the ePWM module
//   epwm1_info.EPwmMaxCMPA = EPWM1A_MAX_CMPA;      // Setup min/max CMPA/CMPB values
//   epwm1_info.EPwmMinCMPA = EPWM1A_MIN_CMPA;
	
}

//Not used in this versions
void update_compare(EPWM_INFO *epwm_info)
{
   // Every 10'th interrupt, change the CMPA/CMPB values
   if(epwm_info->EPwmTimerIntCount == 1000)
   {
       epwm_info->EPwmTimerIntCount = 0;
       GpioDataRegs.GPATOGGLE.bit.GPIO30 = 1;

       // If we were increasing CMPA, check to see if
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
	   if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }

	   // If we were decreasing CMPA, check to see if
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
	   else
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }
   }
   else
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}

void potentiometer_compare_speed(){
	//Set CMPA and CMPB values for ePWM1 signals
   	  ADC_value_0 = ((AdcRegs.ADCRESULT0)>>4);
   	  ADC_value_1 = ((AdcRegs.ADCRESULT1)>>4);
   	  
   	  //Sets both ePWM signals to zero for deadzone
   	  if ((ADC_value_0 > (2048-DEADZONE)) && (ADC_value_0 < (2048+DEADZONE)))
   	  {
   	  	EPwm1Regs.CMPA.half.CMPA = 0;		//Set ePWM1A to zero
   	  	EPwm1Regs.CMPB = 0;					//Set EPWM1B to zero
//   	  	GpioDataRegs.GPASET.bit.GPIO30 = 1;	//Set GPIO30 pin high to indicate being in the deadzone
   	  }
   	  
   	  //Sets ePWM1A to control motor when ADC is above the deadzone
   	  else if (ADC_value_0 >= (2048+DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm1Regs.CMPB = 0;						//Set ePWM1B to zero
   	  	EPwm1Regs.CMPA.half.CMPA = ((ADC_value_0-((float)2048+DEADZONE))/((float)4096-(2048+DEADZONE))*(float)4096);	//Set ePWM1A value when ADC is above deadzone
   	  }
   	  
   	  //Sets ePWM1B to control motor when ADC is below the deadzone
   	  else if (ADC_value_0 <= (2048-DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm1Regs.CMPA.half.CMPA = 0;			//Set ePWM1A to zero
   	  	EPwm1Regs.CMPB = ((((float)2048-DEADZONE)-ADC_value_0)/((float)2048+DEADZONE))*(float)4096;	//Set ePWM1B value when ADC is below the deadzones
   	  }
//================================================================================
if ((ADC_value_1 > (2048-DEADZONE)) && (ADC_value_1 < (2048+DEADZONE)))
   	  {
   	  	EPwm2Regs.CMPA.half.CMPA = 0;		//Set ePWM1A to zero
   	  	EPwm2Regs.CMPB = 0;					//Set EPWM1B to zero
//   	  	GpioDataRegs.GPASET.bit.GPIO30 = 1;	//Set GPIO30 pin high to indicate being in the deadzone
   	  }
   	  //Sets ePWM1A to control motor when ADC is above the deadzone
   	  if (ADC_value_1 >= (2048+DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm2Regs.CMPB = 0;						//Set ePWM1B to zero
   	  	EPwm2Regs.CMPA.half.CMPA = ((ADC_value_1-((float)2048+DEADZONE))/((float)4096-(2048+DEADZONE))*(float)4096);	//Set ePWM1A value when ADC is above deadzone
   	  }
   	  //Sets ePWM1B to control motor when ADC is below the deadzone
   	  if (ADC_value_1 <= (2048-DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm2Regs.CMPA.half.CMPA = 0;			//Set ePWM1A to zero
   	  	EPwm2Regs.CMPB = ((((float)2048-DEADZONE)-ADC_value_1)/((float)2048+DEADZONE))*(float)4096;	//Set ePWM1B value when ADC is below the deadzones
   	  }
   	  DELAY_US(50);
}

void potentiometer_compare_position(){
	//Set CMPA and CMPB values for ePWM1 signals
   	  ADC_value_0 = ((AdcRegs.ADCRESULT0)>>4);
   	  ADC_value_1 = ((AdcRegs.ADCRESULT1)>>4);
   	//Set the encoder compare value based on potentiometer input  
   	 EQep1Regs.QPOSCMP = (ADC_value_0/(float)4095)*(float)750;
   	 
   	 DELAY_US(5);
}
