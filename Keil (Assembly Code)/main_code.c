//400327772, xingg, Grace Xing
//Bus Speed: 30 MHZ, LED: PF0 (D4)


#include <stdint.h>
#include <math.h>
#include <string.h>
#include "tm4c1294ncpdt.h"
#include "PLL.h"
#include "SysTick.h"
#include "i2c.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "VL53l1X_api.h"



#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif


//******************************************************************************** port intilziation ***********************************************************************************


//Enable D4 ( on board led)
void PortF0_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_DIR_R=0b00000001;
	GPIO_PORTF_DEN_R=0b00000001;
	return;
}


//motor control
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};//allow time for clock to stabilize
	GPIO_PORTH_DIR_R=0b00001111; //Enable PH0-3 as outputs
	GPIO_PORTH_DEN_R=0b00001111; //Enable PH0-3 for digitial I/0
	return;
}

//enable port for external button
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11; //activate the clock for Port M
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R11) == 0){};//allow time for clock to stabilize
	GPIO_PORTM_DIR_R=0b00000000; //Emable PM0 as input (buton) 
	GPIO_PORTM_DEN_R=0b00000001; //Enable PM0 for ditigial I/0
	return;
}



//********************************************************************************************************************************************************

//The VL53L1X needs to be reset using XSHUT.  We will use PG0 (Port intialization)
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
  GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
                                                                                                    // configure PG0 as GPIO
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0

    return;
}

//XSHUT     This pin is an active-low shutdown input
//					the board pulls it up to VDD to enable the sensor by default
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.

//essentaily this function below just puts the sensor in standby
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}


//****************************************************************************** motor control *****************************************************************************************
//I want 32 data measurement in one 360deg rotation
//360/32=11.25 deg, so data is measured every 11.25 deg
//512/32=16 (variable in for loop)

//allows the motor to rotate only 11.25 deg
void spin_cw_increments(){
	for(int i=0; i<16; i++){
		//turn onboard led on
		GPIO_PORTF_DATA_R = 0b00000001;
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(5);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(5);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(5);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait1ms(5);
		//turn onboard led off
		GPIO_PORTF_DATA_R = 0b00000000;
		
	}
}

//spin ccw for 360 deg
void spin_ccw(){
	for(int i=0; i<512; i++){
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait1ms(5);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait1ms(5);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait1ms(5);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait1ms(5);
		
	}
}


//****************************************************************** main ******************************************************************************
//****************************************************************** main ******************************************************************************
//****************************************************************** main ******************************************************************************
//****************************************************************** main ******************************************************************************
//****************************************************************** main ******************************************************************************
//****************************************************************** main ******************************************************************************

//LOOK IN TOF DATASHEET
uint16_t	dev=0x52;

int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint8_t dataReady;
	
	//we dont need the values below, was used in code in studioes for that real term application
	// uint8_t RangeStatus;
	//uint16_t SignalRate;
  //uint16_t AmbientRate;
  //uint16_t SpadNum; 
	

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortM_Init();
	PortF0_Init();


	
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
	//status = VL53L1X_GetSensorId(dev, &wordData);


	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  // This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	status = VL53L1X_SetDistanceMode(dev, 2); 
	
  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
		
		
		


	//*************************************************************************************************************************************************
	//*************************************************************************************************************************************************
	//********************************* here is where we acc get the distance measreuments to 3d map *********************************************
	//*************************************************************************************************************************************************
	//*************************************************************************************************************************************************
	

	int x = 0;
	int y;
	int z;
	double angle;
	int input=0;
	int state=0;
	//represens the amount of vertical slices we want
  int count=0;
	//indicates the state of data transmission
	char TxChar;
	
// always wait for the transmission code from pc. if an integer input is recieved then start the data transimission process
//the integer input represents the amount of vertical slices the user inputed
	while(1){		
		//wait for the right transmition initiation code
			//wait for a charcter comes into the serial port of uc ( from pc to uc); receiving
			input = UART_InChar();
			if (input!=0){
				count=input;
				break;
		}
	}
	//for loop to that increments based on the number of vertical slices desired
	for(int j=0; j<count; j++){																						
		//waiting state; motor off, no measuremnets being done
		state = 0;
		while(state != 1){
			while(1){											// check if the button is pushed
				if ((GPIO_PORTM_DATA_R&0b00000001)==0){
					state=1;
					//if the button is pressed, it sends A to the pc, indiciatng we want to collect data for the next vertical slice
					TxChar = 'A';
					UART_OutChar(TxChar); //sending to pc (uc to pc)
					break;
			 }
		 }																				
		}
	
		for(i = 0; i < 32; i++) {
			//wait until the tof sensor data is ready
			while (dataReady == 0 ){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
			}

			dataReady = 0;
			
			//read the data values from TOF sensor
			status = VL53L1X_GetDistance(dev, &Distance);
			
			angle = i * 11.25;
			//convert to radians, since the math.h takes radians only
			angle=angle* (M_PI/180);
			
			
			y = Distance*(sin(angle));
			z = Distance*(cos(angle));
			
			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
		
			//concatation
			sprintf(printf_buffer,"%d %d %d\r\n",x, y, z);
			//print the result readings to UART
			UART_printf(printf_buffer);
			
			SysTick_Wait10ms(10);
			//rotation 11.25 deg cw
			spin_cw_increments();	
			SysTick_Wait10ms(10);
			
		}
		x += 100;																				// incremenet the x-axis by 10cm 
		spin_ccw();																	// spin 360 degrees ccw;untangle wire
		
	}
	VL53L1X_StopRanging(dev);


  while(1) {}

}


