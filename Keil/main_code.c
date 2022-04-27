//400327772, xingg, Grace Xing
//Bus Speed: 30 MHZ, LED: PF0


#include <stdint.h>
#include <math.h>
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

void I2C_Init(void);
void UART_Init(void);
void PortG_Init(void);
void VL53L1X_XSHUT(void);

//******************************************************************************** port intilziation ***********************************************************************************

//connects the external button to the microcontroller
void PortK0_Init(void){
        SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R9;            // activate clock for Port K
        while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R9) == 0){};   // allow time for clock to stabilize
        GPIO_PORTK_DIR_R=0b00000000;                        // Make Pin 0 as input for the button (reading if the buton is pressed or not)
        GPIO_PORTK_DEN_R=0b00000001;                        // Enable Pin 0 
        return;
}


//port for motors
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};//allow time for clock to stabilize
	GPIO_PORTH_DIR_R=0b00001111; //Make Pin0-3 as ouput
	GPIO_PORTH_DEN_R=0b00001111; //Enable Pin0-3
	return;
}



//Enable D4 ( on board led)
void PortF0_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 //activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};
	GPIO_PORTF_DIR_R=0b00000001;
	GPIO_PORTF_DEN_R=0b00000001;
	return;
}


//****************************************************************************** motor control *****************************************************************************************
void spin_ccw(){
	for(int i=0; i<512; i++){
		if (i%64==0 & i!=0){
			GPIO_PORTF_DATA_R = 0b00000001;
		}
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
		
		GPIO_PORTF_DATA_R = 0b00000000;
	}
}

//I want 32 data measurement points in on rotation
//360/32=11.25 deg and 512/32=16 (variable in for loop)

//this fxn will cause the motor to rotate only 11.26 deg
void spin_cw_increments(){
	for(int i=0; i<16; i++){
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		
	}
}


/*
void button(void){
	while(1){//keep checking if the button is pressed 
	
		//checks if the data outputted to the button is 0 in port k0
		//button is an active low
		if((GPIO_PORTK_DATA_R&0b00000001)==0){
				spin(-1,1); //spinw cw
				GPIO_PORTM_DATA_R = 0b00000000; //resets/turns off the motor
			  return;
		}	
	}
}	
*/


//********************************************************** no idea what this is for, know it is realted to i2c ***************************888-
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
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

//XSHUT     This pin is an active-low shutdown input; the board pulls it up to VDD to enable the sensor by default. Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
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

int main(void){
	
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
	uint8_t RangeStatus;
  uint8_t dataReady;
	
	//we dont need the values below, was used in code in studioes for that real term application
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
	PortK0_Init();

	
	// hello world!
	UART_printf("Program Begins\r\n");
	int mynumber = 1;
	sprintf(printf_buffer,"Final Project %d\r\n",mynumber);
	UART_printf(printf_buffer);
	
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	
  // This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
  status = VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
	

	//*************************************************************************************************************************************************
	//*************************************************************************************************************************************************
	//********************************* here is where we acc get the distance measreuments to 3d map *********************************************
	//*************************************************************************************************************************************************
	//*************************************************************************************************************************************************
	

	int16_t x = 0;
	int16_t y;
	int16_t z;
	double angle;


	//we want to collect measuremnets 32 times in one 360 deg rotation
	for( i = 0; i < 32; i++) {
			//wait until the tof sensor data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
			}

			dataReady = 0;
			
			//read the data values from TOF sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);
			
			//calcultes the y and z corr from the distance measurement
			
			angle = i * 11.25;
			//convert to radians, since the math.h takes radians only
			angle=angle* (M_PI/180);
			y = Distance*(sin(angle));
			z = Distance*(cos(angle));
			
			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			
			//print the result readings to UART
			sprintf(printf_buffer,"%d %d %d %d\r\n",i,x, y, z);
			UART_printf(printf_buffer);
			
			SysTick_Wait10ms(10);
			
			//rotation 11.25 deg cw
			spin_cw_increments();													
		}

	
	VL53L1X_StopRanging(dev);

  while(1) {}
		
}
