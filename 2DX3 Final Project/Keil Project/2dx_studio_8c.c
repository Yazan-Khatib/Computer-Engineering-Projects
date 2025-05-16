/*  Time of Flight for 2DX4 -- Studio W8-0
                Code written to support data collection from VL53L1X using the Ultra Light Driver.
                I2C methods written based upon MSP432E4 Reference Manual Chapter 19.
                Specific implementation was based upon format specified in VL53L1X.pdf pg19-21
                Code organized according to en.STSW-IMG009\Example\Src\main.c
                
                The VL53L1X is run with default firmware settings.
 
 
            Written by Tom Doyle
            Updated by  Hafez Mousavi Garmaroudi
            Last Update: March 17, 2020
            
            Last Update: March 03, 2022
            Updated by Hafez Mousavi
            __ the dev address can now be written in its original format.
                Note: the functions  beginTxI2C and  beginRxI2C are modified in vl53l1_platform_2dx4.c file
                
            Modified March 16, 2023
            by T. Doyle
              - minor modifications made to make compatible with new Keil IDE
 
*/
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"
 
 
 
 
 
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
 
#define MAXRETRIES              5           // number of receive attempts before giving up

 

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                                     // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                                  // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                                    // ready?
 
    GPIO_PORTB_AFSEL_R |= 0x0C;                                             // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;                                               // 4) enable open drain on PB3 only
 
    GPIO_PORTB_DEN_R |= 0x0C;                                               // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;                                          // 7) disable analog functionality on PB2,3
 
                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                                                // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                         // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                                    // 8) configure for 100 kbps clock
        
}
 
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
 
//XSHUT     This pin is an active-low shutdown input;
//          the board pulls it up to VDD to enable the sensor by default.
//          Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
    
}
 
 
//*********************************************************************************************************
//*********************************************************************************************************
//***********          MAIN Function        *****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t  dev = 0x29;      //address of the ToF sensor as an I2C slave peripheral
int status=0;
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;				// activate clock for Port H
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};	// allow time for clock to stabilize
	GPIO_PORTH_DIR_R |= 0x0F;        								// configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;     								// disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;        								// enable digital I/O on Port H pins (PH0-PH3)
																									// configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;     								// disable analog functionality on Port H	pins (PH0-PH3)	
	return;
}

 
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};      // Allow time for clock to stabilize
		
	GPIO_PORTM_DIR_R |= 0x01;       								      		// Enable PM0 as output
  GPIO_PORTM_DEN_R |= 0x01;																	// Enable PM0 as digital pin
	return;
}
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ0 and PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ0 and PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//? Configure PJ0 and PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											//??Disable analog functionality on PJ0 and PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;													//	Enable weak pull up resistor on PJ0 and PJ1
}

  
void spin(){ 	// Complete function spin to implement the Full-step Stepping Method
	
  	GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1); //  used to introduce a delay between each step to control the speed of the motor.	
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110; // represent the signals that are sent to the stepper motor
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
 

}
 



void unspin(){  	// Complete function spin to implement the Full-step Stepping Method
	
for (int i = 0; i < 512; i++){ // does a full 360 turn (512 steps for mottor)
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(1);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(1);
		
	}
}


int  position  = 0;
int flag = 0;  
void takeMeasurement(uint16_t Distance, uint8_t RangeStatus){
	status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
				status = VL53L1X_GetDistance(dev, &Distance);					 
				status = VL53L1X_ClearInterrupt(dev);  
	
				// print the resulted readings to UART
				sprintf(printf_buffer,"%u, %u\r\n", Distance, RangeStatus);
        UART_printf(printf_buffer);
				FlashLED3(1);
				SysTick_Wait10ms(50);
	
}
int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum;
  uint8_t RangeStatus;
  uint8_t dataReady;
 
  //initialize
  PLL_Init();  
  SysTick_Init();
  onboardLEDs_Init();
  I2C_Init();
  UART_Init();
  PortH_Init();
  PortJ_Init();
	PortM_Init();
/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1X_GetSensorId(dev, &wordData);
 
 
  // 1 Wait for device ToF booted
  while(sensorState==0){
    status = VL53L1X_BootState(dev, &sensorState);
    SysTick_Wait10ms(10);
  }
  FlashAllLEDs();
  
  
  status = VL53L1X_ClearInterrupt(dev);  
  
 
  status = VL53L1X_SensorInit(dev);
  Status_Check("SensorInit", status);
 
 
  status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
 
	int delay = 1;  
	int position  = 0;
	int step = 16;  
	
	while(1) {
		
		// checks for button press at PJ1
		if (GPIO_PORTJ_DATA_R == 0b10) {
			flag = 1;
		}
		
		// checks for button press at PJ0
		else if (GPIO_PORTJ_DATA_R == 0b01) {
			UART_printf("Measurements done\n"); // sent to PC so it knows scans are done and to visualize now
			break; // breaks while loop since measurements are done and ends program
		}
	
	  
			if (flag == 1){ 
					
				spin(); // calls the function 
				FlashLED1(1);
				//ToF transmits data
				if (position %step == 0 && position !=0){ //checks whether the current position ue of "position " is divisible by "step". 
							while (dataReady == 0){
								status = VL53L1X_CheckForDataReady(dev, &dataReady);
								VL53L1_WaitMs(dev, 5);
				}
					dataReady = 0;  
					takeMeasurement(Distance, RangeStatus);
				}
				
				if (position  == 512){  
					flag = 0;  
					position  = 0;   
					GPIO_PORTN_DATA_R ^= 0b00000001;
					unspin(); 
					GPIO_PORTN_DATA_R ^= 0b00000001;
					
				}
				position ++; 
			}
			
	// SysTick_Wait1ms(10);
	// GPIO_PORTM_DATA_R ^= 0x01; 
	
	}
	
 
}
		
  

  
