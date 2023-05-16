/*
Major task for the CSE 411 Real-time and Embedded Systems Design course, taken during the Spring semester of 2023 at the Faculty of Engineering, Ain Shams University.
Professor: Dr.Sherif Hammad
Teaching Assistants: Eng.Hesham Salah and Eng.Mohamed Tarek

Ptoject by team 4 , Mechatronics and Automation

The project is the code for a passenger's power window control system in an automotive vehicle. The code is implemented using the FreeRTOS kernel and the TIVAWARE drivers library.
*/

/*
Hardware Connections:
		//Inputs 
PA2 - Manual Driver  Up
PA3 - Manual Driver  Down 
PA6 - Manual Passenger Up 
PA7 - Manual Passenger Down

PB2 - Auomatic Driver Up
PB3 - Automatic Driver Down
PB6 - Automatic Passenger Up
PB7 - Automatic Passenger Down

PA4 - Jamming
PA5 - Lock

PC4 - Window down Limit switch 
PC5 - Window up Limit switch

	//Outputs
PE1 - MOTOR A1
PE2 - MOTOR A2
*/

    /***Includes***/
#define PART_TM4C123GH6PM		
#include "TM4C123GH6PM.h"
#include <FreeRTOS.h>
#include "task.h"
#include <semphr.h>
#include <driverlib/gpio.c>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"


			/*Function declarations*/
void init(); // Initializations
void GPIOA_Handler(); // GPIOA ISR
void GPIOB_Handler(); // GPIOB ISR

		/*Delay function in milliseconds*/
void delayMs(uint32_t n)
{
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++)
    {}
  }
}

				/*Task declarations */

void driver_manual_up(void *params);
void driver_manual_down(void *params);
void driver_automatic_up (void *params);
void driver_automatic_down (void *params);
void passenger_manual_up (void *params);
void passenger_manual_down (void *params);
void passenger_automatic_up (void *params);
void passenger_automatic_down (void *params);
void jamming (void* params);
void lock (void* params);


					/*Semaphores declarations*/

SemaphoreHandle_t S_driver_manual_up;
SemaphoreHandle_t S_driver_manual_down;
SemaphoreHandle_t S_driver_automatic_up;
SemaphoreHandle_t S_driver_automatic_down;
SemaphoreHandle_t S_passenger_manual_up;
SemaphoreHandle_t S_passenger_manual_down;
SemaphoreHandle_t S_passenger_automatic_up;
SemaphoreHandle_t S_passenger_automatic_down;
SemaphoreHandle_t S_jamming;
SemaphoreHandle_t S_lock;
							
							/*Mutex Declarations*/
							
SemaphoreHandle_t MotorMutex;
							
							/*Queue Declarations*/
							
xQueueHandle xAutoUpQueue;

int main()
{
							/***Initializations***/
	
	init();
	
					/***Create the semaphhores***/
	
	S_driver_manual_up = xSemaphoreCreateBinary();
	S_driver_manual_down = xSemaphoreCreateBinary();
	S_driver_automatic_up = xSemaphoreCreateBinary();
	S_driver_automatic_down = xSemaphoreCreateBinary();
	S_passenger_manual_up = xSemaphoreCreateBinary();
	S_passenger_manual_down = xSemaphoreCreateBinary();
	S_passenger_automatic_up = xSemaphoreCreateBinary();
	S_passenger_automatic_down = xSemaphoreCreateBinary();
	S_jamming = xSemaphoreCreateBinary();
	S_lock = xSemaphoreCreateBinary();

					/*****Create Mutex****/
	
	MotorMutex = xSemaphoreCreateMutex();
	
					/***Create the tasks***/

  xTaskCreate(driver_manual_up,"driver_manual_up",80,NULL,2,NULL);
  xTaskCreate(driver_manual_down,"driver_manual_down",80,NULL,2,NULL);
	xTaskCreate(driver_automatic_up,"driver_automatic_up",80,NULL,2,NULL);
  xTaskCreate(driver_automatic_down,"driver_automatic_down",80,NULL,2,NULL);
  xTaskCreate(passenger_manual_up,"passenger_manual_up",80,NULL,1,NULL);
  xTaskCreate(passenger_manual_down,"passenger_manual_down",80,NULL,1,NULL);
  xTaskCreate(passenger_automatic_up,"passenger_automatic_up",80,NULL,1,NULL);
  xTaskCreate(passenger_automatic_down,"passenger_automatic_down",80,NULL,1,NULL);
  xTaskCreate(jamming,"jamming",80,NULL,2,NULL);
  xTaskCreate(lock,"lock_passenger",80,NULL,2,NULL);
	
						/***Create the Queues***/
						
	xAutoUpQueue = xQueueCreate( 1, sizeof(int));

						/***Start the scheduler***/
	vTaskStartScheduler();
	
for(;;)
	{
	// This loop should never be reached	
	}
}

									/**Functions**/

void init() // Initializations of GPIOs
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
			;
		// Configure pins of PORTA
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4|GPIO_PIN_5);
		GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4  , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4| GPIO_PIN_5, GPIO_FALLING_EDGE); //All buttons, limit switches, and lock switch are set to work on falling edge detection
		GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_Handler);
		IntPrioritySet(INT_GPIOA, 0XE0);
		
		// Configure pins of PORTB
		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3);
		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3);
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);
		GPIOIntRegister(GPIO_PORTB_BASE, GPIOB_Handler);
		IntPrioritySet(INT_GPIOB, 0XE0);
		
		//Configure pins of PORTC
		GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
		GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		
		//Configure pins of PORTE
		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); 		// PORTE output pins for the motor
		GPIOUnlockPin(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
			
}

				/**Interrupt Service Routines**/

void GPIOA_Handler() //ISR of PORT A, Checks which button has been pressed
	{
	BaseType_t xHigherPriorityTaskWoken= pdFALSE;    //Highest priority flag is initialized to false , the flag is used for context switching if a higher or equal priority task is preempted from the ISR
	/*******Manual Driver Up******/
	if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2) != GPIO_PIN_2)     
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_2); // Clear Interrupt
		xSemaphoreGiveFromISR(S_driver_manual_up,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	/********Manual Driver Down******/
		else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3) != GPIO_PIN_3) 
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3); // Clear Interrupt
		xSemaphoreGiveFromISR(S_driver_manual_down,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	
	/********Manual Passenger Up******/
	else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) != GPIO_PIN_6)
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_6); // Clear Interrupt
		xSemaphoreGiveFromISR(S_passenger_manual_up,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}

	/********Manual Passenger Down******/
		else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_7) != GPIO_PIN_7) 
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_7); // Clear Interrupt
		xSemaphoreGiveFromISR(S_passenger_manual_down,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	
	/******* Lock ************/
	else if ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))!= GPIO_PIN_5)  // check if the lock switch is pressed
{
    delayMs(10); //Debouncing
		if ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))!= GPIO_PIN_5){	
		//int flag = 0; // For debugging
		//flag = !GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5); // For debugging
		GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);  // Clear Interrupt
    xSemaphoreGiveFromISR(S_lock, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);  //context switching if flag == 1
		}
}

	/**********Jamming********/
	
else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_4) != GPIO_PIN_4) // check anhy button kan metdas (fel tivaware) 
  {
 		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_4); // Clear Interrupt
 		xSemaphoreGiveFromISR(S_jamming,&xHigherPriorityTaskWoken);
 		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
 }
	
	else 
		{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_2);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_6);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_7);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_5);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_4);	
	  }
}
	
void GPIOB_Handler() //ISR of PORT B, Checks which button has been pressed
	{
	BaseType_t xHigherPriorityTaskWoken= pdFALSE;    //Highest priority flag is initialized to false , the flag is used for context switching if a higher or equal priority task is preempted from the ISR
	/********Automatic Driver Up******/
	if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_2) != GPIO_PIN_2)     
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2); // Clear Interrupt
		xSemaphoreGiveFromISR(S_driver_automatic_up,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1	
	}

	/********Automatic Driver Down******/
		else if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_3) != GPIO_PIN_3) 
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3); // Clear Interrupt
		xSemaphoreGiveFromISR(S_driver_automatic_down,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	
	/********Automatic Passenger Up******/
	else if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_6) != GPIO_PIN_6)
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_6); // Clear Interrupt
		xSemaphoreGiveFromISR(S_passenger_automatic_up,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}

	/********Automatic Passenger D******/
		else if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_7) != GPIO_PIN_7) 
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_7); // Clear Interrupt
		xSemaphoreGiveFromISR(S_passenger_automatic_down,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	else 
		{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2);
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3);
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_6);
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_7);
	  }
}
	
					/**Tasks**/

void driver_manual_up(void *params)
	{
	xSemaphoreTake(S_driver_manual_up,0); // An initial check to ensure that the semaphore is empty (for safety)
	for(;;)
	{
		xSemaphoreTake (S_driver_manual_up, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2) != GPIO_PIN_2 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5) //As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on 
		{
			// empty
		}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);  //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		xSemaphoreGive(MotorMutex);
	} 
}

void driver_manual_down(void *params)
	{
	xSemaphoreTake(S_driver_manual_down,0); // An initial check to ensure that the semaphore is empty (for safety)
	for(;;)
	{
		xSemaphoreTake (S_driver_manual_down, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3) != GPIO_PIN_3 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4) //As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on 
		{
		// empty
		}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0); //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0);
		xSemaphoreGive(MotorMutex);
	}
	}

void passenger_manual_up (void *params)
	{
		xSemaphoreTake(S_passenger_manual_up,0); // An initial check to ensure that the semaphore is empty (for safety))
		for(;;)
	{
		xSemaphoreTake (S_passenger_manual_up, portMAX_DELAY); 
		//xSemaphoreTake(MotorMutex, portMAX_DELAY); //Mutex is not given to the passenger's manual task so that the priority resides with the driver
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) != GPIO_PIN_6 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5) //As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on 
		{
			// empty
		}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		//xSemaphoreGive(MotorMutex); 
	}
	}

void passenger_manual_down (void *params)
	{
		
		xSemaphoreTake(S_passenger_manual_down,0); // An initial check to ensure that the semaphore is empty (for safety))
		for(;;)
	{
		xSemaphoreTake (S_passenger_manual_down, portMAX_DELAY);
		//xSemaphoreTake(MotorMutex, portMAX_DELAY);//Mutex is not given to the passenger's manual task so that the priority resides with the driver
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_7) != GPIO_PIN_7 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4) //As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on
		{
		// empty
		}
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);// Turn the motor off
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);		
		//xSemaphoreGive(MotorMutex);
	}
	}

void driver_automatic_up (void *params)
	{
		xSemaphoreTake(S_driver_automatic_up,0); // An initial check to ensure that the semaphore is empty (for safety)
	for(;;)
	{
		xSemaphoreTake (S_driver_automatic_up, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);	
			while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5) //Keep the motor on as long as the corresponding limit switch is not reached
			{
				int auto_pressed = 0;
				if(xQueueReceive(xAutoUpQueue, &auto_pressed,0) == pdTRUE)	//check if jamming occured
				{
					if(auto_pressed == 1){ //check if the window is currently in automatic up mode
					// Stop the motor			
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
					delayMs(300);
					// Bring the window down for a little bit	
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); 
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
					delayMs(500);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
					break;	// Break from the nested if condition to check if the jamming condition is cleared
					}
				 }
			}
			
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor off
	  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		xSemaphoreGive(MotorMutex);
		
}
}
	
void driver_automatic_down (void *params)
	{
		xSemaphoreTake(S_driver_automatic_down,0); // An initial check to ensure that the semaphore is empty (for safety)
	for(;;)
	{
		xSemaphoreTake (S_driver_automatic_down, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
		while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4) //Keep the motor on as long as the corresponding limit switch is not reached
			{
			// empty	
			}
			
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor off
	  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
   	  xSemaphoreGive(MotorMutex);
	}
	}
void passenger_automatic_up (void *params)
	{
		xSemaphoreTake(S_passenger_automatic_up,0); // An initial check to ensure that the semaphore is empty (for safety)
	for(;;)
	{
		xSemaphoreTake (S_passenger_automatic_up, portMAX_DELAY);
		//xSemaphoreTake(MotorMutex, portMAX_DELAY);//Mutex is not given to the passenger's manual task so that the priority resides with the driver
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn on the motor
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
			while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5) //Keep the motor on as long as the corresponding limit switch is not reached
			{
				int auto_pressed = 0;
				if(xQueueReceive(xAutoUpQueue, &auto_pressed,0) == pdTRUE)//Check if jamming occured
				{
					if(auto_pressed == 1){ //check if the window is currently in automatic up mode
					// Stop the motor			
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
					delayMs(300);
					// Bring the window down for a little bit	
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); 
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
					delayMs(500);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
					break;	
					}
			}
			}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); // Turn off the motor
	  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		//xSemaphoreGive(MotorMutex);
	}
	}
void passenger_automatic_down (void *params)
	{
		xSemaphoreTake(S_passenger_automatic_down,0); // An initial check to ensure that the semaphore is empty (for safety)
	for(;;)
	{
		xSemaphoreTake (S_passenger_automatic_down, portMAX_DELAY);
		//xSemaphoreTake(MotorMutex, portMAX_DELAY);//Mutex is not given to the passenger's manual task so that the priority resides with the driver
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn on the motor
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
				while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4) //Keep the motor on as long as the corresponding limit switch is not reached
			{
			// empty	
			}	
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn off the motor
	  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
	  //xSemaphoreGive(MotorMutex);	
	}
	}
void lock(void *params)
{
	xSemaphoreTake(S_lock, 0);
	for(;;)
	{
	xSemaphoreTake(S_lock, portMAX_DELAY);	
	while(!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))) //Check if lock switch is pressed
		{
			GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); //Disable manual control by the passenger
			GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); //Disable automatic control by the passenger

		//__WFI();
		}
			GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); //Enable manual control by the passenger
			GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); //Enable automatic control by the passenger
		}
}

void jamming (void* params)
	{
		 
		xSemaphoreTake(S_jamming,0); 
		for(;;)
		{
		xSemaphoreTake (S_jamming, portMAX_DELAY);
		const TickType_t xDelay = 10 / portTICK_RATE_MS; //standard delay to be used with queues
		int flag = 1;
		xQueueSend(xAutoUpQueue,&flag,xDelay);	//Queue that let's other tasks know that jamming was detected
	}
}
	
