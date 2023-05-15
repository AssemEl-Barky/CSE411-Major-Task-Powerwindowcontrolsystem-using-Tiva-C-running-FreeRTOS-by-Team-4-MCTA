# CSE411 Major Task: Power Window Control System using Tiva-C running FreeRTOS
## Project by Team 4 Mechatronics and Automation
This is a repository for the major task of the **CSE411: Real-Time and Embedded Systems Design**  course taught during the Spring semester of 2023 at the faculty of engineering, Ain Shams University.

**Professor**: Dr. Sherif Hammad

**Teaching Assistants**: Eng. Hesham Salah and Eng. Mohamed Tarek

**Team 4 members**

| Name | ID |
| --- | --- |
| Ahmed Khaled Mohamed Fathy | 19P1787 |
| Amr Issa El-Sayed Mohamed Tawfik | 18P8601 |
| Assem Khaled Farouk Morsi El-Barky | 18P4802 |
| Marwan Ayman Mahmoud Elatfehy | 19P2588 |
| John Ashraf Adeeb | 18P5411 |
| Marina Mourad KaramAllah | 18P7884 |

## Project Description
The project target is to program a control system for a power window using TivaC running FreeRTOS.

### Project scope
1. Implementation of front passenger door window with both passenger and driver control
panels.
2. FreeRTOS implementation is a must.
3. Implementation of 2 limit switches to limit the window motor from top and bottom limits of
the window.
4. Obstacle detection implementation is required, with a push button used to indicate jamming instead of a sensor.

The desired area to be controlled is shown in the following figure:
![image](https://github.com/AssemEl-Barky/CSE411-Major-Task-Powerwindowcontrolsystem-using-Tiva-C-running-FreeRTOS-by-Team-4-MCTA/assets/63543410/9b763744-29bb-4c17-afda-add8a0d02ada)

### System basic features
1. Manual open/close function
  
     When the power window switch is pushed or pulled
continuously, the window opens or closes until the switch
is released.

2. One touch auto open/close function

     When the power window switch is pushed or pulled
shortly, the window fully opens or closes.

3. Window lock function

      When the window lock switch is turned on, the opening and closing of
all windows except the driver’s window is disabled.

4. Jam protection function

     This function automatically stops the power window and moves it
downward about 0.5 second if foreign matter gets caught in the
window during one touch auto close operation.

## Project Implementation
### Circuit Schematic
The follwing figure shows the circuit schematic
/*INSERT CIRCUIT SCHEMATIC*/
### Hardware
The following figure shows a hardware implementation of the circuit. 
The buttons from top to botton are:
1. Manual driver up
2. Manual driver down
3. Manual passenger up
4. Manual passenger down
5. Automatic driver up
6. Automatic driver down
7. Automatic passenger up
8. Automatic passenger down
9. button to simulate the jamming sensor
### Finite state machine
The system can be considered as a finite state machine which can be seen in the follwing diagram.
/*INSERT CIRCUIT SCHEMATIC*/
### Demonstration video
The project [demostration video](https://drive.google.com/drive/folders/1Wlv8UiPhQYOf2a-1ebrvUrTnOT7fQLjv?usp=sharing) 
