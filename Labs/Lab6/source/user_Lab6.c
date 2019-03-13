/*
 *  ======== main.c ========
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <c6x.h> // register defines

#include <xdc/std.h>
#include <ti/sysbios/family/c64p/Cache.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>


#include "evmomapl138.h"
#include "evmomapl138_i2c.h"
#include "evmomapl138_timer.h"
#include "evmomapl138_led.h"
#include "evmomapl138_dip.h"
#include "evmomapl138_gpio.h"
#include "evmomapl138_vpif.h"
#include "evmomapl138_spi.h"
#include "COECSL_edma3.h"
#include "COECSL_mcbsp.h"
#include "COECSL_registers.h"

#include "mcbsp_com.h"
#include "ColorVision.h"
#include "ColorLCD.h"
#include "sharedmem.h"
#include "LCDprintf.h"
#include "ladar.h"
#include "xy.h"
#include "MatrixMath.h"
#include "projectinclude.h"

#define FEETINONEMETER 3.28083989501312

extern EDMA3_CCRL_Regs *EDMA3_0_Regs;

volatile uint32_t index;

// test variables
extern float enc1;  // Left motor encoder
extern float enc2;  // Right motor encoder
extern float enc3;
extern float enc4;
extern float adcA0;  // ADC A0 - Gyro_X -400deg/s to 400deg/s  Pitch
extern float adcB0;  // ADC B0 - External ADC Ch4 (no protection circuit)
extern float adcA1;  // ADC A1 - Gyro_4X -100deg/s to 100deg/s  Pitch
extern float adcB1;  // ADC B1 - External ADC Ch1
extern float adcA2;  // ADC A2 -    Gyro_4Z -100deg/s to 100deg/s  Yaw
extern float adcB2;  // ADC B2 - External ADC Ch2
extern float adcA3;  // ADC A3 - Gyro_Z -400deg/s to 400 deg/s  Yaw
extern float adcB3;  // ADC B3 - External ADC Ch3
extern float adcA4;  // ADC A4 - Analog IR1
extern float adcB4;  // ADC B4 - USONIC1
extern float adcA5;  // ADC A5 -    Analog IR2
extern float adcB5;  // ADC B5 - USONIC2
extern float adcA6;  // ADC A6 - Analog IR3
extern float adcA7;  // ADC A7 - Analog IR4
extern float compass;
extern float switchstate;

extern sharedmemstruct *ptrshrdmem;

float vref = 1.0;
float turn = 0;

int tskcount = 0;
char fromLinuxstring[LINUX_COMSIZE + 2];
char toLinuxstring[LINUX_COMSIZE + 2];

float LVvalue1 = 0;
float LVvalue2 = 0;
int new_LV_data = 0;

int newnavdata = 0;
float newvref = 0;
float newturn = 0;
float new_ref_right_wall = 0;
float new_left_turn_Start_threshold = 0;
float new_left_turn_Stop_threshold = 0;
float new_Kp_right_wall = 0;
float new_Kp_front_wall = 0;
float new_front_turn_velocity = 0;
float new_turn_command_saturation = 0;

float ref_right_wall = 0;
float left_turn_Start_threshold = 0;
float left_turn_Stop_threshold = 0;
float Kp_right_wall = 0;
float Kp_front_wall = 0;
float front_turn_velocity = 0;
float turn_command_saturation = 0;

int wall_follow_state = 2;

extern sharedmemstruct *ptrshrdmem;

extern float newLADARdistance[LADAR_MAX_DATA_SIZE];  //in mm
extern float newLADARangle[LADAR_MAX_DATA_SIZE];		// in degrees
float LADARdistance[LADAR_MAX_DATA_SIZE];
float LADARangle[LADAR_MAX_DATA_SIZE];
extern pose ROBOTps;
extern pose LADARps;
extern float newLADARdataX[LADAR_MAX_DATA_SIZE];
extern float newLADARdataY[LADAR_MAX_DATA_SIZE];
float LADARdataX[LADAR_MAX_DATA_SIZE];
float LADARdataY[LADAR_MAX_DATA_SIZE];
extern int newLADARdata;

// Optitrack Variables
int trackableIDerror = 0;
int firstdata = 1;
volatile int new_optitrack = 0;
volatile float previous_frame = -1;
int frame_error = 0;
volatile float Optitrackdata[OPTITRACKDATASIZE];
pose OPTITRACKps;
float temp_theta = 0.0;
float tempOPTITRACK_theta = 0.0;
volatile int temp_trackableID = -1;
int trackableID = -1;
int errorcheck = 1;

float Vadc2 = 0.0;
float Vadc3 = 0.0;
float rate2 = 0.0;
float rate3 = 0.0;
int count = 0;

pose UpdateOptitrackStates(pose localROBOTps, int * flag);

void ComWithLinux(void) {

	int i = 0;
	Task_sleep(100);

	while(1) {

		Cache_inv((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
		//BCACHE_inv((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
		
		if (GET_DATA_FROM_LINUX) {

			if (newnavdata == 0) {
				newvref = ptrshrdmem->Floats_to_DSP[0];
				newturn = ptrshrdmem->Floats_to_DSP[1];
				new_ref_right_wall= ptrshrdmem->Floats_to_DSP[2];
				new_left_turn_Start_threshold= ptrshrdmem->Floats_to_DSP[3];
				new_left_turn_Stop_threshold= ptrshrdmem->Floats_to_DSP[4];
				new_Kp_right_wall= ptrshrdmem->Floats_to_DSP[5];
				new_Kp_front_wall= ptrshrdmem->Floats_to_DSP[6];
				new_front_turn_velocity= ptrshrdmem->Floats_to_DSP[7];
				new_turn_command_saturation= ptrshrdmem->Floats_to_DSP[8];
				newnavdata = 1;
			}

			CLR_DATA_FROM_LINUX;

		}

		if (GET_LVDATA_FROM_LINUX) {

			if (ptrshrdmem->DSPRec_size > 256) ptrshrdmem->DSPRec_size = 256;
				for (i=0;i<ptrshrdmem->DSPRec_size;i++) {
					fromLinuxstring[i] = ptrshrdmem->DSPRec_buf[i];
				}
				fromLinuxstring[i] = '\0';

				if (new_LV_data == 0) {
					sscanf(fromLinuxstring,"%f%f",&LVvalue1,&LVvalue2);
					new_LV_data = 1;
				}

			CLR_LVDATA_FROM_LINUX;

		}

		if ((tskcount%6)==0) {
			if (GET_LVDATA_TO_LINUX) {

				// Default
				ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"1.0 1.0 1.0 1.0");
				// you would do something like this
				//ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",var1,var2,var3,var4);

				for (i=0;i<ptrshrdmem->DSPSend_size;i++) {
					ptrshrdmem->DSPSend_buf[i] = toLinuxstring[i];
				}

				// Flush or write back source
				Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
				//BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

				CLR_LVDATA_TO_LINUX;

			}
		}
		
		if (GET_DATAFORFILE_TO_LINUX) {

			// This is an example write to scratch
			// The Linux program SaveScratchToFile can be used to write the
			// ptrshrdmem->scratch[0-499] memory to a .txt file.
//			for (i=100;i<300;i++) {
//				ptrshrdmem->scratch[i] = (float)i;
//			}

			// Flush or write back source
			Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
			//BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);

			CLR_DATAFORFILE_TO_LINUX;

		}

		tskcount++;
		Task_sleep(40);
	}


}


/*
 *  ======== main ========
 */
Int main()
{ 
	int i = 0;
	
    // unlock the system config registers.
    SYSCONFIG->KICKR[0] = KICK0R_UNLOCK;
    SYSCONFIG->KICKR[1] = KICK1R_UNLOCK;

    SYSCONFIG1->PUPD_SEL |= 0x10000000;  // change pin group 28 to pullup for GP7[12/13] (LCD switches)

    // Initially set McBSP1 pins as GPIO ins
    CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
    SETBIT(SYSCONFIG->PINMUX[1], 0x88888880);  // This is enabling the McBSP1 pins

    CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
    SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13
    CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
    SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13


    //Rick added for LCD DMA flagging test
    GPIO_setDir(GPIO_BANK0, GPIO_PIN8, GPIO_OUTPUT);
    GPIO_setOutput(GPIO_BANK0, GPIO_PIN8, OUTPUT_HIGH);

    GPIO_setDir(GPIO_BANK0, GPIO_PIN0, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN1, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN2, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN3, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN4, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN5, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK0, GPIO_PIN6, GPIO_INPUT);

    GPIO_setDir(GPIO_BANK7, GPIO_PIN8, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN9, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN10, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN11, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN12, GPIO_INPUT);
    GPIO_setDir(GPIO_BANK7, GPIO_PIN13, GPIO_INPUT);

    GPIO_setOutput(GPIO_BANK7, GPIO_PIN8, OUTPUT_HIGH);
    GPIO_setOutput(GPIO_BANK7, GPIO_PIN9, OUTPUT_HIGH);
    GPIO_setOutput(GPIO_BANK7, GPIO_PIN10, OUTPUT_HIGH);
    GPIO_setOutput(GPIO_BANK7, GPIO_PIN11, OUTPUT_HIGH);

    CLRBIT(SYSCONFIG->PINMUX[13], 0xFFFFFFFF);
    SETBIT(SYSCONFIG->PINMUX[13], 0x88888811); //Set GPIO 6.8-13 to GPIOs and IMPORTANT Sets GP6[15] to /RESETOUT used by PHY, GP6[14] CLKOUT appears unconnected

    //#warn GP6.15 is also connected to CAMERA RESET This is a Bug in my board design Need to change Camera Reset to different IO.

    GPIO_setDir(GPIO_BANK6, GPIO_PIN8, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN9, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN10, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN11, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN12, GPIO_OUTPUT);
    GPIO_setDir(GPIO_BANK6, GPIO_PIN13, GPIO_INPUT);


    while ((T1_TGCR & 0x7) != 0x7) {
      for (index=0;index<50000;index++) {}  // small delay before checking again

    }



    USTIMER_init();


    // Turn on McBSP1
    EVMOMAPL138_lpscTransition(PSC1, DOMAIN0, LPSC_MCBSP1, PSC_ENABLE);

    // If Linux has already booted It sets a flag so no need to delay
    if ( GET_ISLINUX_BOOTED == 0) {
        USTIMER_delay(4*DELAY_1_SEC);  // delay allowing Linux to partially boot before continuing with DSP code
    }
	
	// init the us timer and i2c for all to use.
	I2C_init(I2C0, I2C_CLK_100K);
	init_ColorVision();	
	init_LCD_mem(); // added rick
	
    EVTCLR0 = 0xFFFFFFFF;
    EVTCLR1 = 0xFFFFFFFF;
    EVTCLR2 = 0xFFFFFFFF;
    EVTCLR3 = 0xFFFFFFFF;

    init_DMA();
    init_McBSP();

	init_LADAR();
		
    CLRBIT(SYSCONFIG->PINMUX[1], 0xFFFFFFFF);
    SETBIT(SYSCONFIG->PINMUX[1], 0x22222220);  // This is enabling the McBSP1 pins

    CLRBIT(SYSCONFIG->PINMUX[5], 0x00FF0FFF);
    SETBIT(SYSCONFIG->PINMUX[5], 0x00110111);  // This is enabling SPI pins

    CLRBIT(SYSCONFIG->PINMUX[16], 0xFFFF0000);
    SETBIT(SYSCONFIG->PINMUX[16], 0x88880000);  // setup GP7.8 through GP7.13
    CLRBIT(SYSCONFIG->PINMUX[17], 0x000000FF);
    SETBIT(SYSCONFIG->PINMUX[17], 0x00000088);  // setup GP7.8 through GP7.13

	init_LCD();
	
	LADARps.x = 3.5/12; // 3.5/12 for front mounting
	LADARps.y = 0;
	LADARps.theta = 1;  // not inverted

	OPTITRACKps.x = 0;
	OPTITRACKps.y = 0;
	OPTITRACKps.theta = 0;

	for(i = 0;i<LADAR_MAX_DATA_SIZE;i++)
	{ LADARdistance[i] = LADAR_MAX_READING; } //initialize all readings to max value.

	// ROBOTps will be updated by Optitrack during gyro calibration
	// TODO: specify the starting position of the robot
	ROBOTps.x = 0;			//the estimate in array form (useful for matrix operations)
	ROBOTps.y = 0;
	ROBOTps.theta = 0;  // was -PI: need to flip OT ground plane to fix this
	
    // flag pins
    GPIO_setDir(IMAGE_TO_LINUX_BANK, IMAGE_TO_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(OPTITRACKDATA_FROM_LINUX_BANK, OPTITRACKDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(DATA_TO_LINUX_BANK, DATA_TO_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(DATA_FROM_LINUX_BANK, DATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(DATAFORFILE_TO_LINUX_BANK, DATAFORFILE_TO_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(LVDATA_FROM_LINUX_BANK, LVDATA_FROM_LINUX_FLAG, GPIO_OUTPUT);
    GPIO_setDir(LVDATA_TO_LINUX_BANK, LVDATA_TO_LINUX_FLAG, GPIO_OUTPUT);


    CLR_OPTITRACKDATA_FROM_LINUX;  // Clear = tell linux DSP is ready for new Opitrack data
    CLR_DATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new data
    CLR_DATAFORFILE_TO_LINUX;  // Clear = linux not requesting data
    SET_DATA_TO_LINUX;  // Set = put float array data into shared memory for linux
    SET_IMAGE_TO_LINUX;  // Set = put image into shared memory for linux
    CLR_LVDATA_FROM_LINUX;  // Clear = tell linux that DSP is ready for new LV data
    SET_LVDATA_TO_LINUX;  // Set = put LV char data into shared memory for linux

    // clear all possible EDMA
    EDMA3_0_Regs->SHADOW[1].ICR = 0xFFFFFFFF;

    // Add your init code here	
	
    BIOS_start();    /* does not return */
    return(0);
}

long timecount= 0;
int whichled = 0;
// This SWI is Posted after each set of new data from the F28335
void RobotControl(void) {

    count++;
    switch ((int)switchstate){
    case 0: //all switches backwards print LIDAR
        if ((count%250)==0){
            LCDPrintfLine(1,"-M: %d -H: %d ",(int)LADARdistance[1],(int)LADARdistance[54]);
            LCDPrintfLine(2,"0: %d H: %d h: %d",(int)LADARdistance[113],(int)LADARdistance[172],(int)LADARdistance[225]);
        }
        break;
    case 15: //all switches forward print IR
        if ((count%250)==0){
            LCDPrintfLine(1,"IR2: %.1f",(4095-adcA4));
            LCDPrintfLine(2,"IR3: %.1f",(4095-adcA5));
        }
        break;
    case 13: //2nd back compass
        if ((count%250)==0){
            LCDPrintfLine(1,"Compass: %.1f",compass);
            LCDPrintfLine(2,"");
        }
        break;
    case 11: //3rd back Gyro
        Vadc2 = ((adcA2*3.0)/4095.0);
        Vadc3 = ((adcA3*3.0)/4095.0);
        rate2 = (Vadc2-1.23)*100*(PI/180);
        rate3 = (Vadc3-1.23)*400*(PI/180);
        if ((count%250)==0){
            LCDPrintfLine(1,"Gryo2: %.3f",rate2);
            LCDPrintfLine(2,"Gyro3: %.3f",rate3);
        }
        break;
    case 7: //4th back vref turn
        if ((count%250)==0){
            LCDPrintfLine(1,"Vref: %.1f",vref);
            LCDPrintfLine(2,"Turn: %.1f",turn);
        }
        break;
    case 14:
        if ((count%250)==0){
            LCDPrintfLine(1,"Vref: %.1f Turn: %.1f",vref,turn);
            LCDPrintfLine(2,"Deg 0: %.1f",(int)LADARdistance[113]);
        }
        break;
    case 8:
        if ((count%250) ==0){
            LCDPrintfLine(1,"a%.1f b%.1f",new_ref_right_wall,new_left_turn_Start_threshold);
            LCDPrintfLine(2,"c%.1f d%.3f", new_left_turn_Stop_threshold,new_Kp_right_wall);
        }
        break;
    case 9:
        if ((count%250) ==0){
            LCDPrintfLine(1,"e%.3f f%.1f",new_Kp_front_wall,new_front_turn_velocity);
            LCDPrintfLine(2,"g%.1f", new_turn_command_saturation);
        }
        break;
    }


	int newOPTITRACKpose = 0;
	int i = 0;

	if (0==(timecount%1000)) {
		switch(whichled) {
		case 0:
			SETREDLED;
			CLRBLUELED;
			CLRGREENLED;
			whichled = 1;
			break;
		case 1:
			CLRREDLED;
			SETBLUELED;
			CLRGREENLED;
			whichled = 2;
			break;
		case 2:
			CLRREDLED;
			CLRBLUELED;
			SETGREENLED;
			whichled = 0;
			break;
		default:
			whichled = 0;
			break;
		}
	}
	
	if (GET_OPTITRACKDATA_FROM_LINUX) {

		if (new_optitrack == 0) {
			for (i=0;i<OPTITRACKDATASIZE;i++) {
				Optitrackdata[i] = ptrshrdmem->Optitrackdata[i];
			}
			new_optitrack = 1;
		}

		CLR_OPTITRACKDATA_FROM_LINUX;

	}

	if (new_optitrack == 1) {
		OPTITRACKps = UpdateOptitrackStates(ROBOTps, &newOPTITRACKpose);
		new_optitrack = 0;
	}
		
	if (newLADARdata == 1) {
		newLADARdata = 0;
		for (i=0;i<228;i++) {
			LADARdistance[i] = newLADARdistance[i];
			LADARangle[i] = newLADARangle[i];
			LADARdataX[i] = newLADARdataX[i];
			LADARdataY[i] = newLADARdataY[i];
		}
	}

	int minLADARfront = LADARdistance[111];
	for(i=112;i<116;i++) {
	    if (LADARdistance[i] < minLADARfront) {
	        minLADARfront = LADARdistance[i];
	    }
	}

	int minLADARright = LADARdistance[52];
	for(i=53;i<57;i++) {
	    if (LADARdistance[i] < minLADARright) {
	        minLADARright = LADARdistance[i];
	    }
	}

    int minLADARbackright = LADARdistance[0];
    for(i=1;i<6;i++) {
        if (LADARdistance[i] < minLADARbackright) {
            minLADARbackright = LADARdistance[i];
        }
    }

    float front_wall_error = 0;
    float right_wall_error = 0;

    switch(wall_follow_state){
    case 1: //object in front
        front_wall_error = 3000 - minLADARfront;
        turn = Kp_front_wall*front_wall_error;
        vref = -front_turn_velocity;
        if(minLADARfront > left_turn_Stop_threshold){
            wall_follow_state = 2;
        }else if(minLADARbackright > ref_right_wall){
            wall_follow_state = 3;
        }
        break;
    case 2: //object to the right
        right_wall_error = ref_right_wall - minLADARright;
        turn = -Kp_right_wall*right_wall_error;
        vref = newvref;
        if(minLADARfront < left_turn_Start_threshold){
            wall_follow_state = 1;
        }else if(minLADARright < ref_right_wall){
            wall_follow_state = 1;
        }
        break;
    case 3:
        front_wall_error = 3000 - minLADARfront;
        turn = Kp_front_wall*front_wall_error;
        vref = front_turn_velocity;
    }

    if(fabs(turn)>turn_command_saturation){
        turn = (fabs(turn)/turn)*turn_command_saturation;
    }

	if (newnavdata){//check if there is new data. This is check each time into the Swi
	    vref = newvref; // copy data from “new” variables to variables used in Swi
	    turn = newturn;
	    ref_right_wall = new_ref_right_wall;
	    left_turn_Start_threshold = new_left_turn_Start_threshold;
	    left_turn_Stop_threshold = new_left_turn_Stop_threshold;
	    Kp_right_wall = new_Kp_right_wall;
	    Kp_front_wall = new_Kp_front_wall;
	    front_turn_velocity = new_front_turn_velocity;
	    turn_command_saturation = new_turn_command_saturation;
	    newnavdata = 0;//set flag back to zero telling Task that Swi ready for next
	}

	//turn = -enc3/10.0;
	
	SetRobotOutputs(vref,turn,0,0,0,0,0,0,0,0);

	timecount++;
}

pose UpdateOptitrackStates(pose localROBOTps, int * flag) {

	pose localOPTITRACKps;

	// Check for frame errors / packet loss
	if (previous_frame == Optitrackdata[OPTITRACKDATASIZE-1]) {
		frame_error++;
	}
	previous_frame = Optitrackdata[OPTITRACKDATASIZE-1];

	// Set local trackableID if first receive data
	if (firstdata){
		//trackableID = (int)Optitrackdata[OPTITRACKDATASIZE-1]; // removed to add new trackableID in shared memory
		trackableID = Optitrackdata[OPTITRACKDATASIZE-2];
		firstdata = 0;
	}

	// Check if local trackableID has changed - should never happen
	if (trackableID != Optitrackdata[OPTITRACKDATASIZE-2]) {
		trackableIDerror++;
		// do some sort of reset(?)
	}

	// Save position and yaw data
	if (isnan(Optitrackdata[0]) != 1) {  // this checks if the position data being received contains NaNs
		// check if x,y,yaw all equal 0.0 (almost certainly means the robot is untracked)
		if ((Optitrackdata[0] != 0.0) && (Optitrackdata[1] != 0.0) && (Optitrackdata[2] != 0.0)) {
			// save x,y
			// adding 2.5 so everything is shifted such that optitrack's origin is the center of the arena (while keeping all coordinates positive)
			// This was the old way for Optitrack coordinates
			//localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
			//localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

			// This is the new coordinates for Motive
			localOPTITRACKps.x = -1.0*Optitrackdata[0]*FEETINONEMETER; 
			localOPTITRACKps.y = Optitrackdata[1]*FEETINONEMETER+4.0;

			// make this a function
			temp_theta = fmodf(localROBOTps.theta,(float)(2*PI));//(theta[trackableID]%(2*PI));
			tempOPTITRACK_theta = Optitrackdata[2];
			if (temp_theta > 0) {
				if (temp_theta < PI) {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA > 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta > (PI/2)) {
							// THETA > 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA > 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA > 0, kal in QIII, OT in QIII
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta > (3*PI/2)) {
							// THETA > 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA > 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = (floorf((localROBOTps.theta)/((float)(2.0*PI))))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			} else {
				if (temp_theta > -PI) {
					if (tempOPTITRACK_theta <= 0.0) {
						// THETA < 0, kal in QIII/IV, OT in QIII/IV
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
					} else {
						if (temp_theta < (-PI/2)) {
							// THETA < 0, kal in QIII, OT in QII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
						} else {
							// THETA < 0, kal in QIV, OT in QI
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				} else {
					if (tempOPTITRACK_theta >= 0.0) {
						// THETA < 0, kal in QI/II, OT in QI/II
						localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + tempOPTITRACK_theta*2*PI/360.0);
					} else {
						if (temp_theta < (-3*PI/2)) {
							// THETA < 0, kal in QI, OT in QIV
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI - 2*PI + tempOPTITRACK_theta*2*PI/360.0;
						} else {
							// THETA < 0, kal in QII, OT in QIII
							localOPTITRACKps.theta = ((int)((localROBOTps.theta)/(2*PI)))*2.0*PI + tempOPTITRACK_theta*2*PI/360.0;
						}
					}
				}
			}
			*flag = 1;
		}
	}
	return localOPTITRACKps;
}


