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
extern int flagVision;
extern int new_coordata;
extern int colorstate;

extern float green_object_x;
extern float green_object_y;
extern int green_numpels;

extern float blue_object_x;
extern float blue_object_y;
extern int blue_numpels;

float blue_center_x = 0;
float blue_center_y = 0;
int blue_area = 0;

float green_center_x = 0;
float green_center_y = 0;
int green_area = 0;

extern sharedmemstruct *ptrshrdmem;

float vref = 0;
float turn = 0;
int robotstate = 0;
int tskcount = 0;
char fromLinuxstring[LINUX_COMSIZE + 2];
char toLinuxstring[LINUX_COMSIZE + 2];

float LVvalue1 = 0;
float LVvalue2 = 0;
int new_LV_data = 0;

typedef struct edges_tag{
    int r;
    int c;
    int x;
    int y;
    int tally;
    int found;
    int idx1;
    int idx2;
    int idx3;
    int sendLV;
    int orientation;
    int found_flag;
}edges;
typedef struct box_tag{
    edges edge[4];
    int no_left_wall;
    int no_right_wall;
}box;



int newnavdata = 0;
float newvref = 0;
float newturn = 0;
int firsttime = 1;
extern sharedmemstruct *ptrshrdmem;

float x_pred[3][1] = {{0},{0},{0}};					// predicted state

//more kalman vars
float B[3][2] = {{1,0},{1,0},{0,1}};			// control input model
float u[2][1] = {{0},{0}};			// control input in terms of velocity and angular velocity
float Bu[3][1] = {{0},{0},{0}};	// matrix multiplication of B and u
float z[3][1];							// state measurement
float eye3[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// 3x3 identity matrix
float K[3][3] = {{1,0,0},{0,1,0},{0,0,1}};		// optimal Kalman gain
#define ProcUncert 0.0001
#define CovScalar 10
float Q[3][3] = {{ProcUncert,0,ProcUncert/CovScalar},
                 {0,ProcUncert,ProcUncert/CovScalar},
                 {ProcUncert/CovScalar,ProcUncert/CovScalar,ProcUncert}};	// process noise (covariance of encoders and gyro)
#define MeasUncert 1
float R[3][3] = {{MeasUncert,0,MeasUncert/CovScalar},
                 {0,MeasUncert,MeasUncert/CovScalar},
                 {MeasUncert/CovScalar,MeasUncert/CovScalar,MeasUncert}};	// measurement noise (covariance of LADAR)
float S[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance
float S_inv[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// innovation covariance matrix inverse
float P_pred[3][3] = {{1,0,0},{0,1,0},{0,0,1}};	// predicted covariance (measure of uncertainty for current position)
float temp_3x3[3][3];				// intermediate storage
float temp_3x1[3][1];				// intermediate storage
float ytilde[3][1];					// difference between predictions

// deadreckoning
float vel1 = 0,vel2 = 0;
float vel1old = 0,vel2old = 0;
float enc1old = 0,enc2old = 0;

// SETTLETIME should be an even number and divisible by 3
#define SETTLETIME 6000
int settlegyro = 0;
float gyro_zero = 0;
float gyro_angle = 0;
float old_gyro = 0;
float gyro_drift = 0;
float gyro = 0;
int gyro_degrees = 0;
float gyro_radians = 0.0;
float gyro_x = 0,gyro_y = 0;
float gyro4x_gain = 1;
extern float mydist;

int statePos = 0;	// index into robotdest
int robotdestSize = 8;	// number of positions to use out of robotdest
pose robotdest[8];	// array of waypoints for the robot

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

int RobotState = 1;
float Kp_obs = 0.02;
float FrontLeft_error = 0;
float FrontRight_error = 0;
float Kp_vref_obs = 0.001;

//DSP FLoat inits
int new_DSP_Float2 = 0; //Reference right wall
int new_DSP_Float3 = 0; //left turn  start threshold
int new_DSP_Float4 = 0; //left turn stop threshold
int new_DSP_Float5 = 0; //Kp right wall gain
int new_DSP_Float6 = 0; //KP front wall gain
int new_DSP_Float7 = 0; //front turn velocity
int new_DSP_Float8 = 0; //turn command saturation
int new_DSP_Float9 = 0; //sharp right threshold
int new_DSP_Float10 = 0; //Kp sharp right gain
int new_DSP_Float11 = 0; //left turn corner threshold
int new_DSP_Float12 = 0; //
int new_DSP_Float13 = 0; //
int new_DSP_Float14 = 0; //
int new_DSP_Float15 = 0; //
int new_DSP_Float16 = 0; //
int new_DSP_Float17 = 0; //
int new_DSP_Float18 = 0; //
int new_DSP_Float19 = 0; //

//DSP Floats to global vars
int DSP_Float2 = 300; //Reference right wall
int DSP_Float3 = 500; //left turn  start threshold
int DSP_Float4 = 800; //left turn stop threshold
int DSP_Float5 = 0.01; //Kp right wall gain
int DSP_Float6 = -0.005; //KP front wall gain
int DSP_Float7 = 0.4; //front turn velocity
int DSP_Float8 = 1.0; //turn command saturation
int DSP_Float9 = 400; //sharp right threshold
int DSP_Float10 = 0.02; //Kp sharp right gain
int DSP_Float11 = 400; //left turn corner threshold
int DSP_Float12 = 0; //
int DSP_Float13 = 0; //
int DSP_Float14 = 0; //
int DSP_Float15 = 0; //
int DSP_Float16 = 0; //
int DSP_Float17 = 0; //
int DSP_Float18 = 0; //
int DSP_Float19 = 0; //

float cntturn = 0;
float ref_right_wall = 300;
float left_turn_Start_threshold = 500;
float left_turn_Stop_threshold = 800;
float Kp_right_wall = 0.01;
float Kp_front_wall = -0.005;
float front_turn_velocity = 0.4;
float turn_command_saturation = 1.0;
float sharp_right_threshold = 400;
float Kp_sharp_right = 0.02;
float left_turn_corner_threshold = 400;
float KpGyro = 400;
//send obstacle x,y,orientation to labview
float data1 = 0.0;
float data2 = 0.0;
float data3 = 0.0;

float p1_old = 0;
float p2_old = 0;
float angular_vel = 0;
float orientation = 0;
float old_angular_vel = 0;
float p1_current = 0;
float p2_current = 0;
float v1 = 0;
float v2 = 0;
float x = 0;
float y = 0;
int update_Timer = 0;
int countdown = 7000;
int astarRunning = 0;
int astarTrigger = 0;  // Just a Debug variable value prints in Linux each A* run.
int astarDone = 0;
int pathLen = 0;            //used to keep track of number of points in reconstructed path
int pathPos=0;
int pathRow[100];         //reconstructed path in reverse order
int pathCol[100];

int newLVData = 0;
float newROBOTpsx = 0.0;
float newROBOTpsy = 0.0;
float sharedROBOTpsx = 0.0;
float sharedROBOTpsy = 0.0;

char map[176] =         //16x11
{   '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'   };

char originalMap[176] =
{   '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    'x', 'x', 'x', 'x', '0', '0', '0', 'x', 'x', 'x', 'x',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
    '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'   };


edges obs[57];
box boxes[30];
int myRound(float var) {

    if ( ( fabs(var) - floorf( fabs(var) ) ) > 0.5) {
        if (var > 0.0) {return ceil(var);}
        else {return floor(var);}
    }
    else {
        if (var > 0.0) {return floor(var);}
        else {return ceil(var);}
    }
}
float xdist[11];
float ydist[11];
float dist[11];
float distthresh = .25;
float mindist = 100;


// This is telling Astar to RUN!
void runAstar(void) {
    float robotdestx = 0;
    float robotdesty = 0;
    float robotx = 0;
    float roboty = 0;
    int local_statePos = 0;
    while(1) {
        int i = 0;
        Semaphore_pend(SEM_startAstar,BIOS_WAIT_FOREVER);

        if(!GET_ASTAR_COMMAND && !astarRunning) {
            astarRunning = 1;
            astarTrigger = 0;   // Debug print in Linux
            robotx = ROBOTps.x;
            roboty = ROBOTps.y;
            local_statePos = statePos;
            robotdestx = robotdest[local_statePos].x;
            robotdesty = robotdest[local_statePos].y;

            //For now, Update the shared variables that need to be sent for astar
            for (i=0;i<176;i++) {
                ptrshrdmem->sharedAstarMap[i] = map[i];
            }

            ptrshrdmem->astarTrigger = astarTrigger;
            ptrshrdmem->sharedDestRow = (11.0 - robotdesty);
            ptrshrdmem->sharedDestCol = (5.0 + robotdestx);
            ptrshrdmem->sharedRobotRow = (11.0 - myRound(roboty));
            ptrshrdmem->sharedRobotCol = (myRound(robotx) + 5.0);

            Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
            //BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
            SET_ASTAR_COMMAND;
        }
    }

}

// Periodic
void LinuxAstar(void) {
    float robotdestx = 0;
    float robotdesty = 0;
    float robotx = 0;
    float roboty = 0;
    int local_statePos = 0;
    int i = 0;
    Task_sleep(10);
    while(1) {
        Task_sleep(5);
        //GET_STAR_DONE is set when linux finishes running the a star algo irrespective of whether its succesful or failed
        if (GET_ASTAR_DONE) {
            //In case it failed-> couldn't find path, map is reset to original, only walls everything else cleared
            if (GET_ASTAR_FAILED) {

                CLR_ASTAR_DONE;
                robotx = ROBOTps.x;
                roboty = ROBOTps.y;
                local_statePos = statePos;
                robotdestx = robotdest[local_statePos].x;
                robotdesty = robotdest[local_statePos].y;
                astarRunning = 1;
                astarTrigger = 5;   // Debug print in Linux

                // If we get here, there was no path found between the robot and one of the waypoints.
                // The map is reset
                for (i=0;i<176;i++) {
                    map[i] = originalMap[i];
                    ptrshrdmem->sharedAstarMap[i] = map[i];
                }

                // ALSO make sure to clear your obstacle tally an other obstacle recognition variables you have created.
                for(i=0;i<57;i++){
                    obs[i].tally = 0;
                    obs[i].sendLV = 0;
                    obs[i].found_flag = 0;
                }

                ptrshrdmem->astarTrigger = astarTrigger;
                ptrshrdmem->sharedDestRow = (11.0 - robotdesty);
                ptrshrdmem->sharedDestCol = (5.0 + robotdestx);
                ptrshrdmem->sharedRobotRow = (11.0 - myRound(roboty));
                ptrshrdmem->sharedRobotCol = (myRound(robotx) + 5.0);

                Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
                //BCACHE_wb((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
                SET_ASTAR_COMMAND; //set every time we want to go into astar as opposed to done which is cleared only when we our astar is succesful

            }
            else { // Astar successful
                Cache_inv((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);
                //BCACHE_inv((void *)ptrshrdmem,sizeof(sharedmemstruct),EDMA3_CACHE_WAIT);
                pathLen = ptrshrdmem->sharedPathLen;

                for (i=0;i<pathLen;i++) {

                    pathCol[pathLen - i - 1] = (ptrshrdmem->sharedPathCol[i] - 5.0);
                    pathRow[pathLen - i - 1] = (11.0 - ptrshrdmem->sharedPathRow[i]); // Inverting the order of array. And then converting from Astar to Robot coordinates
                }
                CLR_ASTAR_DONE;
                if (pathLen > 1) {
                    pathPos = 1;
                } else {
                    pathPos = 0;
                }

                astarRunning = 0; // basic functionality same as CLR_ASTAR_DONE but on DSP side this is more reliable
                astarDone = 1;
                //.. given CLR_ASTAR_DONE can also be set by linux
            }
        }
    }
}
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
                new_DSP_Float2 = ptrshrdmem->Floats_to_DSP[2];
                new_DSP_Float3 = ptrshrdmem->Floats_to_DSP[3];
                new_DSP_Float4 = ptrshrdmem->Floats_to_DSP[4];
                new_DSP_Float5 = ptrshrdmem->Floats_to_DSP[5];
                new_DSP_Float6 = ptrshrdmem->Floats_to_DSP[6];
                new_DSP_Float7 = ptrshrdmem->Floats_to_DSP[7];
                new_DSP_Float8 = ptrshrdmem->Floats_to_DSP[8];
                new_DSP_Float9 = ptrshrdmem->Floats_to_DSP[9];
                new_DSP_Float10 = ptrshrdmem->Floats_to_DSP[10];
                new_DSP_Float10 = ptrshrdmem->Floats_to_DSP[11];
                new_DSP_Float12 = ptrshrdmem->Floats_to_DSP[12];
                new_DSP_Float13 = ptrshrdmem->Floats_to_DSP[13];
                new_DSP_Float14 = ptrshrdmem->Floats_to_DSP[14];
                new_DSP_Float15 = ptrshrdmem->Floats_to_DSP[15];
                new_DSP_Float16 = ptrshrdmem->Floats_to_DSP[16];
                new_DSP_Float17 = ptrshrdmem->Floats_to_DSP[17];
                new_DSP_Float18 = ptrshrdmem->Floats_to_DSP[18];
                new_DSP_Float19 = ptrshrdmem->Floats_to_DSP[19];
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

                for (i = 0; i<57; i++){
                    if(obs[i].sendLV == 1){
                        data1 = obs[i].x;
                        data2 = obs[i].y;
                        data3 = obs[i].orientation;
                        obs[i].sendLV = 0;
                        break;
                    }
                }
                if(newLVData == 1){
                    newROBOTpsx = sharedROBOTpsx;
                    newROBOTpsy = sharedROBOTpsy;
                    newLVData = 0;
                }
                // Default
                ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f %.1f",(newROBOTpsx+10),(13-newROBOTpsy),data1, data2);
                // you would do something like this
                //ptrshrdmem->DSPSend_size = sprintf(toLinuxstring,"%.1f %.1f %.1f",var1,var2,var3,var4);

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
            // First make sure all scratch elements are zero
            for (i=0;i<500;i++) {
                ptrshrdmem->scratch[i] = 0;
            }
            // Write LADARdataX to scratch
            for (i=0;i<228;i++) {
                ptrshrdmem->scratch[i] = LADARdataX[i];
            }
            // Write LADARdataY to scratch
            for (i=0;i<228;i++) {
                ptrshrdmem->scratch[228+i] = LADARdataY[i];
            }
            // Flush or write back source
            Cache_wb((void *)ptrshrdmem,sizeof(sharedmemstruct), Cache_Type_ALL, EDMA3_CACHE_WAIT);

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

#warn GP6.15 is also connected to CAMERA RESET This is a Bug in my board design Need to change Camera Reset to different IO.

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
    x_pred[0][0] = ROBOTps.x; //estimate in structure form (useful elsewhere)
    x_pred[1][0] = ROBOTps.y;
    x_pred[2][0] = ROBOTps.theta;

    // TODO: defined destinations that moves the robot around and outside the course
    robotdest[0].x = 0; 	robotdest[0].y = 0;
    robotdest[1].x = -5;	robotdest[1].y = 9;
    robotdest[2].x = 5;     robotdest[2].y = 9;
    robotdest[3].x = 0;    robotdest[3].y = 0;
    robotdest[4].x = 5;     robotdest[4].y = 9;
    robotdest[5].x = -5;    robotdest[5].y = 9;
    robotdest[6].x = 5;     robotdest[6].y = 9;
    robotdest[7].x = -5;    robotdest[7].y = 9;
    //    //middle of bottom
    //    robotdest[2].x = 0;		robotdest[2].y = 5;
    //    //outside the course
    //    robotdest[3].x = 4;		robotdest[3].y = 10;
    //    //back to middle
    //    robotdest[4].x = 0;		robotdest[4].y =5;
    //    robotdest[5].x = -4;		robotdest[5].y = -10;
    //    robotdest[6].x =0;		robotdest[6].y = 5;
    //    robotdest[7].x = 4;		robotdest[7].y = -10;
    for(i=0; i<6; i++){
        obs[i].x = -5 + 2*i;
        obs[i].y = 10;
        obs[i].r = 1;
        obs[i].c = 2*i;
        obs[i].tally = 0;
        obs[i].found = 0;
        if(obs[i].c == 0){
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        else if(obs[i].c == 10){
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
        }
        else{
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        obs[i].orientation = 0; // horizontal orientation
        obs[i].sendLV = 0;
    }
    for(i=6; i<11; i++){
        obs[i].x = -4 + 2*(i % 6);
        obs[i].y = 9;
        obs[i].r = 2;
        obs[i].c = 1 + 2*(i % 6);
        obs[i].tally = 0;
        obs[i].found = 0;
        obs[i].idx1 = (obs[i].r-1) *11 + obs[i].c;
        obs[i].idx2 = obs[i].r *11 + obs[i].c;
        obs[i].idx3 = (obs[i].r+1) * 11 + obs[i].c;
        obs[i].orientation = 1; // vertical orientation
        obs[i].sendLV = 0;
    }
    for(i=11; i<17; i++){
        obs[i].x = -5 + 2*((i+1)%6);
        obs[i].y = 8;
        obs[i].r = 3;
        obs[i].c = 2*((i+1)%6);
        obs[i].tally = 0;
        obs[i].found = 0;
        if(obs[i].c == 0){
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        else if(obs[i].c == 10){
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
        }
        else{
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        obs[i].orientation = 0; // horizontal orientation
        obs[i].sendLV = 0;
    }
    for(i=17; i<22; i++){
        obs[i].x = -4 + 2*((i+1)%6);
        obs[i].y = 7;
        obs[i].tally = 0;
        obs[i].found = 0;
        obs[i].r = 4;
        obs[i].c = 1 + 2*((i+1)%6);
        obs[i].idx1 = (obs[i].r-1) *11 + obs[i].c;
        obs[i].idx2 = obs[i].r *11 + obs[i].c;
        obs[i].idx3 = (obs[i].r+1) * 11 + obs[i].c;
        obs[i].orientation = 1; // vertical orientation
        obs[i].sendLV = 0;
    }
    for(i=22; i<28; i++){
        obs[i].x = -5 + 2*((i+2)%6);
        obs[i].y = 6;
        obs[i].r = 5;
        obs[i].c =  2*((i+2)%6);
        obs[i].tally = 0;
        obs[i].found = 0;
        if(obs[i].c == 0){
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        else if(obs[i].c == 10){
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
        }
        else{
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        obs[i].orientation = 0; // horizontal orientation
        obs[i].sendLV = 0;
    }
    for(i=28; i<33; i++){
        obs[i].x = -4 + 2*((i+2)%6);
        obs[i].y = 5;
        obs[i].tally = 0;
        obs[i].found = 0;
        obs[i].r = 6;
        obs[i].c = 1 + 2*((i+2)%6);
        obs[i].idx1 = (obs[i].r-1) *11 + obs[i].c;
        obs[i].idx2 = obs[i].r *11 + obs[i].c;
        obs[i].idx3 = (obs[i].r+1) * 11 + obs[i].c;
        obs[i].orientation = 1; // vertical orientation
        obs[i].sendLV = 0;
    }
    for(i=33; i<39; i++){
        obs[i].x = -5 + 2*((i+3)%6);
        obs[i].y = 4;
        obs[i].r = 7;
        obs[i].c = 2*((i+3)%6);
        obs[i].tally = 0;
        obs[i].found = 0;
        if(obs[i].c == 0){
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        else if(obs[i].c == 10){
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
        }
        else{
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        obs[i].orientation = 0; // horizontal orientation
        obs[i].sendLV = 0;
    }
    for(i=39; i<44; i++){
        obs[i].x = -4 + 2*((i+3)%6);
        obs[i].y = 3;
        obs[i].tally = 0;
        obs[i].found = 0;
        obs[i].r = 8;
        obs[i].c = 1 + 2*((i+3)%6);
        obs[i].idx1 = (obs[i].r-1) *11 + obs[i].c;
        obs[i].idx2 = obs[i].r *11 + obs[i].c;
        obs[i].idx3 = (obs[i].r+1) * 11 + obs[i].c;
        obs[i].orientation = 1; // vertical orientation
        obs[i].sendLV = 0;
    }
    for(i=44; i<50; i++){
        obs[i].x = -5 + 2*((i+4)%6);
        obs[i].y = 2;
        obs[i].r = 9;
        obs[i].c = 2*((i+4)%6);
        obs[i].tally = 0;
        obs[i].found = 0;
        if(obs[i].c == 0){
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        else if(obs[i].c == 10){
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
        }
        else{
            obs[i].idx1 = obs[i].r *11 + (obs[i].c-1);
            obs[i].idx2 = obs[i].r *11 + (obs[i].c);
            obs[i].idx3 = obs[i].r * 11 + (obs[i].c+1);
        }
        obs[i].orientation = 0; // horizontal orientation
        obs[i].sendLV = 0;
    }
    for(i=50; i<55; i++){
        obs[i].x = -4 + 2*((i+4)%6);
        obs[i].y = 1;
        obs[i].tally = 0;
        obs[i].found = 0;
        obs[i].r = 10;
        obs[i].c = 1 + 2*((i+4)%6);
        obs[i].idx1 = (obs[i].r-1) *11 + obs[i].c;
        obs[i].idx2 = obs[i].r *11 + obs[i].c;
        obs[i].idx3 = (obs[i].r+1) * 11 + obs[i].c;
        obs[i].orientation = 1; // vertical orientation
        obs[i].sendLV = 0;
    }
    for(i=0;i<5;i++){
        obs[11*i].idx1 = 400;
        obs[11*i + 5].idx3 = 400;
    }
    obs[55].x = -1;
    obs[55].y = 0;
    obs[55].r = 10;
    obs[55].c = 4;
    obs[55].tally = 0;
    obs[55].found = 0;
    obs[55].idx1 = obs[55].r *11 + obs[55].c-1;
    obs[55].idx2 = obs[55].r *11 + obs[55].c;
    obs[55].idx3 = obs[55].r * 11 + obs[55].c+1;
    obs[55].orientation = 0; // horizontal orientation
    obs[55].sendLV = 0;
    obs[56].x = 1;
    obs[56].y = 0;
    obs[56].r = 10;
    obs[56].c = 6;
    obs[56].tally = 0;
    obs[56].found = 0;
    obs[56].idx1 = obs[56].r *11 + obs[56].c-1;
    obs[56].idx2 = obs[56].r *11 + obs[56].c;
    obs[56].idx3 = obs[56].r * 11 + obs[56].c+1;
    obs[56].orientation = 0; // horizontal orientation
    obs[56].sendLV = 0;
    for(i = 0; i < 30; i ++){
        boxes[i].no_left_wall = 0;
        boxes[i].no_right_wall = 0;
    }
    for(i = 0; i < 5; i ++){
        boxes[i].edge[0] = obs[11*i]; // top, boxes go down from top left to bottom left and then shift over a column
        boxes[i].edge[1] = obs[6 + 11*i]; // right
        boxes[i].edge[2] = obs[11 + 11*i]; // bottom
        boxes[i].no_left_wall = 1;
    }
    for(i = 5; i < 10; i ++){
        boxes[i].edge[0] = obs[1 + 11*(i%5)];
        boxes[i].edge[1] = obs[7 + 11*(i%5)];
        boxes[i].edge[2] = obs[12 + 11*(i%5)];
        boxes[i].edge[3] = obs[6 + 11*(i%5)];
    }
    for(i = 10; i < 15; i ++){
        boxes[i].edge[0] = obs[2 + 11*(i%5)];
        boxes[i].edge[1] = obs[8 + 11*(i%5)];
        boxes[i].edge[2] = obs[13 + 11*(i%5)];
        boxes[i].edge[3] = obs[7 + 11*(i%5)];
    }
    for(i = 15; i < 20; i ++){
        boxes[i].edge[0] = obs[3 + 11*(i%5)];
        boxes[i].edge[1] = obs[9 + 11*(i%5)];
        boxes[i].edge[2] = obs[13 + 11*(i%5)];
        boxes[i].edge[3] = obs[8 + 11*(i%5)];
    }
    for(i = 20; i < 25; i ++){
        boxes[i].edge[0] = obs[4 + 11*(i%5)];
        boxes[i].edge[1] = obs[10 + 11*(i%5)];
        boxes[i].edge[2] = obs[14 + 11*(i%5)];
        boxes[i].edge[3] = obs[9 + 11*(i%5)];
    }
    for(i = 25; i < 30; i ++){
        boxes[i].edge[0] = obs[5 + 11*(i%5)];
        boxes[i].edge[2] = obs[15 + 11*(i%5)];
        boxes[i].edge[3] = obs[10 + 11*(i%5)];
        boxes[i].no_right_wall = 1;
    }
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

    int minLADARfrontright = LADARdistance[29];
    for(i=30;i<105;i++) { //29 105
        if (LADARdistance[i] < minLADARfrontright) {
            minLADARfrontright = LADARdistance[i];
        }
    }


    int minLADARfrontleft = LADARdistance[124];
    for(i=125;i<196;i++) { //124 196
        if (LADARdistance[i] < minLADARfrontleft) {
            minLADARfrontleft = LADARdistance[i];
        }
    }


    int minLADARfront = LADARdistance[110];
    for(i=111;i<116;i++) {//110 116
        if (LADARdistance[i] < minLADARfront) {
            minLADARfront = LADARdistance[i];
        }
    }

    int minLADARright = LADARdistance[52];
    for(i=53;i<56;i++) {// 52 56
        if (LADARdistance[i] < minLADARright) {
            minLADARright = LADARdistance[i];
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

    // using 400deg/s gyro
    gyro = adcA3*3.0/4096.0;
    if (settlegyro < SETTLETIME) {
        settlegyro++;
        if (settlegyro < (SETTLETIME/3)) {
            // do nothing
        } else if (settlegyro < (2*SETTLETIME/3)) {
            gyro_zero = gyro_zero + gyro/(SETTLETIME/3);
        } else {
            gyro_drift += (((gyro-gyro_zero) + old_gyro)*.0005)/(SETTLETIME/3);
            old_gyro = gyro-gyro_zero;
        }
        if(settlegyro%500 == 0) {
            LCDPrintfLine(1,"Cal Gyro -- %.1fSecs", (float)(SETTLETIME - settlegyro)/1000.0 );
            LCDPrintfLine(2,"");
        }
        enc1old = enc1;
        enc2old = enc2;
        newOPTITRACKpose = 0;
        switchstate = 3;

        SetRobotOutputs(0,0,0,0,0,0,0,0,0,0);
    } else {

        gyro_angle = gyro_angle - ((gyro-gyro_zero) + old_gyro)*.0005 + gyro_drift;
        old_gyro = gyro-gyro_zero;
        gyro_radians = (gyro_angle * (PI/180.0)*400.0*gyro4x_gain);

        // Kalman filtering
        vel1 = (enc1 - enc1old)/(193.0*0.001);	// calculate actual velocities
        vel2 = (enc2 - enc2old)/(193.0*0.001);
        if (fabsf(vel1) > 10.0) vel1 = vel1old;	// check for encoder roll-over should never happen
        if (fabsf(vel2) > 10.0) vel2 = vel2old;
        enc1old = enc1;	// save past values
        enc2old = enc2;
        vel1old = vel1;
        vel2old = vel2;

        // Step 0: update B, u
        B[0][0] = cosf(ROBOTps.theta)*0.001;
        B[1][0] = sinf(ROBOTps.theta)*0.001;
        B[2][1] = 0.001;
        u[0][0] = 0.5*(vel1 + vel2);	// linear velocity of robot
        u[1][0] = (gyro-gyro_zero)*(PI/180.0)*400.0*gyro4x_gain;	// angular velocity in rad/s (negative for right hand angle)

        // Step 1: predict the state and estimate covariance
        Matrix3x2_Mult(B, u, Bu);					// Bu = B*u
        Matrix3x1_Add(x_pred, Bu, x_pred, 1.0, 1.0); // x_pred = x_pred(old) + Bu
        Matrix3x3_Add(P_pred, Q, P_pred, 1.0, 1.0);	// P_pred = P_pred(old) + Q
        // Step 2: if there is a new measurement, then update the state
        if (1 == newOPTITRACKpose) {
            newOPTITRACKpose = 0;
            z[0][0] = OPTITRACKps.x;	// take in the LADAR measurement
            z[1][0] = OPTITRACKps.y;
            // fix for OptiTrack problem at 180 degrees
            if (cosf(ROBOTps.theta) < -0.99) {
                z[2][0] = ROBOTps.theta;
            }
            else {
                z[2][0] = OPTITRACKps.theta;
            }
            // Step 2a: calculate the innovation/measurement residual, ytilde
            Matrix3x1_Add(z, x_pred, ytilde, 1.0, -1.0);	// ytilde = z-x_pred
            // Step 2b: calculate innovation covariance, S
            Matrix3x3_Add(P_pred, R, S, 1.0, 1.0);							// S = P_pred + R
            // Step 2c: calculate the optimal Kalman gain, K
            Matrix3x3_Invert(S, S_inv);
            Matrix3x3_Mult(P_pred,  S_inv, K);								// K = P_pred*(S^-1)
            // Step 2d: update the state estimate x_pred = x_pred(old) + K*ytilde
            Matrix3x1_Mult(K, ytilde, temp_3x1);
            Matrix3x1_Add(x_pred, temp_3x1, x_pred, 1.0, 1.0);
            // Step 2e: update the covariance estimate   P_pred = (I-K)*P_pred(old)
            Matrix3x3_Add(eye3, K, temp_3x3, 1.0, -1.0);
            Matrix3x3_Mult(temp_3x3, P_pred, P_pred);

            //our code for robot orientation and position
            angular_vel = (((((adcA3*3.0)/4095.0) - gyro_zero)*PI)/180)*KpGyro;
            orientation += ((angular_vel+old_angular_vel)/2)*0.001;
            if(abs(orientation)>(2*PI)){
                orientation = (abs(orientation)-2*PI)*abs(orientation)/orientation;
            }


        }	// end of correction step

        // set ROBOTps to the updated and corrected Kalman values.
        ROBOTps.x = x_pred[0][0];
        ROBOTps.y = x_pred[1][0];
        ROBOTps.theta = x_pred[2][0];

        if(newLVData == 0){
            sharedROBOTpsx = ROBOTps.x;
            sharedROBOTpsy = ROBOTps.y;
            newLVData = 1;
        }

        if (firsttime ==1  && timecount > 200) {
            Semaphore_post(SEM_startAstar);
            firsttime = 0;
        }
        //        if (countdown == 0){
        //            p1_current = enc1/192.0;
        //            p2_current = enc2/192.0;
        //
        //            v1 = (p1_current-p1_old)/0.001;
        //            v2 = (p2_current-p2_old)/0.001;
        //
        //            //pos = pos + ((v1+v2)/2)*0.001;
        //            x += ((v1+v2)/2)*0.001 * cos(orientation);
        //            y += ((v1+v2)/2)*0.001 * sin(orientation);
        //            p1_old = p1_current;
        //            p2_old = p2_current;
        //            old_angular_vel = angular_vel;
        //        }else{ countdown--;}
        //
        //        //correct gyro drift
        //        if (update_Timer == 0){
        //            if ((minLADARfront < left_turn_Start_threshold) && (minLADARright < ref_right_wall) && (green_area > 500) ) {
        //                if ((compass > 3300) || (compass < 300)) { //determine which corner
        //                    x = 5;
        //                    y = 11;
        //                    orientation = PI/2;
        //                }else if ((compass > 2400) && (compass < 3000)){
        //                    x = -5;
        //                    y = 11;
        //                    orientation = PI;
        //                }
        //                update_Timer = 2000;
        //            }
        //        }else{
        //            update_Timer--;
        //        }

        //LCD Printing
        switch((int)(switchstate)) {
        case 0:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case %d RbSt: %d",(int)(switchstate),RobotState);
                LCDPrintfLine(2,"FL: %d FR: %d",minLADARfrontleft,minLADARfrontright);
            }
            break;
        case 1:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case %d",(int)(switchstate));
                LCDPrintfLine(2,"V: %.2f T: %.2f",vref,turn);
            }
            break;
        case 2:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case %d",(int)(switchstate));
                LCDPrintfLine(2,"x: %.1f y: %.1f",ROBOTps.x,ROBOTps.y);
            }
            break;
        case 3:
            if ((timecount%250)==0){ //gyro
                LCDPrintfLine(1,"Cal Gyro -- %.1fSecs", (float)(SETTLETIME - settlegyro)/1000.0 );
                LCDPrintfLine(2,"");
            }
            break;
        case 4:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 5:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 6:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 7:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 8:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 9:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 10:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 11:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 12:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 13:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 14:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        case 15:
            if ((timecount%250)==0){
                LCDPrintfLine(1,"Case");
                LCDPrintfLine(2,"%d",(int)(switchstate));
            }
            break;
        }




        int j;
        if (newLADARdata == 1) {
            newLADARdata = 0;
            for (i=0;i<228;i++) {
                LADARdistance[i] = newLADARdistance[i];
                LADARangle[i] = newLADARangle[i];
                LADARdataX[i] = newLADARdataX[i];
                LADARdataY[i] = newLADARdataY[i];
            }
            //            LADARdataX[30] = 0;
            //            LADARdataY[30] = 9;
            for(i=0;i<11;i++){
                xdist[i] = LADARdataX[30 + 17*i];
                ydist[i] = LADARdataY[30 + 17*i];
                if(LADARdistance[30 + 17*i] > 1800){
                    xdist[i] = 170; // guaranteed not to be inside course so value will be ignored
                    ydist[i] = 170;// guaranteed not to be inside course ...
                }
            }
            for(i=0;i<57;i++){
                mindist =  100;
                if(obs[i].found_flag != 1){
                    for(j=0;j<11;j++){
                        dist[j] = sqrtf(((xdist[j] - obs[i].x)*(xdist[j] - obs[i].x)) + ((ydist[j] - obs[i].y)*(ydist[j] - obs[i].y)));
                        if(mindist > dist[j]){
                            mindist = dist[j];
                        }
                    }
                    if(mindist < distthresh){
                        obs[i].tally++;
                    }
                    if((obs[i].tally > 4) && (obs[i].orientation == 1)){
                        map[obs[i].idx1] = 'x';
                        map[obs[i].idx2] = 'x';
                        map[obs[i].idx3] = 'x';
                        obs[i].sendLV = 1;
                        obs[i].found_flag = 1;
                        if(orientation = 0 ){

                        }
                        Semaphore_post(SEM_startAstar);
                    }
                    if((obs[i].tally > 4) && (obs[i].orientation == 0) && (obs[i].idx1 == 400)){
                        map[obs[i].idx2] = 'x';
                        map[obs[i].idx3] = 'x';
                        obs[i].sendLV = 1;
                        obs[i].found_flag = 1;
                        Semaphore_post(SEM_startAstar);
                    }
                    if((obs[i].tally > 4) && (obs[i].orientation == 0) && (obs[i].idx3 == 400)){
                        map[obs[i].idx1] = 'x';
                        map[obs[i].idx2] = 'x';
                        obs[i].sendLV = 1;
                        obs[i].found_flag = 1;
                        Semaphore_post(SEM_startAstar);
                    }
                    if((obs[i].tally > 4) && (obs[i].orientation == 0) && !(map[obs[i].idx2] == 'x')){
                        map[obs[i].idx1] = 'x';
                        map[obs[i].idx2] = 'x';
                        map[obs[i].idx3] = 'x';
                        obs[i].sendLV = 1;
                        obs[i].found_flag = 1;
                        Semaphore_post(SEM_startAstar);
                    }
                }
            }
        }

        //        float obstacle_avoidance_threshold = 0.5;
        //        float prev_obstacle_avoidance_threshold = 1.0;
        //        float dx;
        //        float dy;
        //        float dist = 0.0F;
        //        float dx_prev;
        //        float dy_prev;
        //        float prev_dist = 0.0F;
        //        int prevstatePos = 0;
        // uses xy code to step through an array of positions
        //        dx = robotdest[statePos].x - ROBOTps.x;
        //        dy = robotdest[statePos].y - ROBOTps.y;
        //        dist = sqrtf( dx*dx + dy*dy );
        //        if(statePos != 0){
        //            prevstatePos = statePos - 1;
        //            dx_prev = robotdest[prevstatePos].x - ROBOTps.x;
        //            dy_prev = robotdest[prevstatePos].y - ROBOTps.y;
        //            prev_dist = sqrtf(dx_prev*dx_prev + dy_prev*dy_prev);
        //        }

        //        if( xy_control(&vref, &turn, 3.0, ROBOTps.x, ROBOTps.y, robotdest[statePos].x, robotdest[statePos].y, ROBOTps.theta, 0.25, 0.5))
        //        { statePos = (statePos+1)%robotdestSize; }
        switch(robotstate){
        case 1:
                if( xy_control(&vref, &turn, 3.0, ROBOTps.x, ROBOTps.y, robotdest[statePos].x, robotdest[statePos].y, ROBOTps.theta, 0.25, 0.5))
                { statePos = (statePos+1)%robotdestSize; }
                if(ROBOTps.y >= 0){
                    robotstate = 2;
                }
        case 2:
        if( xy_control(&vref, &turn, 2.0, ROBOTps.x, ROBOTps.y, pathCol[pathPos], pathRow[pathPos], ROBOTps.theta, 0.25, 0.5))
        {
            pathPos++;
            if (pathPos == pathLen) {
                statePos = (statePos+1)%robotdestSize;
                Semaphore_post(SEM_startAstar);
            }
            if(ROBOTps.y < 0){
                robotstate = 1;
            }

        }
        }
        // obstacle avoidance maybe use ultrasonic sensors

        //
        //        // everytime an obstacle is recorded we call A star in the map code
        //        FrontLeft_error = 600 - minLADARfrontleft;
        //        FrontRight_error = 600 - minLADARfrontright;
        //
        //        //        if((dist > obstacle_avoidance_threshold) || (prev_dist > prev_obstacle_avoidance_threshold)){
        //        switch(RobotState) {
        //        case 1: //Dodge to the right
        //            turn += Kp_obs*FrontLeft_error;
        //            //vref -= Kp_vref_obs*FrontLeft_error;
        //            //                if (350 > minLADARfrontright)  {
        //            //                    RobotState = 4;
        //            //                }else
        //            if (minLADARfrontleft < 600){
        //                RobotState = 1;
        //            }else {
        //                RobotState = 3;
        //            }
        //            break;
        //        case 2: //Dodge to the left
        //            turn -= Kp_obs*FrontRight_error;
        //            //vref -= Kp_vref_obs*FrontRight_error;
        //            //                if (350 > minLADARfrontleft) {
        //            //                    RobotState = 4;
        //            //                }else
        //            if (minLADARfrontright < 600){
        //                RobotState = 2;
        //            }else{
        //                RobotState = 3;
        //            }
        //            break;
        //        case 3: //no avoidance needed
        //            if (minLADARfrontright < 600) {
        //                RobotState = 2;
        //            }else if (minLADARfrontleft < 600) {
        //                RobotState = 1;
        //            }
        //            break;
        //            //            case 4: //Tunnel Vision
        //            //                turn = Kp_obs*FrontRight_error;
        //            //                vref = 1; //Kp_vref_obs*FrontLeft_error;
        //            //                if ((350 < minLADARfrontright) || (350 < minLADARfrontleft)){
        //            //                    RobotState = 3;
        //            //                }
        //            //                break;
        //        }//code worked before tunnel case
        //    }


        if (vref<0.5){
            vref = 0.5;
        }

        if (newnavdata) {
            DSP_Float2 = new_DSP_Float2;
            DSP_Float3 = new_DSP_Float3;
            DSP_Float4 = new_DSP_Float4;
            DSP_Float5 = new_DSP_Float5;
            DSP_Float6 = new_DSP_Float6;
            DSP_Float7 = new_DSP_Float7;
            DSP_Float8 = new_DSP_Float8;
            DSP_Float9 = new_DSP_Float9;
            DSP_Float10 = new_DSP_Float10;
            DSP_Float11 = new_DSP_Float11;
            DSP_Float12 = new_DSP_Float12;
            DSP_Float13 = new_DSP_Float13;
            DSP_Float14 = new_DSP_Float14;
            DSP_Float15 = new_DSP_Float15;
            DSP_Float16 = new_DSP_Float16;
            DSP_Float17 = new_DSP_Float17;
            DSP_Float18 = new_DSP_Float18;
            DSP_Float19 = new_DSP_Float19;
            newnavdata = 0;
        }

        if(astarRunning == 1){
            vref = 0;
            turn = 0;
        }

        SetRobotOutputs(vref,turn,0,0,0,0,0,0,0,0);

        timecount++;
    }
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


