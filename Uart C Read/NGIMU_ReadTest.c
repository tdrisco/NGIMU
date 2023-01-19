#include "NgimuReceive.h"

#include <wiringPi.h>
#include <wiringSerial.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>

float result_ngimu,mkr,res_final;
int imu_res[5];
unsigned long me,my,ms,ma;
float a=0,okok;
char bb[128],ch;
int res,c=0,len=0,lenmkr,decimal_point=0,enc_1,enc_2;
double q0,q1,q2,q3,r00,r01,r02,r10,r11,r12,r20,r21,r22;
char send_str[10],app[10];
int first_time = 1;
float Vn[3],X0[3],Yt[3],Ytp[3],vt1[3],vt2[3],angle3D;
float intial_qua[4];
int serial_port;
clock_t start, end;
double exeTime;

int speed_Hz = 0;

void ngimuReceiveErrorCallback(const char* const errorMessage);
void ngimuSensorsCallback(const NgimuSensors ngimuSensors);
//void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion);
void ngimuEulerCallback(const NgimuEuler ngimuEuler);

int main() {

    /*Setup*/
    
    if ((serial_port = serialOpen("/dev/ttyACM0", 115200)) < 0) {	/* open serial port: uart4 (ttyAMA1)*/ 
		fprintf(stderr, "Unable to open serial device 1: %s\n", strerror(errno)) ;
		return 1 ;
	}

    
    NgimuReceiveInitialise();
    // Assign NGIMU receive callback functions
    NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
    NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
    //NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
    NgimuReceiveSetEulerCallback(ngimuEulerCallback);

    /*Loop*/
    int count=0;
    start = clock();
    while(1) {
        
        // Process each received byte
        while (serialDataAvail(serial_port)) 
        {
            NgimuReceiveProcessSerialByte(serialGetchar(serial_port));
        }
        
        
        end = clock();
        exeTime = (double)(end-start)/CLOCKS_PER_SEC;
        count = count+1;
        if(exeTime>=1){
            break;
        }
        
    }
    
    printf("%d\n",count);
    
    return 1;

}

// This function is called each time there is a receive error
void ngimuReceiveErrorCallback(const char* const errorMessage) {
    printf(errorMessage);
    printf("\r\n");
}

// This function is called each time a "/sensors" message is received
void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {
    printf("/sensors, ");
    printf("%f",ngimuSensors.gyroscopeX);
    printf(", ");
    printf("%f",ngimuSensors.gyroscopeY);
    printf(", ");
    printf("%f",ngimuSensors.gyroscopeZ);
    printf(", ");
    printf("%f",ngimuSensors.accelerometerX);
    printf(", ");
    printf("%f",ngimuSensors.accelerometerY);
    printf(", ");
    printf("%f",ngimuSensors.accelerometerZ);
    printf(", ");
    printf("%f",ngimuSensors.magnetometerX);
    printf(", ");
    printf("%f",ngimuSensors.magnetometerY);
    printf(", ");
    printf("%f",ngimuSensors.magnetometerZ);
    printf("%f",ngimuSensors.barometer);
    printf("\r\n");
}

// This function is called each time a "/euler" message is received.
void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
    printf("%0.4f ",exeTime);
    printf("%d:Euler (R,P,Y), ",speed_Hz);
    printf("%f",ngimuEuler.roll);
    printf(", ");
    printf("%f",ngimuEuler.pitch);
    printf(", ");
    printf("%f",ngimuEuler.yaw);
    printf("\r\n");
    speed_Hz = speed_Hz + 1;
}
