#include "NgimuReceive.h"
//#include "Osc99.h"

//#include "Osc99.h"
/*#include "OscAddress.h"

#include "OscBundle.h"
#include "OscCommon.h"
#include "OscError.h"
#include "OscMessage.h"
#include "OscPacket.h"
#include "OscSlip.h"*/

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

void ngimuReceiveErrorCallback(const char* const errorMessage);
void ngimuSensorsCallback(const NgimuSensors ngimuSensors);
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion);
void ngimuEulerCallback(const NgimuEuler ngimuEuler);

int main() {

    /*Setup*/
    
    if ((serial_port = serialOpen("/dev/ttyAMA3", 115200)) < 0) {	/* open serial port: uart4 */ 
		fprintf(stderr, "Unable to open serial device 1: %s\n", strerror(errno)) ;
		return 1 ;
	}

    
    NgimuReceiveInitialise();
    // Assign NGIMU receive callback functions
    NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
    NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
    NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
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
        printf("a");
        printf("%d.",imu_res[0]);
        printf("%d",imu_res[1]);
        printf("b\n");
        //delay(1);
        
        end = clock();
        exeTime = (double)(end-start)/CLOCKS_PER_SEC;
        count++;
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

// This function is called each time a "/quaternion" message is received
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {
//    Serial.print("/quaternion, ");
//    Serial.print(ngimuQuaternion.w);
//    Serial.print(", ");
//    Serial.print(ngimuQuaternion.x);
//    Serial.print(", ");
//    Serial.print(ngimuQuaternion.y);
//    Serial.print(", ");
//    Serial.print(ngimuQuaternion.z);
//    Serial.print("\r\n");
    q0=ngimuQuaternion.w;
    q1=ngimuQuaternion.x;
    q2=ngimuQuaternion.y;
    q3=ngimuQuaternion.z;

    if(first_time<20&&first_time>0)
    {
      first_time = first_time + 1;
      }
      if(first_time>=20)
      {
        intial_qua[0] = q0;
        intial_qua[1] = q1;
        intial_qua[2] = q2;
        intial_qua[3] = q3;
        
//first step calculate Vn and Y0
//        Vn[0]= 2*(q1*q3+q0*q2);
//        Vn[1]= 2*(q2*q3-q0*q1);
//        Vn[2]= 2*(q0*q0+q3*q3)-1;
        Vn[0]= 2*(q1*q3-q0*q2);
        Vn[1]= 2*(q2*q3+q0*q1);
        Vn[2]= 1-2*q1*q1-2*q2*q2;
        

        X0[0]= 1-2*q2*q2-2*q3*q3;
        X0[1]= 2*(q1*q2-q0*q3);
        X0[2]= 2*(q1*q3+q0*q2);

        first_time = -1;
        
        
        }
//********************************************************initial position calculation**************************************************//
    if(first_time==-1)
    {
      // second step Yt and Ytp
      
        Yt[0]= 2*(q1*q2+q0*q3);
        Yt[1]= 1-2*(q1*q1+q3*q3);
        Yt[2]= 2*(q2*q3-q0*q1);

        vt1[0]=Yt[0]*Vn[0]+Yt[1]*Vn[1]+Yt[2]*Vn[2];

        vt2[0]=vt1[0]*Vn[0];
        vt2[1]=vt1[0]*Vn[1];
        vt2[2]=vt1[0]*Vn[2];    
        
        Ytp[0]=Yt[0]-vt2[0];
        Ytp[1]=Yt[1]-vt2[1];
        Ytp[2]=Yt[2]-vt2[2];

        angle3D = acos((Ytp[0]*X0[0]+Ytp[1]*X0[1]+Ytp[2]*X0[2])/(sqrt(Ytp[0]*Ytp[0]+Ytp[1]*Ytp[1]+Ytp[2]*Ytp[2])*sqrt(X0[0]*X0[0]+X0[1]*X0[1]+X0[2]*X0[2])));
        angle3D = angle3D* 180/3.14159265;
      }
//  Serial.println(angle3D);
//    
//    r00 =  1- 2*(q2 * q2 + q3 * q3);
//     
//    r10 = 2 * (q1 * q2 + q0 * q3);
//
//    result_ngimu=  (atan2(r10,r00)* 180/3.14159265 );
    if(angle3D>0)
    {
      res=(angle3D)*100;
    }
    else
    {
      res=(360+angle3D)*100;
    }
        res_final=((float)(res)/100);
        imu_res[0]=(res/256);
        imu_res[1]=(res%256);
}

// This function is called each time a "/euler" message is received.
void ngimuEulerCallback(const NgimuEuler ngimuEuler) {
    printf("/euler, ");
    printf("%f",ngimuEuler.roll);
    printf(", ");
    printf("%f",ngimuEuler.pitch);
    printf(", ");
    printf("%f",ngimuEuler.yaw);
    printf("\r\n");
}
