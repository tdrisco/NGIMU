# Tim Driscoll
# November 30th 2022

#Python script to read serial data from arduino mega
#Read and process NGIMU Euler angles

import serial

import time

import datetime

import numpy as np

import math

import osc_decoder

#Define Globals

_PORT_ARDUINO = 'COM15'

_DATA_NUMBER = 100


class NGIMU:

    def __init__(self) -> None:
        
        #Intialize and object representing the IMU

        self.IMU = serial.Serial(_PORT_ARDUINO, 115200)

        time.sleep(2)

        # Read any garbage from the serial buffer and reset both input and output buffer

        for i in range(50):

            self.IMU.readline()
    
        self.IMU.reset_input_buffer()

        self.IMU.reset_output_buffer()

        self.YAW = []

        self.PITCH = []

        self.ROLL = []

    def Read_Euler(self, readMode = 1, printLive = 1) -> None:

        count = 1

        try:

            while 1:

                self.IMU.reset_input_buffer()

                self.IMU.reset_output_buffer()

                #time.sleep(2)

                Incoming_Data = self.IMU.readline()

                if(Incoming_Data):

                    for message in osc_decoder.decode(Incoming_Data):
                        print(message)

                time.sleep(0.0025)

                #Incoming_Data = Incoming_Data.split()


               # Time_Stamp = datetime.datetime.now()

               # if(len(Incoming_Data) == 3):

               #     self.ROLL.append(Incoming_Data[0].decode())

               #     self.PITCH.append(Incoming_Data[1].decode())

               #     self.YAW.append(Incoming_Data[2].decode())

               #     if(printLive):
               #         print("{} {}: Roll {}\tPitch {}\tYaw {}\n".format(count, Time_Stamp, self.ROLL[-1],self.PITCH[-1],self.YAW[-1]))
                    
                #    count += 1


        except KeyboardInterrupt:

            print("\nINFO: Ctrl + C was pressed, stopping data stream")

            self.IMU.close()
    
    def Print_Euler(self) -> None:

        print("INFO: Printing the euler angles previously collected")

        for i in range(len(self.YAW)):

            print("Roll: {}\tPitch: {}\tYaw: {}\n".format(self.ROLL[i],self.PITCH[i],self.YAW[i]))

def main():

    Serial_IMU = NGIMU()

    Serial_IMU.Read_Euler()


if __name__ == "__main__":

    main()