/*
 * Copyright (c) 2014, The Robot Studio
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice, this
 *	  list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright notice,
 *	  this list of conditions and the following disclaimer in the documentation
 *	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 02, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <std_msgs/Int32MultiArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <string>
#include "robotDefines.h"

/*** Defines ***/
#define TIMEOUT 			5 //5
#define REPLY_SIZE 			64
#define LOOP_RATE 			50 //20 //50

/*** Variables ***/
char data[REPLY_SIZE];

/*** Main ***/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trs_arduinoPixy_node");
    ros::NodeHandle n;
    cereal::CerealPort device;
    int32_t loopNb = 0;
	int32_t position = 0;
	bool inc = true;

	ros::Rate r(LOOP_RATE);

	ros::Publisher pixy_pub = n.advertise<std_msgs::Int32MultiArray>("/pixy", 1);

    // Change the next line according to your port name and baud rate
    //try{ device.open("/dev/ttyACM0", 115200); } //115200  //921600 //460800
	try{ device.open("/dev/ttyACM1", 115200); } //115200  //921600 //460800
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port /dev/ttyACM1 at 115200 baud !!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened : /dev/ttyACM1 at 115200 baud");

    //init array
    for(int i=0; i<REPLY_SIZE; i++)
    {
    	data[0] = 0;
    }

    //Flush the device
    device.flush();

    char delims[] = "#";
    char *result = NULL;

    int sig_i[2] = {0};
    int x_i[2] = {0};
    int y_i[2] = {0};
    int width_i[2] = {0};
    int height_i[2] = {0};

    while(ros::ok())
    {
        //read data on serial Rx
        // Get the reply, the last value is the timeout in ms
        try{ device.readBytes(data, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
        }

		//ROS_INFO("DATA : %s", data);

		char* sig1_c = NULL;
		char* x1_c = NULL;
		char* y1_c = NULL;
		char* width1_c = NULL;
		char* height1_c = NULL;

		char* sig2_c = NULL;
		char* x2_c = NULL;
		char* y2_c = NULL;
		char* width2_c = NULL;
		char* height2_c = NULL;

		result = strtok(data, "<");
		result = strtok(result, "#");
		sig1_c = result;
		result = strtok(NULL, delims);
		x1_c = result;
		result = strtok(NULL, delims);
		y1_c = result;
		result = strtok(NULL, delims);
		width1_c = result;
		result = strtok(NULL, delims);
		height1_c = result;

		result = strtok(NULL, delims);
		sig2_c = result;
		result = strtok(NULL, delims);
		x2_c = result;
		result = strtok(NULL, delims);
		y2_c = result;
		result = strtok(NULL, delims);
		width2_c = result;
		result = strtok(NULL, delims);
		height2_c = result;

		std_msgs::Int32MultiArray pixy;
		pixy.data.clear();

		//continue only if all pointer are not null
		if((sig1_c != NULL) && (x1_c != NULL) && (y1_c != NULL) && (width1_c != NULL) && (height1_c != NULL)
		&& (sig2_c != NULL) && (x2_c != NULL) && (y2_c != NULL) && (width2_c != NULL) && (height2_c != NULL))
		{
			//fill with data
			pixy.data.push_back(atoi(sig1_c));
			pixy.data.push_back(atoi(x1_c));
			pixy.data.push_back(atoi(y1_c));
			pixy.data.push_back(atoi(width1_c));
			pixy.data.push_back(atoi(height1_c));

			//WARNING send twice the same, so both arm grab the same
			pixy.data.push_back(atoi(sig1_c));
			pixy.data.push_back(atoi(x1_c));
			pixy.data.push_back(atoi(y1_c));
			pixy.data.push_back(atoi(width1_c));
			pixy.data.push_back(atoi(height1_c));
		}
		else
		{
			//fill with 0
			pixy.data.push_back(0);
			pixy.data.push_back(0);
			pixy.data.push_back(0);
			pixy.data.push_back(0);
			pixy.data.push_back(0);

			//fill with 0
			pixy.data.push_back(0);
			pixy.data.push_back(0);
			pixy.data.push_back(0);
			pixy.data.push_back(0);
			pixy.data.push_back(0);
		}

		//publish Pixy
		pixy_pub.publish(pixy);

		//flush the RS232 FIFO
        device.flush();

        r.sleep();
    }
}



