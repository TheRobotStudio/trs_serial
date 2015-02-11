/*
 * Copyright (c) 2013, The Robot Studio
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
 *  Created on: Feb 28, 2013
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include "trs_msgs/MotorCmdSet.h"
#include "trs_msgs/MotorDataSet.h"
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT
#define TIMEOUT 				20 //1000
#define BYTES_PER_MSG			6 //1 nodeID 1 mode 4 data
#define BYTES_PER_DATA			8
#define REPLY_SIZE 				NUMBER_OF_MOTORS*8+1

/*** Variables ***/
char data[NUMBER_OF_MOTORS][BYTES_PER_MSG];
bool msgReceived = false;

/*** Functions ***/
char calculateChecksum(char* data, int nbBytes)
{
	char checksum = 0x00;
	int sum = 0;

	for(int i=0; i<nbBytes; i++)
	{
		sum += data[i];
	}
	//ROS_INFO("sum 0x%02X", sum);

	checksum = (char)(~sum);
	//ROS_INFO("checksum 0x%02X", checksum);

	return checksum;
}

char verifyChecksum(char* data, int nbBytes)
{
	char checksum = 0x00;

	for(int i=0; i<nbBytes; i++)
	{
		checksum += data[i];
	}
	//ROS_INFO("sum 0x%02X", sum);

	checksum++;
	//checksum = (uint8_t)(checksum);
	//ROS_INFO("checksum 0x%02X", checksum);

	if(checksum == 0x00) return true;
	else return false;
}

/*** Callback functions ***/
void sendCmdSet_cb(const trs_msgs::MotorCmdSetConstPtr& motCmdSet)
{
	//toggle flag, a message has been received
	msgReceived = true;

	#ifdef TRS_DEBUG
	ROS_INFO("msgReceived");
	#endif

	//update data with the new msg cmds
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		data[i][0] = (char)(motCmdSet->motorCmd[i].nodeID); //nodeID
		data[i][1] = (char)(motCmdSet->motorCmd[i].mode); //mode
		data[i][2] = (char)(motCmdSet->motorCmd[i].value); //data 4 bytes
		data[i][3] = (char)(motCmdSet->motorCmd[i].value >> 8);
		data[i][4] = (char)(motCmdSet->motorCmd[i].value >> 16);
		data[i][5] = (char)(motCmdSet->motorCmd[i].value >> 24);
	
		//debug code to trace an error, it was an EPOS2 board damaged on the CAN connexion side probably
		//if(data[i][0] == 2) data[i][0] = (char)6; //(char)(motCmdSet->motorCmd[i].nodeID); //nodeID
		//if(data[i][0] == 2) ROS_INFO("loop nb %d : Cmd node ID 2 : %d", i, motCmdSet->motorCmd[i].value);
	}
}

/*** Main ***/
int main(int argc, char** argv)
{
	// Initialize ROS
    ros::init(argc, argv, "trs_masterMbed_node");
    ros::NodeHandle n;

    cereal::CerealPort device;
    char reply[REPLY_SIZE] = {0x00};
    char checksum[1];

    bool dataValid = false;

    int32_t loopNb = 0;
	int32_t position = 0;
	bool inc = true;

	ros::Rate r(LOOP_RATE); //30//40

	ros::Subscriber cmd_sub = n.subscribe("/sendCmdSet", 1, sendCmdSet_cb);
	ros::Publisher data_pub = n.advertise<trs_msgs::MotorDataSet>("/motorDataSet", 1);
	trs_msgs::MotorDataSet dataSet;

    // Change the next line according to your port name and baud rate
    try{ device.open("/dev/ttyUSB0", 460800); } //115200  //921600 //460800
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port /dev/ttyUSB0 at 460800 baud !!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened : /dev/ttyUSB0 at 460800 baud");

    //init array
    for(int i=0; i<NUMBER_OF_MOTORS; i++)
    {
    	data[i][0] = i+1;
    	data[i][1] = 0x02;
    	data[i][2] = 0x10 + i;
    	data[i][3] = 0x20 + i;
    	data[i][4] = 0x30 + i;
    	data[i][5] = 0x40 + i;
    }

    while(ros::ok())
    {
    	msgReceived = false;

    	//check for a new message incoming
    	ros::spinOnce(); //this will toggle the flag msgReceived if a msg has been published by another node

    	//if no msg received
    	if(!msgReceived)
    	{
    		//reset data mode with 0xFF, i.e. idle slave mode
    		for(int i=0; i<NUMBER_OF_MOTORS; i++)
			{
				data[i][0] = 0x00; //nodeID
				data[i][1] = 0xFF; //mode
				data[i][2] = 0x00; //data 4 bytes
				data[i][3] = 0x00;
				data[i][4] = 0x00;
				data[i][5] = 0x00;
			}
    	}

    	//write the new data on serial Tx (update in the subscriber or 0xFF data above
    	checksum[0] = calculateChecksum(data[0], NUMBER_OF_MOTORS*BYTES_PER_MSG);

		device.write(data[0], NUMBER_OF_MOTORS*BYTES_PER_MSG); //send pointer to the first element, and size
		device.write(">>>>>", 5); //TOSO send checksum //62 or 0x3E
		device.write(checksum, 1);

        //read data on serial Rx
        // Get the reply, the last value is the timeout in ms
        try{ device.readBytes(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }

        for(int i=0; i<1; i++)
        {
        	//ROS_INFO("Reply: %02X %02X %02X %02X %02X %02X %02X %02X", (uint8_t)reply[i*8 + 0], (uint8_t)reply[i*8 + 1], (uint8_t)reply[i*8 + 2], (uint8_t)reply[i*8 + 3], (uint8_t)reply[i*8 + 4], (uint8_t)reply[i*8 + 5], (uint8_t)reply[i*8 + 6], (uint8_t)reply[i*8 + 7]);
        }
        //ROS_INFO("cs: %02X", (uint8_t)reply[REPLY_SIZE-1]);

        //check if the data are valid
        dataValid = verifyChecksum(reply, REPLY_SIZE);

        if(dataValid)
        {
        	//build the custom message
        	for(int i=0; i<NUMBER_OF_MOTORS; i++)
        	{
            	dataSet.motorData[i].encPosition =  (uint8_t)reply[i*BYTES_PER_DATA + 0]
												  + (((uint8_t)reply[i*BYTES_PER_DATA + 1]) << 8)
												  + (((uint8_t)reply[i*BYTES_PER_DATA + 2]) << 16)
												  + (((uint8_t)reply[i*BYTES_PER_DATA + 3]) << 24);
            	dataSet.motorData[i].current =  (uint8_t)reply[i*BYTES_PER_DATA + 4]
											  + (((uint8_t)reply[i*BYTES_PER_DATA + 5]) << 8);
            	dataSet.motorData[i].force =  (uint8_t)reply[i*BYTES_PER_DATA + 6]
										    + (((uint8_t)reply[i*BYTES_PER_DATA + 7]) << 8);
        	}

        	//publish it
        	data_pub.publish(dataSet);
        	//ROS_INFO("Valid");
        	dataValid = false; //reset flag
        }

        device.flush();

        r.sleep();
    }
}


