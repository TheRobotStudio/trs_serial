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
#include "trs_msgs/MotorCmdMultiArray.h"
#include "trs_msgs/MotorDataMultiArray.h"
#include <sstream>
#include <string>
#include "robotDefines.h"

/*** Defines ***/
#define LOOP_RATE				HEART_BEAT
#define TIMEOUT 				66//35//25 //20 // 1/(2*HEART_BEAT)
#define BYTES_PER_MSG			6 //1 nodeID 1 mode 4 data
#define BYTES_PER_DATA			8
#define REPLY_SIZE 				NUMBER_OF_MOTORS*8+1 //+1 for the checksum
#define RS232_BAUDRATE			460800

/*** Variables ***/
char data[NUMBER_SLAVE_BOARDS][NUMBER_MAX_EPOS2_PER_SLAVE][BYTES_PER_MSG];
bool msgReceived = false;

/*** Functions ***/
char calculateChecksum(char* dataArray, int nbBytes)
{
	char checksum = 0x00;
	int sum = 0;

	for(int i=0; i<nbBytes; i++)
	{
		sum += dataArray[i];
		//ROS_INFO("0x%02X", dataArray[i]);
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
	else
	{
		ROS_INFO("checksum 0x%02X", checksum);
		return false;
	}
}

/*** Callback functions ***/
void sendMotorCmdMultiArray_cb(const trs_msgs::MotorCmdMultiArrayConstPtr& motorCmd_ma)
{
	if((motorCmd_ma->layout.dim[0].size == NUMBER_SLAVE_BOARDS) && (motorCmd_ma->layout.dim[1].size == NUMBER_MAX_EPOS2_PER_SLAVE))
	{
		//toggle flag, a message has been received
		msgReceived = true;

		#ifdef TRS_DEBUG
		ROS_INFO("msgReceived");
		#endif

		//update data with the new msg cmds
		for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
		{
			for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
			{
				data[i][j][0] = (char)(motorCmd_ma->motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].nodeID);// + i*NUMBER_MAX_EPOS2_PER_SLAVE); //nodeID
				data[i][j][1] = (char)(motorCmd_ma->motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].mode); //mode
				data[i][j][2] = (char)(motorCmd_ma->motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value); //data 4 bytes
				data[i][j][3] = (char)(motorCmd_ma->motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value >> 8);
				data[i][j][4] = (char)(motorCmd_ma->motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value >> 16);
				data[i][j][5] = (char)(motorCmd_ma->motorCmd[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].value >> 24);
			}
		}
	}
}

/*** Main ***/
int main(int argc, char** argv)
{
	// Initialize ROS
    ros::init(argc, argv, "trs_masterMbed_node");
    ros::NodeHandle nh("~");

    ros::Rate r(LOOP_RATE);

    // Parameters
    std::string usb_device_name;
    // Grab the parameters
    nh.param("usb_device", usb_device_name, std::string("/dev/ttyUSB0"));

    cereal::CerealPort device;
    char reply[REPLY_SIZE] = {0x00};
    char checksum[1];

    bool dataValid = false;

	ros::Subscriber cmd_sub = nh.subscribe("/motor_cmd_array", 1, sendMotorCmdMultiArray_cb);
	ros::Publisher data_pub = nh.advertise<trs_msgs::MotorDataMultiArray>("/motor_data_array", 1);
	trs_msgs::MotorDataMultiArray motorData_ma;

	//create the data multi array
	motorData_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorData_ma.layout.dim[0].size = NUMBER_SLAVE_BOARDS;
	motorData_ma.layout.dim[0].stride = NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE;
	motorData_ma.layout.dim[0].label = "slaves";
	motorData_ma.layout.dim.push_back(std_msgs::MultiArrayDimension());
	motorData_ma.layout.dim[1].size = NUMBER_MAX_EPOS2_PER_SLAVE;
	motorData_ma.layout.dim[1].stride = NUMBER_MAX_EPOS2_PER_SLAVE;
	motorData_ma.layout.dim[1].label = "motors";
	motorData_ma.layout.data_offset = 0;
	motorData_ma.motorData.clear();
	motorData_ma.motorData.resize(NUMBER_SLAVE_BOARDS*NUMBER_MAX_EPOS2_PER_SLAVE);

	//init data array
	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
	{
		for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
		{
			motorData_ma.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].encPosition = 0;
			motorData_ma.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].current = 0;
			motorData_ma.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].force = 0;

			//init RS232 char data
			data[i][j][0] = i*NUMBER_MAX_EPOS2_PER_SLAVE + j + 1;
			data[i][j][1] = 0xFF; //0x02;
			data[i][j][2] = 0x00; //0x10 + i;
			data[i][j][3] = 0x00; //0x20 + i;
			data[i][j][4] = 0x00; //0x30 + i;
			data[i][j][5] = 0x00; //0x40 + i;
		}
	}

    // Change the next line according to your port name and baud rate
	try{ device.open(usb_device_name.c_str(), RS232_BAUDRATE); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port %s at 460800 baud !!!", usb_device_name.c_str());
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened : %s at 460800 baud", usb_device_name.c_str());

    //Main loop
    while(ros::ok())
    {
    	msgReceived = false;

    	//check for a new message incoming
    	ros::spinOnce(); //this will toggle the flag msgReceived if a msg has been published by another node

    	//if no msg received
    	if(!msgReceived)
    	{
    		//reset data mode with 0xFF, i.e. idle slave mode
    		for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
			{
				for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
				{
					data[i][j][0] = 0x00;//optional

					data[i][j][1] = 0xFF;

					//optional
					data[i][j][2] = 0x00;
					data[i][j][3] = 0x00;
					data[i][j][4] = 0x00;
					data[i][j][5] = 0x00;
				}
			}

    		//ROS_INFO("NO msg received!");
    	}
    /*	else
    	{
    		ROS_INFO("YES msg received!");
    	}*/


    	//write the new data on serial Tx (update in the subscriber or 0xFF data above)
    	checksum[0] = calculateChecksum(data[0][0], NUMBER_OF_MOTORS*BYTES_PER_MSG);

		device.write(data[0][0], NUMBER_OF_MOTORS*BYTES_PER_MSG); //send pointer to the first element, and size
		device.write(">>>>>", 5); //TODO send checksum //62 or 0x3E
		device.write(checksum, 1);

        //read data on serial Rx
        // Get the reply, the last value is the timeout in ms
        try{ device.readBytes(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR("Timeout!");
        }

        //check if the data are valid
        dataValid = verifyChecksum(reply, REPLY_SIZE);

        if(dataValid)
        {
        	//ROS_INFO("Data valid");

        	//build the custom message
        	for(int i=0; i<NUMBER_SLAVE_BOARDS; i++)
			{
				for(int j=0; j<NUMBER_MAX_EPOS2_PER_SLAVE; j++)
				{
					motorData_ma.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].encPosition =
														  (uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 0]
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 1]) << 8)
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 2]) << 16)
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 3]) << 24);

					motorData_ma.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].current =
														  (uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 4]
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 5]) << 8);

					motorData_ma.motorData[i*NUMBER_MAX_EPOS2_PER_SLAVE + j].force =
														  (uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 6]
													  + (((uint8_t)reply[(i*NUMBER_MAX_EPOS2_PER_SLAVE + j)*BYTES_PER_DATA + 7]) << 8);
				}
			}

        	//publish it
        	data_pub.publish(motorData_ma);
        	//ROS_INFO("Valid");
        	dataValid = false; //reset flag
        }
        else
        {
        	//ROS_INFO("Data NOT valid");
        }

        device.flush();

        r.sleep();
    }
}
