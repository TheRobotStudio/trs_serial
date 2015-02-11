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
 *  Created on: Jan 28, 2014
 *      Author: Cyril Jourdan (cyril.jourdan@therobotstudio.com)
 */

/*** Includes ***/
#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Geometry>
#include <trs_msgs/MotorDataSet.h>
#include "trs_msgs/MotorCmdSet.h"
#include <string>
#include "robotDefines.h"

/*** Defines ***/
#define TIMEOUT 			5 //5
#define REPLY_SIZE 			150 //50
#define LOOP_RATE 			50 //50
#define STAFF_LENGHT			1.38
#define BALANCE_FACTOR			2			
#define BALANCE_DEADBAND		5	
#define P_FACTOR			4

/*** Variables ***/
char data[REPLY_SIZE];

//std::string tfOri_str = "/origin";
ros::Publisher legs_cmd_pub;
trs_msgs::MotorCmdSet cmd;
trs_msgs::MotorDataSet posture;

//to wait for the msg from both posture and joint_states_kinect topics
//bool postureArrived = false;

/*** Functions ***/
void initCmdSet()
{
	for(int i=0; i<NUMBER_OF_MOTORS; i++)
	{
		cmd.motorCmd[i].nodeID = i+1;
		cmd.motorCmd[i].mode = NO_MODE;
		cmd.motorCmd[i].value = 0;
	}
}

/*
void posture_cb(const trs_msgs::MotorDataSetConstPtr& data)
{
	posture = *data;
	postureArrived = true;
}
*/

/*** Main ***/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "trs_arduino_node");
    ros::NodeHandle n;

    cereal::CerealPort device;

    int32_t loopNb = 0;
	int32_t position = 0;
	bool inc = true;
	//postureArrived = false;

	ros::Rate r(LOOP_RATE);

	//ros::Subscriber postureSub = n.subscribe ("/motorDataSet", 10, posture_cb);
	legs_cmd_pub = n.advertise<trs_msgs::MotorCmdSet>("/setLegsCommands", 100, true);
	ros::Publisher markerIMU_pub = n.advertise<visualization_msgs::Marker>("/imu_marker", 1);
	ros::Publisher markerAcc_pub = n.advertise<visualization_msgs::Marker>("/acc_marker", 1);
	ros::Publisher markerFall_pub = n.advertise<visualization_msgs::Marker>("/fall_marker", 1);
	//ros::Publisher markerFeet_pub = n.advertise<visualization_msgs::Marker>("/Feet_marker", 1);

	ros::Publisher adc_pub = n.advertise<std_msgs::Int32MultiArray>("/adc", 1);

	static tf::TransformBroadcaster br_origin;
	tf::Transform origin_tf;
	origin_tf.setOrigin( tf::Vector3(0, 0, 0) );
	origin_tf.setRotation( tf::Quaternion(0, 0, 0, 1));
	br_origin.sendTransform(tf::StampedTransform(origin_tf, ros::Time::now(),  "/ground", "/origin"));

	static tf::TransformBroadcaster br_IMU;
	tf::Transform imu_tf;

	visualization_msgs::Marker cubeIMU;
	uint32_t shape = visualization_msgs::Marker::CUBE;
	cubeIMU.header.frame_id = "/origin";
	cubeIMU.ns = "basic_shapes";
	cubeIMU.id = 0;
	cubeIMU.type = shape;
	cubeIMU.action = visualization_msgs::Marker::ADD;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	cubeIMU.scale.x = 0.05;
	cubeIMU.scale.y = 0.1;
	cubeIMU.scale.z = 0.01;
	// Set the color -- be sure to set alpha to something non-zero!
	cubeIMU.color.r = 0.0f;
	cubeIMU.color.g = 1.0f;
	cubeIMU.color.b = 0.0f;
	cubeIMU.color.a = 1.0;

    // Change the next line according to your port name and baud rate
    try{ device.open("/dev/ttyACM0", 115200); } //115200  //921600 //460800
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port /dev/ttyACM0 at 115200 baud !!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened : /dev/ttyACM0 at 115200 baud");

    //init array
    for(int i=0; i<REPLY_SIZE; i++)
    {
    	data[0] = 0;
    }

    //Flush the device
    device.flush();

    char delims[] = "#";
    char *result = NULL;

    double w_f = 0;
    double x_f = 0;
    double y_f = 0;
    double z_f = 0;

    double accX_f = 0;
	double accY_f = 0;
	double accZ_f = 0;

	double prev_accX_f = 0;
	double prev_accY_f = 0;
	double prev_accZ_f = 0;

	visualization_msgs::Marker acc_marker;
	acc_marker.header.frame_id = "/imu";
	acc_marker.ns = "marker_test_arrow_by_points";
	acc_marker.id = 2;
	acc_marker.type = visualization_msgs::Marker::ARROW;
	acc_marker.action = visualization_msgs::Marker::ADD;
	acc_marker.pose.orientation.x = 0.0;
	acc_marker.pose.orientation.y = 0.0;
	acc_marker.pose.orientation.z = 0.0;
	acc_marker.pose.orientation.w = 1.0;
	acc_marker.pose.position.x = 0;
	acc_marker.pose.position.y = 0;
	acc_marker.scale.x = 0.005;
	acc_marker.scale.y = 0.01;
	acc_marker.color.r = 1.0;
	acc_marker.color.g = 0.0;
	acc_marker.color.b = 0.0;
	acc_marker.color.a = 1.0;

	acc_marker.points.resize(2);
	acc_marker.points[0].x = 0.0f;
	acc_marker.points[0].y = 0.0f;
	acc_marker.points[0].z = 0.0f;

	visualization_msgs::Marker fall_marker;
	fall_marker.header.frame_id = "/origin";
	fall_marker.ns = "marker_test_arrow_by_points";
	fall_marker.id = 3;
	fall_marker.type = visualization_msgs::Marker::ARROW;
	fall_marker.action = visualization_msgs::Marker::ADD;
	fall_marker.pose.orientation.x = 0.0;
	fall_marker.pose.orientation.y = 0.0;
	fall_marker.pose.orientation.z = 0.0;
	fall_marker.pose.orientation.w = 1.0;
	fall_marker.pose.position.x = 0;
	fall_marker.pose.position.y = 0;
	fall_marker.scale.x = 0.005;
	fall_marker.scale.y = 0.01;
	fall_marker.color.r = 0.0;
	fall_marker.color.g = 1.0;
	fall_marker.color.b = 0.0;
	fall_marker.color.a = 1.0;

	fall_marker.points.resize(2);
	fall_marker.points[0].x = 0.0f;
	fall_marker.points[0].y = 0.0f;
	fall_marker.points[0].z = 0.0f;
	fall_marker.points[1].z = 0.0f;
/*
	visualization_msgs::Marker feet_marker;
	feet_marker.header.frame_id = "/origin";
	feet_marker.ns = "marker_feet";
	feet_marker.id = 7;
	feet_marker.type = visualization_msgs::Marker::SPHERE;
	feet_marker.action = visualization_msgs::Marker::ADD;
	feet_marker.pose.orientation.x = 0.0;
	feet_marker.pose.orientation.y = 0.0;
	feet_marker.pose.orientation.z = 0.0;
	feet_marker.pose.orientation.w = 1.0;
	feet_marker.pose.position.x = 0.1;
	feet_marker.pose.position.y = 0.1;
	feet_marker.scale.x = 0.01;
	feet_marker.scale.y = 0.01;
	feet_marker.color.r = 1.0;
	feet_marker.color.g = 0.0;
	feet_marker.color.b = 0.0;
	feet_marker.color.a = 1.0;
*/
	//printf("The calibration will start ! Press Enter when ready !\n");
	//std::cin.get();

//	printf("The main balance loop will start ! Press Enter when ready !\n");
//	std::cin.get();
/*
	bool calibDone = false;
	int counterCalib = 0;
	std_msgs::Int32MultiArray avgAdcCalib;
	avgAdcCalib.data.clear();
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);
	avgAdcCalib.data.push_back(0);

	std_msgs::Int32MultiArray offsetAdc;
	offsetAdc.data.clear();
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);
	offsetAdc.data.push_back(0);

	initCmdSet();
*/

    while(ros::ok())
    {
    	br_origin.sendTransform(tf::StampedTransform(origin_tf, ros::Time::now(),  "/ground", "/origin"));

        //read data on serial Rx
        // Get the reply, the last value is the timeout in ms
        try{ device.readBytes(data, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            //ROS_ERROR("Timeout!");
        }

	//ROS_INFO("DATA : %s", data);
	
        char* w_c = NULL;
	char* x_c = NULL;
	char* y_c = NULL;
	char* z_c = NULL;

	char* accX_c = NULL;
	char* accY_c = NULL;
	char* accZ_c = NULL;

	char* adc0 = NULL;
	char* adc1 = NULL;
	char* adc2 = NULL;
	char* adc3 = NULL;
	char* adc4 = NULL;
	char* adc5 = NULL;

	result = strtok(data, "<");
        result = strtok(NULL, delims);
        w_c = result;
	result = strtok(NULL, delims);
        x_c = result;
        result = strtok(NULL, delims);
        y_c = result;
        result = strtok(NULL, delims);
        z_c = result;
        result = strtok(NULL, delims);
        accX_c = result;
	result = strtok(NULL, delims);
	accY_c = result;
	result = strtok(NULL, delims);
	accZ_c = result;

	result = strtok(NULL, delims);
	adc0 = result;
	result = strtok(NULL, delims);
	adc1 = result;
	result = strtok(NULL, delims);
	adc2 = result;
	result = strtok(NULL, delims);
	adc3 = result;
	result = strtok(NULL, delims);
	adc4 = result;
	result = strtok(NULL, delims);
	adc5 = result;

	//continue only if all pointer are not null
	if((w_c != NULL) && (x_c != NULL) && (y_c != NULL) && (z_c != NULL) && (accX_c != NULL) && (accY_c != NULL) && (accZ_c != NULL) && (adc0 != NULL) && (adc1 != NULL) && (adc2 != NULL) && (adc3 != NULL) && (adc4 != NULL) && (adc5 != NULL))
	{
		//ROS_INFO("if ok");
		//modify the IMU marker
		//cubeIMU.header.stamp = ros::Time::now();
/*
		cubeIMU.pose.orientation.x = atof(x_c);
		cubeIMU.pose.orientation.y = atof(y_c);
		cubeIMU.pose.orientation.z = atof(z_c);
		cubeIMU.pose.orientation.w = atof(w_c);

		//Arrow for acceleration
		acc_marker.header.stamp = ros::Time();

		//use orientation as a temp var
		acc_marker.pose.orientation.x = atof(accX_c);
		acc_marker.pose.orientation.y = atof(accY_c);
		acc_marker.pose.orientation.z = atof(accZ_c);

		accX_f = acc_marker.pose.orientation.x;
		accY_f = acc_marker.pose.orientation.y;
		accZ_f = acc_marker.pose.orientation.z;

		//then rewrite 0
		acc_marker.pose.orientation.x = 0.0;
		acc_marker.pose.orientation.y = 0.0;
		acc_marker.pose.orientation.z = 0.0;

		//rescale to vizualise
		accX_f /= 100;
		accY_f /= 100;
		accZ_f /= 100;


		acc_marker.points[1].x = accX_f;
		acc_marker.points[1].y = accY_f;
		acc_marker.points[1].z = accZ_f;

		markerAcc_pub.publish(acc_marker);

		Eigen::Quaternion<float> q(cubeIMU.pose.orientation.w, cubeIMU.pose.orientation.x, cubeIMU.pose.orientation.y, cubeIMU.pose.orientation.z);
		Eigen::Matrix3f m;
		m = q.toRotationMatrix();
		Eigen::Vector3f z0(0,0,1);
		Eigen::Vector3f z1;

		//float d = STAFF_LENGHT;
		z1 = m*z0;
		z1.normalize();
		z1 *= STAFF_LENGHT;
*/

			/*
					cubeIMU.pose.position.x = z1[0];
					cubeIMU.pose.position.y = z1[1];
					cubeIMU.pose.position.z = z1[2];
					markerIMU_pub.publish(cubeIMU);
			*/

/*
		imu_tf.setOrigin( tf::Vector3(z1[0], z1[1], z1[2]));

		imu_tf.setRotation( tf::Quaternion(cubeIMU.pose.orientation.x, cubeIMU.pose.orientation.y, cubeIMU.pose.orientation.z, cubeIMU.pose.orientation.w));
		br_IMU.sendTransform(tf::StampedTransform(imu_tf, ros::Time::now(),  "/origin", "/imu"));

		//Arrow for fall direction

		fall_marker.header.stamp = ros::Time();
		fall_marker.points[1].x = z1[0];
		fall_marker.points[1].y = z1[1];

		markerFall_pub.publish(fall_marker);
*/
		//publish adc
		std_msgs::Int32MultiArray adc;
		adc.data.clear();
		adc.data.push_back(atof(adc0)); //-offsetAdc.data[0]);//-321); //left heel
		adc.data.push_back(atof(adc1)); //-offsetAdc.data[1]);//-246); //left out
		adc.data.push_back(atof(adc2)); //-offsetAdc.data[2]);//-389); //left in
		adc.data.push_back(atof(adc3)); //-offsetAdc.data[3]);//-240); //right heel
		adc.data.push_back(atof(adc4)); //-offsetAdc.data[4]);//-384); //right out
		adc.data.push_back(atof(adc5)); //-offsetAdc.data[5]);//-438); //right in
		adc_pub.publish(adc);


		//ROS_INFO("%d \t %d \t %d \t %d \t %d \t %d\n", adc.data[0], adc.data[1], adc.data[2], adc.data[3], adc.data[4], adc.data[5]);
/*
		//feet marker, center of gravity
		feet_marker.pose.position.x = adc.data[3]-adc.data[0];
		feet_marker.pose.position.y = 0.1;
		markerFeet_pub.publish(feet_marker);

		//calibDone = true;
		if(!calibDone)
		{
			for(int i=0; i<6; i++)
			{
				if(counterCalib>=10) avgAdcCalib.data[i] += adc.data[i];
			}
			counterCalib++;
			if(counterCalib == 20) 
			{		
				for(int i=0; i<6; i++)
				{
					offsetAdc.data[i] = avgAdcCalib.data[i]/10;
				}			
				calibDone = true;
			}
		}
		else
		{
			//run balance
			int left_front_foot = adc.data[1] + adc.data[2];
			int right_front_foot = adc.data[4] + adc.data[5];
			int left_heel = adc.data[0];
			int right_heel = adc.data[3];
			
			if(left_heel + left_front_foot > 100)
			{
				//start balance

				int leftBalanceError = 0;				

				left_front_foot /= 2;	
				left_heel /= BALANCE_FACTOR;
				leftBalanceError = 	left_heel - left_front_foot;

				if((leftBalanceError >= BALANCE_DEADBAND) || (leftBalanceError < -BALANCE_DEADBAND))
				{					
					cmd.motorCmd[45].nodeID = 46;
					cmd.motorCmd[45].mode = POSITION_MODE;
					cmd.motorCmd[45].value = posture.motorData[45].encPosition + leftBalanceError*P_FACTOR;					
				}				
			}

			if(right_heel + right_front_foot > 100)
			{	
				int rightBalanceError = 0;				

				right_front_foot /= 2;	
				right_heel /= BALANCE_FACTOR;
				rightBalanceError = 	right_heel - right_front_foot;
				
				if((rightBalanceError >= BALANCE_DEADBAND) || (rightBalanceError < -BALANCE_DEADBAND))
				{
					cmd.motorCmd[46].nodeID = 47;
					cmd.motorCmd[46].mode = POSITION_MODE;
					cmd.motorCmd[46].value = posture.motorData[46].encPosition + rightBalanceError*P_FACTOR;					
				}
			}
			
			legs_cmd_pub.publish(cmd);
			ros::Duration(0.1).sleep();
			legs_cmd_pub.publish(cmd);
			ros::Duration(0.1).sleep();
		}
*/
	}
	else
	{
		
		//ROS_INFO("if not ok");
/*
		if(w_c == NULL) ROS_INFO("w_c == NULL");
		if(x_c == NULL) ROS_INFO("x_c == NULL");
		if(y_c == NULL) ROS_INFO("y_c == NULL");
		if(z_c == NULL) ROS_INFO("z_c == NULL");
		if(accX_c == NULL) ROS_INFO("accX_c == NULL");
		if(accY_c == NULL) ROS_INFO("accY_c == NULL");
		if(accZ_c == NULL) ROS_INFO("accZ_c == NULL");
		*/
		//ROS_INFO(data);
	}

		//flush the RS232 FIFO
        device.flush();
/*
	while(!postureArrived)
	{
		ros::spinOnce(); //listen to topics
	}
*/
	//printf("...OK\n");	
	//postureArrived = false;

        r.sleep();
    }
}
