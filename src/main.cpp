/* 
 * @copyright
 * Copyright (c) Xsens Technologies B.V., 2006-2012. All rights reserved.

	  This source code is provided under the MT SDK Software License Agreement
and is intended for use only by Xsens Technologies BV and
	   those that have explicit written permission to use it from
	   Xsens Technologies BV.

	  THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	   KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	   IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	   PARTICULAR PURPOSE.
 */

/**
 * @file main.cpp
 * @brief ROS driver for Xsens MTi-10 and MTi-100 series
 * @details This file contains the main function of the ROS node.
 *          It initializes the node and establishes communication with Xsens
 *          device. It also creates an mtiG object that will be used to handle
 *          incoming data packets and ROS messages publishing
 * @author Lucas Casanova Nogueira, based on code from Ji Zhang and Silvio Maeta
 *          in the <a href="http://wiki.ros.org/receive_xsens">receive_xsens</a> package
 */
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

//#include "mtiG.h"

#include "CallbackHandler.h"





/**
 * @brief initializes ros nodes, establishes connection to the device and starts reading from it.
 */
int main(int argc, char** argv)
{
	ros::init(argc, argv, "receive_xsens");
		
	//DEBUG
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();	
	}

	if (!setSerialKey())
	{
		std::cout << "Invalid serial key." << std::endl;
		std::cout << "Press [ENTER] to continue." << std::endl; std::cin.get();
		return 1;
	}

	// Create XsControl object
	XsControl* control = XsControl::construct();
	assert(control != 0);

	try
	{
		// Scan for connected devices
		XsPortInfoArray portInfoArray = XsScanner::scanPorts();

		// Find an MTi / MTx / MTmk4 device
		XsPortInfoArray::const_iterator mtPort = portInfoArray.begin();
		while (mtPort != portInfoArray.end() && !mtPort->deviceId().isMtx() && !mtPort->deviceId().isMtMk4()) {++mtPort;}
		if (mtPort == portInfoArray.end())
		{
			throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
		}
		std::cout << "Found a device with id: " << mtPort->deviceId().toString().toStdString() << " @ port: " << mtPort->portName().toStdString() << ", baudrate: " << mtPort->baudrate() << std::endl;

		// Open the port with the detected device
		if (!control->openPort(mtPort->portName().toStdString(), mtPort->baudrate()))
		{
			throw std::runtime_error("Could not open port. Aborting.");
		}

		try
		{
			// Get the device object
			XsDevice* device = control->device(mtPort->deviceId());
			assert(device != 0);
		
			// Print information about detected MTi / MTx / MTmk4 device
			std::cout << "Device: " << device->productCode().toStdString() << " opened." << std::endl;
			ROS_INFO("Output Mode: %.4x",  device->outputMode() );

			// Creates an mtiG object using the device object and command-line parameters			
			mtiG myXsens(device, argc, argv);

			// Create and attach callback handler to device
			CallbackHandler callback;
			device->addCallbackHandler(&callback);

			// Put the device in measurement mode
			if (!device->gotoMeasurement())
			{
				throw std::runtime_error("Could not put device into measurement mode. Aborting.");
			}

			//std::cout << "\nMain loop (press Ctrl+C to quit)" << std::endl;
			//std::cout << std::string(79, '-') << std::endl;
			while (ros::ok())
			{
				if (callback.packetAvailable())
				{
					
					ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "PACKET INFORMATION: "); 
					
					// Retrieve a packet
					XsDataPacket packet = callback.getNextPacket();

					// Obtains and keeps all relevant data from the packet
					myXsens.fillData(&packet);

					// Publish ROS message
					myXsens.publish();					

					std::cout << std::flush;
				}

    				ros::spinOnce();
				XsTime::msleep(1.0);
			}
			//std::cout << "\r" << std::string(79, '-') << "\n";
			//std::cout << std::endl;
		}
		catch (std::runtime_error const & error)
		{
			std::cout << error.what() << std::endl;
		}
		catch (...)
		{
			std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
		}

		// Close port
		control->closePort(mtPort->portName().toStdString());
	}
	catch (std::runtime_error const & error)
	{
		std::cout << error.what() << std::endl;
	}
	catch (...)
	{
		std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
	}

	// Free XsControl object
	control->destruct();

	std::cout << "Successful exit." << std::endl;

	return 0;
}