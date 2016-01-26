///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "Client.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <vector>
#include <string.h>

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <unistd.h> // For sleep()

#include <time.h>

using namespace ViconDataStreamSDK::CPP;

namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Kilogram:
        return "Kilogram";
      case Unit::Second:
        return "Second";
      case Unit::Ampere:
        return "Ampere";
      case Unit::Kelvin:
        return "Kelvin";
      case Unit::Mole:
        return "Mole";
      case Unit::Candela:
        return "Candela";
      case Unit::Radian:
        return "Radian";
      case Unit::Steradian:
        return "Steradian";
      case Unit::MeterSquared:
        return "MeterSquared";
      case Unit::MeterCubed:
        return "MeterCubed";
      case Unit::MeterPerSecond:
        return "MeterPerSecond";
      case Unit::MeterPerSecondSquared:
        return "MeterPerSecondSquared";
      case Unit::RadianPerSecond:
        return "RadianPerSecond";
      case Unit::RadianPerSecondSquared:
        return "RadianPerSecondSquared";
      case Unit::Hertz:
        return "Hertz";
      case Unit::Joule:
        return "Joule";
      case Unit::Watt:
        return "Watt";
      case Unit::Pascal:
        return "Pascal";
      case Unit::Lumen:
        return "Lumen";
      case Unit::Lux:
        return "Lux";
      case Unit::Coulomb:
        return "Coulomb";
      case Unit::Ohm:
        return "Ohm";
      case Unit::Farad:
        return "Farad";
      case Unit::Weber:
        return "Weber";
      case Unit::Tesla:
        return "Tesla";
      case Unit::Henry:
        return "Henry";
      case Unit::Siemens:
        return "Siemens";
      case Unit::Becquerel:
        return "Becquerel";
      case Unit::Gray:
        return "Gray";
      case Unit::Sievert:
        return "Sievert";
      case Unit::Katal:
        return "Katal";

      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }

}

int main( int argc, char* argv[] )
{
  // Program options
  std::string ServerName = "192.168.10.1:801";
  std::string HostName = "192.168.10.81";
  std::string TargetSubjectName = "QAV_GREEN";
  std::string ViconBaseFrame = "/world";
  std::string MulticastAddress = "224.0.0.0:44801";
  
  // do the ROS setup
  ros::init(argc, argv, "vicon_udp");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  // parameters
  if (ros::param::has("~target_subject_name")) {
    ROS_INFO("Got target parameter");
    ros::param::get("~target_subject_name",TargetSubjectName);
  }

  // the publisher
  std::string TopicName = "/vicon/" + TargetSubjectName + "/" + TargetSubjectName;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::TransformStamped>(TopicName, 1000);
 
  // initialize the transform
  geometry_msgs::TransformStamped MyTransform;
  MyTransform.header.frame_id = ViconBaseFrame;

  // initialize transform broadcaster
  tf::TransformBroadcaster TfBroadcaster;
  tf::Transform MyTfTransform;

  bool ConnectToMultiCast = true;
  bool EnableMultiCast = false;
  bool EnableHapticTest = false;
  bool bReadCentroids = false;
  std::vector<std::string> HapticOnList(0);

  double QuaternionCheck = 0.0;

  std::ofstream ofs;
  // Make a new client
  Client MyClient;

// Connect to a server
std::cout << "Connecting to " << HostName << " ..." << std::flush;
while( !MyClient.IsConnected().Connected )
{
  // Direct connection

  bool ok = false;
  // Multicast connection
  ok = ( MyClient.ConnectToMulticast( HostName, MulticastAddress ).Result == Result::Success );

  if(!ok)
  {
	std::cout << "Warning - connect failed..." << std::endl;
  }

  std::cout << ".";
  sleep(1);
}
std::cout << "Connected" << std::endl;

ROS_INFO("Connected to multicast address %s", MulticastAddress.c_str());

// Enable some different data types
MyClient.EnableSegmentData();

// Set the streaming mode
//MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
// MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

// Set the global up axis
MyClient.SetAxisMapping( Direction::Forward, 
						 Direction::Left, 
						 Direction::Up ); // Z-up
// MyClient.SetGlobalUpAxis( Direction::Forward, 
//                           Direction::Up, 
//                           Direction::Right ); // Y-up

Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
					   << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
					   << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;

// Discover the version number
Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
std::cout << "Version: " << _Output_GetVersion.Major << "." 
						 << _Output_GetVersion.Minor << "." 
						 << _Output_GetVersion.Point << std::endl;

ROS_INFO("Tracking object %s", TargetSubjectName.c_str());

// now add multicast enable here
Client CtrlClient;
CtrlClient.Connect(ServerName);
assert( CtrlClient.IsConnected().Connected );      
ROS_INFO("Enabling multicast");
CtrlClient.EnableSegmentData();    
CtrlClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );
CtrlClient.StopTransmittingMulticast();
CtrlClient.StartTransmittingMulticast(ServerName, MulticastAddress);
ROS_INFO("Multicast running");

// Loop until a key is pressed
while( ros::ok() )
{
  // Get a frame
  std::cout << "Waiting for new frame..." << std::endl;      

  while( (MyClient.GetFrame().Result != Result::Success) && (ros::ok()) )
  {
	// wait longer
	std::cout << "." << std::endl;
	sleep(1);
  }

  // Get the frame number
  Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
  std::cout << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;
  Output_GetFrameRate Rate = MyClient.GetFrameRate();
  std::cout << "Frame rate: "           << Rate.FrameRateHz          << std::endl;

  // Get the timecode
  Output_GetTimecode _Output_GetTimecode  = MyClient.GetTimecode();

  std::cout << "Timecode: "
			<< _Output_GetTimecode.Hours               << "h "
			<< _Output_GetTimecode.Minutes             << "m " 
			<< _Output_GetTimecode.Seconds             << "s "
			<< _Output_GetTimecode.Frames              << "f "
			<< _Output_GetTimecode.SubFrame            << "sf "
			<< Adapt( _Output_GetTimecode.FieldFlag ) << " " 
			<< _Output_GetTimecode.Standard            << " " 
			<< _Output_GetTimecode.SubFramesPerFrame   << " " 
			<< _Output_GetTimecode.UserBits            << std::endl << std::endl;

  // Get the global rotation of the target segment
  Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = 
		MyClient.GetSegmentGlobalRotationQuaternion( TargetSubjectName, TargetSubjectName );
	  std::cout << "+=+= Global Rotation Quaternion of " << TargetSubjectName << "/" << TargetSubjectName
				              << ": (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]     << ", " 
														   << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]     << ", " 
														   << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]     << ", " 
														   << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]     << ") " 
														   << Adapt( _Output_GetSegmentGlobalRotationQuaternion.Occluded ) << std::endl;

  // Get the global segment translation
  Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
		MyClient.GetSegmentGlobalTranslation( TargetSubjectName, TargetSubjectName );
	  std::cout << "+=+= Global Translation of " << TargetSubjectName << "/" << TargetSubjectName
				              << ": (" << _Output_GetSegmentGlobalTranslation.Translation[ 0 ]  << ", " 
												   << _Output_GetSegmentGlobalTranslation.Translation[ 1 ]  << ", " 
												   << _Output_GetSegmentGlobalTranslation.Translation[ 2 ]  << ") " 
												   << Adapt( _Output_GetSegmentGlobalTranslation.Occluded ) << std::endl;
  
  // sanity check based on quaternion
  QuaternionCheck = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]*_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]
				  + _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]*_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]
				  + _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]*_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]
				  + _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]*_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]
				  - 1.0;
  std::cout << "==== Quaternion Sanity Check: " << QuaternionCheck << std::endl;

  if ((QuaternionCheck <= 1e-15) && (QuaternionCheck >= -1e-15)) {

	// populate the transform
	MyTransform.transform.translation.x = _Output_GetSegmentGlobalTranslation.Translation[ 0 ];
	MyTransform.transform.translation.y = _Output_GetSegmentGlobalTranslation.Translation[ 1 ];
	MyTransform.transform.translation.z = _Output_GetSegmentGlobalTranslation.Translation[ 2 ];

	MyTransform.transform.rotation.x = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ];
	MyTransform.transform.rotation.y = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ];
	MyTransform.transform.rotation.z = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ];
	MyTransform.transform.rotation.w = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ];

	// publish the thing
	pose_pub.publish(MyTransform);

	// and the TF broadcast
	MyTfTransform.setOrigin( tf::Vector3(_Output_GetSegmentGlobalTranslation.Translation[ 0 ] / 1.0e3,
										 _Output_GetSegmentGlobalTranslation.Translation[ 1 ] / 1.0e3,
										 _Output_GetSegmentGlobalTranslation.Translation[ 2 ] / 1.0e3) );
	MyTfTransform.setRotation( tf::Quaternion(_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ],
					   _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ],
					   _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ],
					   _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]) );
	TfBroadcaster.sendTransform(tf::StampedTransform(MyTfTransform, ros::Time::now(), ViconBaseFrame, TopicName));

  } 

  // run ROS activities
  //ros::spinOnce(); // this line produces a boost exception
  //loop_rate.sleep();

}

MyClient.DisableSegmentData();
MyClient.DisableMarkerData();
MyClient.DisableUnlabeledMarkerData();
MyClient.DisableDeviceData();

// Disconnect and dispose
int t = clock();
std::cout << " Disconnecting..." << std::endl;
MyClient.Disconnect();
CtrlClient.StopTransmittingMulticast();
CtrlClient.Disconnect();
int dt = clock() - t;
double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
std::cout << " Disconnect time = " << secs << " secs" << std::endl;

}
