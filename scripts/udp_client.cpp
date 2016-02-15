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

  //std::cout << "Starting Vicon UDP connector..." << std::endl << std::flush;
 
  /* int ii;
  for (ii=0; ii<=argc; ii++) {
	  //std::cout << argv[ii] << std::endl;
  } */
  
  // Program options
  
  std::string HostName = "192.168.10.81";
  std::string TargetSubjectName = "QAV_GREEN";
  std::string ViconBaseFrame = "/world";
  std::string MulticastAddress = "224.0.0.0:44801";

  // scaling constant
  double pos_scale = 1e-3;
  
  // do the ROS setup
  ros::init(argc, argv, "udp_client");
  ros::NodeHandle n;

  // ROS parameters
  std::string s;
  // mutlicast address
  if (n.getParam("vicon_multicast_address", s)) {
    MulticastAddress = s;
    //std::cout << "Got multicast " << MulticastAddress << " from parameter." << std::endl;
  }
  // own hostname
  if (n.getParam("vicon_client_hostname", s)) {
    HostName = s;
    //std::cout << "Got hostname " << HostName << " from parameter." << std::endl;
  }
  // vicon frame ID
  if (n.getParam("vicon_tf_parent_frame", s)) {
    ViconBaseFrame = s;
    //std::cout << "Got frame ID " << ViconBaseFrame << " from parameter." << std::endl;
  }
  // tracking object (this one is private as unique to each node)
  // try the private thing using the "bare" method
  if (ros::param::has("~vicon_target_subject")) {
     ros::param::get("~vicon_target_subject", TargetSubjectName);
  }

  // set up ROS publishing
  std::string TopicName = "vicon/" + TargetSubjectName + "/" + TargetSubjectName;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::TransformStamped>(TopicName, 1000);
  ros::Rate loop_rate(300);

  //std::cout << "HostName: " << HostName << std::endl;
  //std::cout << "Multicast: " << MulticastAddress << std::endl;
  //std::cout << "Parent frame: " << ViconBaseFrame << std::endl;
  //std::cout << "Target subject: " << TargetSubjectName << std::endl;

  // initialize the transform
  geometry_msgs::TransformStamped MyTransform;
  MyTransform.header.frame_id = ViconBaseFrame;
  //ROS_INFO("Publishing vicon object %s/%s on topic %s", TargetSubjectName.c_str(), TargetSubjectName.c_str(), TopicName.c_str());
  
  // initialize transform broadcaster
  tf::TransformBroadcaster TfBroadcaster;
  tf::Transform MyTfTransform;

  // log contains:
  // version number
  // log of framerate over time
  // --multicast
  // kill off internal app
  bool ConnectToMultiCast = true;
  bool EnableMultiCast = false;
  bool EnableHapticTest = false;
  bool bReadCentroids = false;
  std::vector<std::string> HapticOnList(0);

  // variables for data retrieval
  Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion;
  Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation;
  Output_GetFrame _Output_GetFrame;
  
  // Make a new client
  Client MyClient;

    // Connect to a server
    ROS_INFO("Connecting to multicast %s as host %s", MulticastAddress.c_str(), HostName.c_str());
    //std::cout << "Connecting to " << HostName << " ..." << std::flush;
    while( !MyClient.IsConnected().Connected )
    {
      // Direct connection

      bool ok = false;
      if(ConnectToMultiCast)
      {
        // Multicast connection
        ok = ( MyClient.ConnectToMulticast( HostName, MulticastAddress ).Result == Result::Success );

      }
      else
      {
        ok =( MyClient.Connect( HostName ).Result == Result::Success );
      }
      if(!ok)
      {
        ////std::cout << "Warning - connect failed..." << std::endl;
        ROS_WARN("Vicon connection failed...");
      }


      //std::cout << ".";
      sleep(1);
    }
    //std::cout << "Connected" << std::endl;

    // Enable some different data types
    MyClient.EnableSegmentData();
    //MyClient.EnableMarkerData();
    //MyClient.EnableUnlabeledMarkerData();
    //MyClient.EnableDeviceData();
    if( bReadCentroids )
    {
      MyClient.EnableCentroidData();
    }

    //std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
    //std::cout << "Marker Data Enabled: "           << Adapt( MyClient.IsMarkerDataEnabled().Enabled )          << std::endl;
    //std::cout << "Unlabeled Marker Data Enabled: " << Adapt( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
    //std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;
    //std::cout << "Centroid Data Enabled: "         << Adapt( MyClient.IsCentroidDataEnabled().Enabled )        << std::endl;

    // Set the streaming mode
    MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up ); // Z-up
    // MyClient.SetGlobalUpAxis( Direction::Forward, 
    //                           Direction::Up, 
    //                           Direction::Right ); // Y-up

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    //std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
     //                      << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
     //                      << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    //std::cout << "Version: " << _Output_GetVersion.Major << "." 
    //                         << _Output_GetVersion.Minor << "." 
    //                         << _Output_GetVersion.Point << std::endl;

    if( EnableMultiCast )
    {
      assert( MyClient.IsConnected().Connected );
      MyClient.StartTransmittingMulticast( HostName, MulticastAddress );
    }

    //ROS_INFO("Publishing vicon object %s relative to %s", TargetSubjectName.c_str(), ViconBaseFrame.c_str());
    ROS_INFO("Publishing vicon object %s", TargetSubjectName.c_str());

    size_t FrameRateWindow = 1000; // frames
    size_t Counter = 0;
    clock_t LastTime = clock();
    // Loop until a key is pressed
    while( ros::ok() )
    {
		
      // get ROS stuff done first
      ros::spinOnce();

      // a pause
      loop_rate.sleep();

      //try {

      // Get a frame
      //ROS_INFO("Getting new frame...");
      _Output_GetFrame = MyClient.GetFrame();
      //ROS_INFO("Got frame");
      if (_Output_GetFrame.Result != Result::Success) {
		  ROS_INFO("Missed frame");
		  //std::cout << "Didn't get new frame..." << std::endl << std::flush;
		  continue;
	  }

      // Get the frame number
      //Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      ////std::cout << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

      //Output_GetFrameRate Rate = MyClient.GetFrameRate();
      ////std::cout << "Frame rate: "           << Rate.FrameRateHz          << std::endl;

      // Get the timecode
      //Output_GetTimecode _Output_GetTimecode  = MyClient.GetTimecode();

      /* //std::cout << "Timecode: "
                << _Output_GetTimecode.Hours               << "h "
                << _Output_GetTimecode.Minutes             << "m " 
                << _Output_GetTimecode.Seconds             << "s "
                << _Output_GetTimecode.Frames              << "f "
                << _Output_GetTimecode.SubFrame            << "sf "
                << Adapt( _Output_GetTimecode.FieldFlag ) << " " 
                << _Output_GetTimecode.Standard            << " " 
                << _Output_GetTimecode.SubFramesPerFrame   << " " 
                << _Output_GetTimecode.UserBits            << std::endl << std::endl;
      */
      
      // Get the global rotation of the target segment
      _Output_GetSegmentGlobalRotationQuaternion = 
            MyClient.GetSegmentGlobalRotationQuaternion( TargetSubjectName, TargetSubjectName );

      // test if we got it OK
      if (_Output_GetSegmentGlobalRotationQuaternion.Result == Result::Success) {
          /* std::cout << "+=+= Global Rotation Quaternion of " << TargetSubjectName << "/" << TargetSubjectName
					              << ": (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]     << ", " 
                                                               << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]     << ", " 
                                                               << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]     << ", " 
                                                               << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]     << ") " 
                                                               << Adapt( _Output_GetSegmentGlobalRotationQuaternion.Occluded ) << std::endl; */
      }
      else {
         //std::cout << "Failed to get quaternion" << std::endl;
         continue;
      }

      // Get the global segment translation
      _Output_GetSegmentGlobalTranslation = 
            MyClient.GetSegmentGlobalTranslation( TargetSubjectName, TargetSubjectName );

      // test if we got it OK
      if (_Output_GetSegmentGlobalTranslation.Result == Result::Success) {
          /* std::cout << "+=+= Global Translation of " << TargetSubjectName << "/" << TargetSubjectName
					              << ": (" << _Output_GetSegmentGlobalTranslation.Translation[ 0 ]  << ", " 
                                                       << _Output_GetSegmentGlobalTranslation.Translation[ 1 ]  << ", " 
                                                       << _Output_GetSegmentGlobalTranslation.Translation[ 2 ]  << ") " 
                                                       << Adapt( _Output_GetSegmentGlobalTranslation.Occluded ) << std::endl; */
      }
      else {
         //std::cout << "Failed to get translation" << std::endl;
         continue;
      }

        // populate the transform
        MyTransform.transform.translation.x = _Output_GetSegmentGlobalTranslation.Translation[ 0 ] * pos_scale;
        MyTransform.transform.translation.y = _Output_GetSegmentGlobalTranslation.Translation[ 1 ] * pos_scale;
        MyTransform.transform.translation.z = _Output_GetSegmentGlobalTranslation.Translation[ 2 ] * pos_scale;

        MyTransform.transform.rotation.x = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ];
        MyTransform.transform.rotation.y = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ];
        MyTransform.transform.rotation.z = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ];
        MyTransform.transform.rotation.w = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ];

        // publish the thing
        pose_pub.publish(MyTransform);

        // and the TF broadcast
        MyTfTransform.setOrigin( tf::Vector3(_Output_GetSegmentGlobalTranslation.Translation[ 0 ] * pos_scale,
                                             _Output_GetSegmentGlobalTranslation.Translation[ 1 ] * pos_scale,
                                             _Output_GetSegmentGlobalTranslation.Translation[ 2 ] * pos_scale) );
        MyTfTransform.setRotation( tf::Quaternion(_Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ],
						   _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ],
						   _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ],
						   _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]) );
        TfBroadcaster.sendTransform(tf::StampedTransform(MyTfTransform, ros::Time::now(), ViconBaseFrame, TopicName));
      //}
      /* catch (...) {
		  ROS_INFO("Vicon problem");
	  }
      */
    }

    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();

    ROS_INFO("Disconnecting from Vicon...");

    // Disconnect and dispose
    int t = clock();
    //std::cout << " Disconnecting..." << std::endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    //std::cout << " Disconnect time = " << secs << " secs" << std::endl;

}
