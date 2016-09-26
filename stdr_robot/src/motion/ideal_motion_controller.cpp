/******************************************************************************
   STDR Simulator - Simple Two DImensional Robot Simulator
   Copyright (C) 2013 STDR Simulator
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
   
   Authors : 
   * Manos Tsardoulias, etsardou@gmail.com
   * Aris Thallas, aris.thallas@gmail.com
   * Chris Zalidis, zalidis@gmail.com 
******************************************************************************/

#include <stdr_robot/motion/ideal_motion_controller.h>
#include <string>
#include <string.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

double special_sin(double err_ang) 
{
    if(err_ang >= M_PI/2 && err_ang <= 3*M_PI/2 || err_ang <= -M_PI/2 && err_ang >= -3*M_PI/2)
    {
	return 1.0;
    }
    else
    {
	return fabs(sin(err_ang));
    }	    
}

double filt(double a, double b, double p) //x comando attuale, y comando filtrato (precedente), p peso
{
    b = p*b + (1-p)*a;
    return b;
}

namespace stdr_robot {
    
  /**
  @brief Default constructor
  @param pose [const geometry_msgs::Pose2D&] The robot pose
  @param tf [tf::TransformBroadcaster&] A ROS tf broadcaster
  @param n [ros::NodeHandle&] The ROS node handle
  @param name [const std::string&] The robot frame id
  @return void
  **/
  IdealMotionController::IdealMotionController(
    const geometry_msgs::Pose2D& pose, 
    tf::TransformBroadcaster& tf, 
    ros::NodeHandle& n, 
    const std::string& name,
    const stdr_msgs::KinematicMsg params)
      : MotionController(pose, tf, name, n, params)
  {
    _calcTimer = n.createTimer(
      _freq, 
      &IdealMotionController::calculateMotion, 
      this);
  }
   
  /**
  @brief Calculates the motion - updates the robot pose
  @param event [const ros::TimerEvent&] A ROS timer event
  @return void
  **/
  void IdealMotionController::calculateMotion(const ros::TimerEvent& event) 
  {
    //!< updates _posePtr based on _currentTwist and time passed (event.last_real)
    count++;
    
    if(count < -5)
	ROS_WARN_STREAM_THROTTLE(1,"PROBLEMA: Robot " << _namespace << " " << count);
    
    ros::Duration dt = ros::Time::now() - event.last_real;
    
//     ROS_WARN_STREAM_THROTTLE(1,"Robot "<< robots.at(id).id << " state " << robots.at(id).robot_state);
    
//     double fx = robots.at(id).ref_x - _pose.x;
//     double fy = robots.at(id).ref_y - _pose.y;
// 	    
//     //Compute linear and angular error for robot i
//     double err_ang = atan2(fy,fx) - _pose.theta;    
//     double err_lin = sqrt(pow(fx,2) + pow(fy,2));

    if (robots.at(id).robot_state == 0)
	{
	    if(sin(robots.at(id).err_ang) > 0)
	    {
		_currentTwist.angular.z = 3*special_sin(robots.at(id).err_ang);
		_currentTwist.linear.x = 0.0;	
		if (_currentTwist.angular.z > 2)
		    _currentTwist.angular.z = 2;
	    }
	    else
	    {
		_currentTwist.angular.z = -3*special_sin(robots.at(id).err_ang);
		_currentTwist.linear.x = 0.0;	
		if (_currentTwist.angular.z < -2)
		    _currentTwist.angular.z = -2;
	    }
	}
    if(robots.at(id).robot_state == 1)
	{
	    _currentTwist.angular.z = 3*special_sin(robots.at(id).err_ang);
	    _currentTwist.linear.x = 5*robots.at(id).err_lin; 
	    if (_currentTwist.linear.x > 50)
		_currentTwist.linear.x = 50;
	    if (_currentTwist.linear.x < 0.1)
		_currentTwist.linear.x = 0.1;
	}
    if(robots.at(id).robot_state == 2)
	{ 
	    _currentTwist.angular.z = 3*special_sin(robots.at(id).err_ang);
	    _currentTwist.linear.x = 1.5*robots.at(id).err_lin; 
	    if (_currentTwist.linear.x > 10)
		_currentTwist.linear.x = 10;
	    if (_currentTwist.linear.x < 0.1)
		_currentTwist.linear.x = 0.1;		
	}
    if(robots.at(id).robot_state == 3)
	{
	    if(sin(robots.at(id).err_ang) > 0)
	    {
		_currentTwist.angular.z = 3*special_sin(robots.at(id).err_ang);
		_currentTwist.linear.x = 0.0;	
		if (_currentTwist.angular.z > 2)
		    _currentTwist.angular.z = 2;
	    }
	    else
	    {
		_currentTwist.angular.z = -3*special_sin(robots.at(id).err_ang);
		_currentTwist.linear.x = 0.0;	
		if (_currentTwist.angular.z < -1)
		    _currentTwist.angular.z = -1;
	    }
	}	 
	
	double p = 0.2;
	
	//////////////////////////////////////////////////////////
    if (fabs(_currentTwist.angular.z) <= 0.0001) 
    {      
      _pose.x = filt(_pose.x + _currentTwist.linear.x * dt.toSec() * cosf(_pose.theta), _pose.x, p);
      _pose.y = filt(_pose.y + _currentTwist.linear.x * dt.toSec() * sinf(_pose.theta), _pose.y, p);
    }
    
    else 
    {  
      _pose.x = filt(_pose.x - _currentTwist.linear.x / _currentTwist.angular.z * sinf(_pose.theta) + _currentTwist.linear.x / _currentTwist.angular.z * 
		sinf(_pose.theta + dt.toSec() * _currentTwist.angular.z), _pose.x, p);
      
      _pose.y = filt(_pose.y + _currentTwist.linear.x / _currentTwist.angular.z * cosf(_pose.theta) - _currentTwist.linear.x / _currentTwist.angular.z * 
		cosf(_pose.theta + dt.toSec() * _currentTwist.angular.z), _pose.y, p);
    }
     _pose.theta = filt(_pose.theta + _currentTwist.angular.z * dt.toSec(), _pose.theta, p);
    
//     if (fabs(_currentTwist.angular.z) <= 0.0001) {
//       
//       _pose.x += _currentTwist.linear.x * dt.toSec() * cosf(_pose.theta);
//       _pose.y += _currentTwist.linear.x * dt.toSec() * sinf(_pose.theta);
//     }
//     
//     else 
//     {  
//       _pose.x += - _currentTwist.linear.x / _currentTwist.angular.z * 
//         sinf(_pose.theta) + 
//         _currentTwist.linear.x / _currentTwist.angular.z * 
//         sinf(_pose.theta + dt.toSec() * _currentTwist.angular.z);
//       
//       _pose.y -= - _currentTwist.linear.x / _currentTwist.angular.z * 
//         cosf(_pose.theta) + 
//         _currentTwist.linear.x / _currentTwist.angular.z * 
//         cosf(_pose.theta + dt.toSec() * _currentTwist.angular.z);
//     }
//     _pose.theta += _currentTwist.angular.z * dt.toSec();
  
    ROS_WARN_STREAM_THROTTLE(1,"Robot: "<< robots.at(id).id << " -> pose_x: " << _pose.x << " -> pose_y: " << _pose.y << " -> pose_theta: " << _pose.theta 
			    << " -> linear_x: " << _currentTwist.linear.x  << " -> angular: " << _currentTwist.angular.z << " -> state: " << robots.at(id).robot_state 
			    << " -> ang_err: " << robots.at(id).err_ang << " -> lin_err: " << robots.at(id).err_lin);   
}
  
  /**
  @brief Default destructor 
  @return void
  **/
  IdealMotionController::~IdealMotionController(void)
  {
    
  }
    
}
