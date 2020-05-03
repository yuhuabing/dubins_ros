/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include <tf/tf.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include "reeds_shepp_paths_ros/reeds_shepp_paths_ros.h"
#include <math.h>
#include "std_msgs/Bool.h"


#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include <fstream>
#include <list>
using namespace std;


bool flag=true;
// 四元数转角度（弧度制）
double trans2angle(geometry_msgs::Quaternion q){
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
void trans(vector<double> src, geometry_msgs::PoseStamped& des){
  des.pose.position.x=src[0];
  des.pose.position.y=src[1];
  des.pose.position.z=0;
  //angle转四元数
  double yaw=(src[2]-90)/180*3.14;
  double pitch=0;
  double roll=0;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  des.pose.orientation.w = cy * cp * cr + sy * sp * sr;
  des.pose.orientation.x = cy * cp * sr - sy * sp * cr;
  des.pose.orientation.y = sy * cp * sr + cy * sp * cr;
  des.pose.orientation.z = sy * cp * cr - cy * sp * sr;
}

void chatterCallback(std_msgs::Bool m)
{
   if(m.data)flag=true;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "dubins_ros_demo");
  ros::NodeHandle nh("/");
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("finish_flag", 1000);
  ros::Subscriber sub = nh.subscribe("change_flag", 1000, chatterCallback);
  // 输入输出yaml文件路径
  std::string straightpathstr ="/home/yhb/in.yaml";
  std::string curvepathstr ="/home/yhb/out.yaml";
  reeds_shepp::RSPathsROS RSPlanner("demo", NULL, NULL);
  while(ros::ok()){
    if(flag){
      // 读取yaml文件，设置起始点，终止点
      YAML::Node straightyamlConfig=YAML::LoadFile(straightpathstr);
      int pointnum = straightyamlConfig["terminal_num"].as<int>();
      vector<vector<double> > startposes;
      vector<vector<double> > endposes;
      for(int i=2;i<pointnum;i+=2){
        vector<double> tmpstart,tmpend;
        tmpstart.push_back(straightyamlConfig[i]["point"][0].as<double>());
        tmpstart.push_back(straightyamlConfig[i]["point"][1].as<double>());
        tmpstart.push_back(straightyamlConfig[i]["point"][2].as<double>());
        startposes.push_back(tmpstart);
        tmpend.push_back(straightyamlConfig[i+1]["point"][0].as<double>());
        tmpend.push_back(straightyamlConfig[i+1]["point"][1].as<double>());
        tmpend.push_back(straightyamlConfig[i+1]["point"][2].as<double>());
        endposes.push_back(tmpend);
      }
      YAML::Node yamlConfig;
      std::ofstream outfile;
      outfile.open(curvepathstr);
      int nums=startposes.size();
      yamlConfig["point_num"]=20;
      yamlConfig["path_num"]=nums;
      for(int i=0;i<nums;i++){
        geometry_msgs::PoseStamped start, goal;
        trans(startposes[i],start);
        trans(endposes[i],goal);
        start.header.frame_id = goal.header.frame_id = "map";
        std::vector<geometry_msgs::PoseStamped> pathPoses;
        RSPlanner.planPath(start, goal, pathPoses);
        for(int j=0;j<pathPoses.size();j++){
          double angle=trans2angle(pathPoses[j].pose.orientation)/3.14*180;
          if(angle<0)angle+=360;
          YAML::Node pointTemp = YAML::Load("[]");
          pointTemp.push_back(pathPoses[j].pose.position.x);
          pointTemp.push_back(pathPoses[j].pose.position.y);
          // pointTemp.push_back(pathPoses[j].pose.position.z);
          pointTemp.push_back(angle);
          yamlConfig[i+1][j+1] = pointTemp;
        }
      }
      outfile << yamlConfig; 
      outfile.close();
      std_msgs::Bool std_flag;
      std_flag.data=true;
      pub.publish(std_flag);
      flag=false;
    }
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
  return 0;
}
