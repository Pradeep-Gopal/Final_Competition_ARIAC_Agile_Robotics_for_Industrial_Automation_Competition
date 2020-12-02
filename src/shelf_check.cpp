// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <vector>
#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> //--needed for tf2::Matrix3x3

#include "competition.h"
#include "utils.h"
#include "gantry_control.h"

#include <nist_gear/AGVControl.h>
#include <tf2/LinearMath/Quaternion.h>

#define MAX_NUMBER_OF_CAMERAS 18

#include <math.h>
#include <stdlib.h>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

using namespace std;

std::vector<std::vector<double>> shelf_vector_comp(9, std::vector<double>(3));
int main(int argc, char **argv) {

  ros::init(argc, argv, "rwa5_node");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(8);
  spinner.start();

  std::vector < std::string > shelf_vector;
  shelf_vector.push_back("/shelf3_frame");
  shelf_vector.push_back("/shelf4_frame");
  shelf_vector.push_back("/shelf5_frame");
  shelf_vector.push_back("/shelf6_frame");
  shelf_vector.push_back("/shelf7_frame");
  shelf_vector.push_back("/shelf8_frame");
  shelf_vector.push_back("/shelf9_frame");
  shelf_vector.push_back("/shelf10_frame");
  shelf_vector.push_back("/shelf11_frame");
  Competition comp(node);
  comp.init();
  for (auto c : shelf_vector) {
    comp.shelf_callback(c);
  }
  shelf_vector_comp = comp.get_shelf_vector();
  ROS_INFO_STREAM("Distance between the shelves");
  for (int i = 0; i <= 7; i++) {
    if (5 <= (abs(shelf_vector_comp[i][0] - shelf_vector_comp[i + 1][0]))
        and (abs(shelf_vector_comp[i][0] - shelf_vector_comp[i + 1][0])) <= 7) {
      ROS_INFO_STREAM(
          "Gaps between shelves" << i + 3 << " and " << i + 4 << " "
              << abs(shelf_vector_comp[i][0] - shelf_vector_comp[i + 1][0]));
    }
  }
  return 0;
}
