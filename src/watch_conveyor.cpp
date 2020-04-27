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

// %Tag(FULLTEXT)%
// %Tag(INCLUDE_STATEMENTS)%
#include <algorithm>
#include <vector>
#include <tuple>

#include <ros/ros.h>
#include <math.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Pose.h>
// %EndTag(INCLUDE_STATEMENTS)%

// %Tag(START_COMP)%
/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
  // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
  ros::ServiceClient start_client =
    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  // If it's not already ready, wait for it to be ready.
  // Calling the Service using the client before the server is ready would fail.
  if (!start_client.exists()) {
    ROS_INFO("Waiting for the competition to be ready...");
    start_client.waitForExistence();
    ROS_INFO("Competition is now ready.");
  }
  ROS_INFO("Requesting competition start...");
  std_srvs::Trigger srv;  // Combination of the "request" and the "response".
  start_client.call(srv);  // Call the start Service.
  if (!srv.response.success) {  // If not successful, print out why.
    ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
  } else {
    ROS_INFO("Competition started!");
  }
}
// %EndTag(START_COMP)%

/// Example class that can hold state and provide methods that handle incoming data.
class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node){}

  /// Called when a new message is received.
  void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
    if (msg->data == "done" && competition_state_ != "done")
    {
      ROS_INFO("Competition ended.");
    }
    competition_state_ = msg->data;
  }

  /// Called when a new Order message is received.
  void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    // ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);

    auto shipment = received_orders_.back().shipments[0];

    std::vector<std::string> priority_list;
    
    for(auto &product : shipment.products){
      // product_index++;
      int priority = 0;
      std::string part = product.type;

      //Check if part is in either of the bins
      if (std::find(bin3_parts_.begin(), bin3_parts_.end(), part) != bin3_parts_.end()){
        priority = 1;
        ROS_INFO("This part is in bin 3");
      }
    
      else if (std::find(bin4_parts_.begin(), bin4_parts_.end(), part) != bin4_parts_.end()){
        priority = 1;
        ROS_INFO("This part is in bin 4");
      }
      
      if (priority == 1)
        priority_list.push_back(product.type);
      else 
        priority_list.insert(priority_list.begin(),product.type);
    }

    ROS_INFO("Order to collect the parts");
    for (std::string &part : priority_list)
      ROS_INFO_STREAM(part);

  }

  void bin3_camera_subscriber(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    auto models = image_msg->models;

    bin3_parts_.clear();
    bin3_part_poses_.clear();

    for (auto &model : models){
      bin3_parts_.push_back(model.type);
      bin3_part_poses_.push_back(model.pose);
    }

  }

  void bin4_camera_subscriber(
    const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
  {
    auto models = image_msg->models;

    bin4_parts_.clear();
    bin4_part_poses_.clear();

    for (auto &model : models){
      bin4_parts_.push_back(model.type);
      bin4_part_poses_.push_back(model.pose);
    }

  }

  void laser_profiler_subscriber(
    const sensor_msgs::LaserScan::ConstPtr & laser_msg)
  {
    auto ranges = laser_msg->ranges;

    auto max = laser_msg->angle_max;
    auto min = laser_msg->angle_min;
    auto increment = laser_msg->angle_increment;


    double alpha = double(min);
    std::vector<int> heights;
    std::vector<double> x_positions;

    bool last_slice = is_part;
    int width;
    int avg_height;
    int offset;

   	for (auto &range: ranges){
    	alpha = alpha + increment;
    	if(range<laser_msg->range_max && range>laser_msg->range_min)
    	{
    		int h = int((range*cos(alpha)-.69)*-1000);
    		
    		if (h > 3){
    			heights.push_back(h);
    			double x_pos = range*sin(alpha);
    			x_positions.push_back(x_pos);
    		}
    	}	
    }

  	if (heights.size() != 0) {
      is_part = 1;
  		double w = x_positions.front() - x_positions.back();
  		offset = int((x_positions.front() + x_positions.back())/2 *1000);
  		
      if (w<0)
  			w = w*-1;

  		width = int(w*1000);
	    int sum = 0;
	    for (int &h: heights)
	    	sum = sum + h;

	    avg_height = int(sum/heights.size());
	  }
	  else
	  	is_part = 0;

	  if (is_part && !last_slice){
	  	start_time = laser_msg->header.stamp;
	  	slice_widths.clear();
	  	slice_heights.clear();
      slice_offsets.clear();

	  	slice_widths.push_back(width);
	  	slice_heights.push_back(avg_height);
	  	slice_offsets.push_back(offset);

	  }
	  if (last_slice && is_part){
	  	slice_widths.push_back(width);
	  	slice_heights.push_back(avg_height);
	  	slice_offsets.push_back(offset);
	  	end_time = laser_msg->header.stamp;
	  }
	  else if (last_slice){
	  	int bound_l = int((end_time.toSec() - start_time.toSec())*.2 *1000);
	  	int bound_w = *std::max_element(slice_widths.begin(),slice_widths.end());
	  	int bound_h = *std::max_element(slice_heights.begin(),slice_heights.end());

	  	std::string type;
	  	if(bound_h < 8)
	  		type = "piston rod";
	  	else if(bound_h >= 8)
	  		type = "gear";

	  	double time = start_time.toSec()+(end_time.toSec() - start_time.toSec())/2;

	  	int sum = 0;
	  	for (int &off: slice_offsets)
	    	sum = sum + off;
	    
	    int x_offset = int(sum/slice_offsets.size());
	  	
	  	auto cur_part = std::make_tuple(type,time,x_offset);

	  	conveyor_parts_.push_back(cur_part);

	  	// ROS_INFO(" ");
	  	// ROS_INFO("A new part was detected. The current list of parts is:");

	  	for (auto &part: conveyor_parts_){
	  		auto item_type = std::get<0>(part);
	      auto break_beam_time = std::get<1>(part);
	      auto x_pos = std::get<2>(part);
	      // ROS_INFO_STREAM("item: " << item_type << " | x position: " << x_pos << " | time at laser: " << break_beam_time);
	  	}

		}
    
  }

  /// Called when a new Proximity message is received.
  void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      conveyor_parts_.erase(conveyor_parts_.begin());
      
      ROS_INFO_STREAM("Break beam triggered. There are " << conveyor_parts_.size() << " parts on the conveyor");

    }
  }

  bool is_part;

private:
  std::string competition_state_;
  std::vector<osrf_gear::Order> received_orders_;

  std::vector<std::string> logical_camera_parts_;
  std::vector<geometry_msgs::Pose> logical_camera_part_poses_;

  std::vector<std::string> bin3_parts_;
  std::vector<geometry_msgs::Pose> bin3_part_poses_;

  std::vector<std::string> bin4_parts_;
  std::vector<geometry_msgs::Pose> bin4_part_poses_;

  std::vector<std::tuple<std::string,double,int>> conveyor_parts_;

  ros::Time start_time;
  ros::Time end_time;

  std::vector<int> slice_widths;
  std::vector<int> slice_heights;
  std::vector<int> slice_offsets;

};


// %Tag(MAIN)%
int main(int argc, char ** argv) {
  // Last argument is the default name of the node.
  ros::init(argc, argv, "conveyor_node");

  ros::NodeHandle node;

  // Instance of custom class from above.
  MyCompetitionClass comp_class(node);

  // Subscribe to the '/ariac/competition_state' topic.
  ros::Subscriber competition_state_subscriber = node.subscribe(
    "/ariac/competition_state", 10,
    &MyCompetitionClass::competition_state_callback, &comp_class);

  // %Tag(SUB_CLASS)%
  // Subscribe to the '/ariac/orders' topic.
  ros::Subscriber orders_subscriber = node.subscribe(
    "/ariac/orders", 10,
    &MyCompetitionClass::order_callback, &comp_class);
  // %EndTag(SUB_CLASS)%

  // Subscribe to the '/ariac/break_beam_2_change' topic.
  ros::Subscriber break_beam_subscriber = node.subscribe(
    "/ariac/break_beam_2_change", 10,
    &MyCompetitionClass::break_beam_callback, &comp_class);

  // Subscribe to the '/ariac/logical_camera_2' topic.
  ros::Subscriber bin3_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_2", 10,
    &MyCompetitionClass::bin3_camera_subscriber, &comp_class);

  // Subscribe to the '/ariac/logical_camera_3' topic.
  ros::Subscriber bin4_camera_subscriber = node.subscribe(
    "/ariac/logical_camera_3", 10,
    &MyCompetitionClass::bin4_camera_subscriber, &comp_class);

 	comp_class.is_part = 0;
  ros::Subscriber laser_profiler_subscriber = node.subscribe(
    "/ariac/laser_profiler_1", 10,
    &MyCompetitionClass::laser_profiler_subscriber, &comp_class);

  ROS_INFO("Setup complete.");
  start_competition(node);

  ros::spin();  // This executes callbacks on new data until ctrl-c.



  return 0;
}
// %EndTag(MAIN)%
// %EndTag(FULLTEXT)%
