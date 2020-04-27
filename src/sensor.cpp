//
// Created by zeid on 2/27/20.
//
#include "sensor.h"

AriacSensorManager::AriacSensorManager(){

    ROS_INFO_STREAM(">>>>> Subscribing to logical sensors");
    camera_1_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_1", 10,
                                                &AriacSensorManager::LogicalCamera1Callback, this);
    camera_2_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_2", 10,
                                                &AriacSensorManager::LogicalCamera2Callback, this);
    camera_3_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_3", 10,
                                                &AriacSensorManager::LogicalCamera3Callback, this);
    camera_4_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_4", 10,
                                                &AriacSensorManager::LogicalCamera4Callback, this);
    camera_5_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_5", 10,
                                                &AriacSensorManager::LogicalCamera5Callback, this);
    camera_6_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_6", 10,
                                                &AriacSensorManager::LogicalCamera6Callback, this);
    camera_7_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_7", 10,
                                                &AriacSensorManager::LogicalCamera7Callback, this);
    // camera_8_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_8", 10,
    //                                             &AriacSensorManager::LogicalCamera8Callback, this);
    // camera_9_subscriber_ = sensor_nh_.subscribe("/ariac/logical_camera_9", 10,
    //                                             &AriacSensorManager::LogicalCamera9Callback, this);
    break_beam_1_subscriber_ = sensor_nh_.subscribe("/ariac/break_beam_1_change", 1000,
                                                &AriacSensorManager::breakBeam1Callback, this);
    break_beam_2_subscriber_ = sensor_nh_.subscribe("/ariac/break_beam_2_change", 1000,
                                                &AriacSensorManager::breakBeam2Callback, this);
    quality_sensor_1_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                                                &AriacSensorManager::QualitySensor1Callback, this);
    quality_sensor_2_subscriber_ = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                                &AriacSensorManager::QualitySensor2Callback, this);
    laser_profiler_subscriber_ = sensor_nh_.subscribe("/ariac/laser_profiler_1", 10,
                                                &AriacSensorManager::LaserProfilerCallback, this);

    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;
    camera7_frame_counter_ = 0;

    break_beam_1_trig_counter_ = 0;
    break_beam_2_trig_counter_ = 0;

    faulty_parts_1_num_ = 0;
    faulty_parts_2_num_ = 0;

    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;
    cam_7_ = false;
    break_beam_1_ = false;
    break_beam_2_ = false;

    is_part = false;

    // ros::spin();

}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::LogicalCamera1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");

    if (image_msg->models.size() == 0) {
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 1 does not see anything");
    }

    current_parts_1_ = *image_msg;
    this->BuildProductFrames(1);
}

void AriacSensorManager::LogicalCamera2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 2 does not see anything");

    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void AriacSensorManager::LogicalCamera3Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 3 does not see anything");

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::LogicalCamera4Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 4: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 4 does not see anything");

    current_parts_4_ = *image_msg;
    this->BuildProductFrames(4);
}

void AriacSensorManager::LogicalCamera5Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 5 does not see anything");

    current_parts_5_ = *image_msg;
    this->BuildProductFrames(5);
}

void AriacSensorManager::LogicalCamera6Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    if (init_) return;
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 6 does not see anything");

    current_parts_6_ = *image_msg;
    this->BuildProductFrames(6);
}

void AriacSensorManager::LogicalCamera7Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    // if (init_) return;   
    ROS_INFO_STREAM_THROTTLE(20,
                             "Logical camera 7: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_WARN_STREAM_THROTTLE(20,"Logical Camera 7 does not see anything");

    current_parts_7_ = *image_msg;
    this->BuildProductFrames(7);
}

void AriacSensorManager::QualitySensor1Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

    ROS_WARN_STREAM_THROTTLE(30,
                             "Quality sensor 1: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_INFO_STREAM_THROTTLE(30,"Quality sensor 1 does not see anything");
    faulty_parts_1_num_ = image_msg->models.size();
    quality_parts_1_ = *image_msg;
    this->BuildProductFrames(8);
}

void AriacSensorManager::QualitySensor2Callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){

    ROS_WARN_STREAM_THROTTLE(30,
                             "Quality sensor 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_INFO_STREAM_THROTTLE(30,"Quality sensor 2 does not see anything");
    faulty_parts_2_num_ = image_msg->models.size();
    quality_parts_2_ = *image_msg;
    this->BuildProductFrames(9);
}

void AriacSensorManager::BuildProductFrames(int camera_id){
    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";
            product_type_pose_.first = product_frame;
            product_type_pose_.second = this->GetPartPose("/world",product_frame);
            products_list_[msg.type].emplace_back(product_type_pose_);
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";
    
            // product_frame_list_[msg.type].emplace_back(product_frame);
            product_type_pose_.first = product_frame;
            product_type_pose_.second = this->GetPartPose("/world",product_frame);
            products_list_[msg.type].emplace_back(product_type_pose_);
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";
            product_type_pose_.first = product_frame;
            product_type_pose_.second = this->GetPartPose("/world",product_frame);
            products_list_[msg.type].emplace_back(product_type_pose_);
            // product_frame_list_[msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
        }
        cam_3_ = true;
    }
    else if (camera_id == 4) {
        for (auto& msg : current_parts_4_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_4_" + msg.type + "_" +
                                        std::to_string(camera4_frame_counter_) + "_frame";
            product_type_pose_.first = product_frame;
            product_type_pose_.second = this->GetPartPose("/world",product_frame);
            products_list_[msg.type].emplace_back(product_type_pose_);
            // product_frame_list_[msg.type].emplace_back(product_frame);
            camera4_frame_counter_++;
        }
        cam_4_ = true;
    }
    else if (camera_id == 5) {
        for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";
            product_type_pose_.first = product_frame;
            product_type_pose_.second = this->GetPartPose("/world",product_frame);
            products_list_[msg.type].emplace_back(product_type_pose_);
            // product_frame_list_[msg.type].emplace_back(product_frame);
            camera5_frame_counter_++;
        }
        cam_5_ = true;
    }
    else if (camera_id == 6) {
        for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";
            product_type_pose_.first = product_frame;
            product_type_pose_.second = this->GetPartPose("/world",product_frame);
            products_list_[msg.type].emplace_back(product_type_pose_);
            // product_frame_list_[msg.type].emplace_back(product_frame);
            camera6_frame_counter_++;
        }
        cam_6_ = true;
    }
    else if (camera_id == 7) {
        // ROS_INFO_STREAM_THROTTLE(10,"Logical camera 7 is built");
        for (auto& msg : current_parts_7_.models) {
            //--build the frame for each product
            std::string product_frame = "logical_camera_7_" + msg.type + "_" +
                  std::to_string(camera7_frame_counter_) + "_frame";
            product_frame_list_conv_[msg.type].emplace_back(product_frame);
            // camera4_frame_counter_++;
        }
        // ROS_INFO("Product Frame" << product_frame);
        cam_7_ = true;
    }
    else if (camera_id == 8){
        for (auto& msg : quality_parts_1_.models) {
            part_list_faulty_.emplace_back(quality_parts_1_.pose);
        }
    }
    else if (camera_id == 9){
        for (auto& msg : quality_parts_2_.models) {
            part_list_faulty_.emplace_back(quality_parts_2_.pose);
        }
    }
    if (cam_1_ && cam_2_ && cam_3_ && cam_4_ && cam_5_ && cam_6_) {
        init_ = true;
    }
}



geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");
    ROS_INFO_STREAM("Source Frame" << src_frame);
    ROS_INFO_STREAM("Target Frame" << target_frame);
    try{
        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);
        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();
        part_pose.orientation.x = camera_tf_transform_.getRotation().x();
        part_pose.orientation.y = camera_tf_transform_.getRotation().y();
        part_pose.orientation.z = camera_tf_transform_.getRotation().z();
        part_pose.orientation.w = camera_tf_transform_.getRotation().w();
    }
    catch (tf::TransformException &ex){
        part_pose.orientation.w = 100.0;
        part_pose.orientation.z = 100.0;
     }

    return part_pose;
}

geometry_msgs::Pose AriacSensorManager::GetPartPoseFromConv(const std::string& src_frame,
                                        const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");
    ROS_INFO_STREAM("Source Frame " << src_frame);
    ROS_INFO_STREAM("Target Frame " << target_frame);

    camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(15));
    camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

    part_pose.position.x = camera_tf_transform_.getOrigin().x();
    part_pose.position.y = camera_tf_transform_.getOrigin().y();
    part_pose.position.z = camera_tf_transform_.getOrigin().z();
    part_pose.orientation.x = camera_tf_transform_.getRotation().x();
    part_pose.orientation.y = camera_tf_transform_.getRotation().y();
    part_pose.orientation.z = camera_tf_transform_.getRotation().z();
    part_pose.orientation.w = camera_tf_transform_.getRotation().w();

    return part_pose;
}

//-- Break beam sensor 1 call back done for RWA-3
void AriacSensorManager::breakBeam1Callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
      ROS_WARN_STREAM("Break beam 1 triggered.");
        break_beam_1_ = true;
        break_beam_1_trig_counter_++;
    } else {
        break_beam_1_ = false;
    }
}

//-- Break beam sensor 2 call back which is right under the logical cam 4 done for RWA-3
void AriacSensorManager::breakBeam2Callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected) {  // If there is an object in proximity.
        ROS_WARN_STREAM("Break beam 2 triggered.");
        break_beam_2_ = true;
        break_beam_2_trig_counter_++;
        conveyor_parts_.erase(conveyor_parts_.begin());
    } else {
        break_beam_2_ = false;
    }
}

void AriacSensorManager::LaserProfilerCallback(const sensor_msgs::LaserScan::ConstPtr & laser_msg){
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

    // For Calulcating part pose
    std::string frame, world_frame;
    world_frame = "/world";
    geometry_msgs::Pose part_pose;
    // part_pose.orientation.x = 0;
    // part_pose.orientation.y = 0;
    // part_pose.orientation.z = 0;
    // part_pose.orientation.w = 0;
    camera_tf_listener_.waitForTransform(world_frame, "/laser_profiler_1_laser_source_frame", ros::Time(0),
                                             ros::Duration(3));
    camera_tf_listener_.lookupTransform(world_frame, "/laser_profiler_1_laser_source_frame", ros::Time(0),
                                            camera_tf_transform_);

    part_pose.position.x = camera_tf_transform_.getOrigin().x();
    part_pose.position.y = camera_tf_transform_.getOrigin().y();
    double laser_y = part_pose.position.y;
    part_pose.position.z = camera_tf_transform_.getOrigin().z();
    // ROS_INFO_STREAM("The pose of the lp is x: " << part_pose.position.x << " y: "
    //     <<part_pose.position.y << " z: "<<part_pose.position.z);


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
            type = "piston_rod_part";
        else if(bound_h >= 8)
            type = "gear_part";

        double time = start_time.toSec()+(end_time.toSec() - start_time.toSec())/2;

        int sum = 0;
        for (int &off: slice_offsets)
            sum = sum + off;
    
        int x_offset = int(sum/slice_offsets.size());
      part_pose.position.x = part_pose.position.x + x_offset/1000;
      part_pose.position.y = -0.5;
      part_pose.position.z = part_pose.position.z - 0.69 + bound_h/1000;

      time = (laser_y + 0.5)/0.2 + time;
        auto cur_part = std::make_tuple(type,time,part_pose);

        conveyor_parts_.push_back(cur_part);

        // ROS_INFO(" ");
        // ROS_INFO("A new part was detected. The current list of parts is:");

        for (auto &part: conveyor_parts_){
            auto item_type = std::get<0>(part);
          auto break_beam_time = std::get<1>(part);
          auto x_pos = std::get<2>(part);
          ROS_INFO_STREAM("item: " << item_type << " | x position: " << x_pos.position.x <<
        " | y position: " << x_pos.position.y <<
        " | z position: " << x_pos.position.z << " | time at laser: " << break_beam_time);
        }

        }

  }
