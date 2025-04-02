/**
**********************************************************************************
* @file     cw2_class.cpp
* @author   Colin Laganier, Jacob Nash, Carl Parsons
* @date     2023-04-14
* @brief   This file contains the constructor and methods for the cw2 class.
*          The class advertises the services for the coursework tasks and 
*          triggers the robot to perform the tasks.
**********************************************************************************
* @attention  Requires cw2_class header file.
*/

#include <cw2_class.h>

////////////////////////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////////////////////////
cw2::cw2(ros::NodeHandle nh):
  g_cloud_ptr (new PointC), // input point cloud
  g_cloud_filtered (new PointC), // filtered point cloud
  g_cloud_filtered2 (new PointC), // filtered point cloud
  g_cloud_filtered_octomap (new PointC), // filtered point cloud for octomap
  g_octomap_ptr (new pcl::PointCloud<pcl::PointXYZ>), // input octomap point cloud
  g_octomap_filtered (new pcl::PointCloud<pcl::PointXYZ>) // filtered octomap point cloud
{
  /* class constructor */
  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);

  // Define the publishers
  g_pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("filtered_cloud", 1, true);
  g_pub_pose = nh.advertise<geometry_msgs::PointStamped> ("cyld_pt", 1, true);
  g_pub_cloud_octomap = nh.advertise<sensor_msgs::PointCloud2> ("octomap_cloud", 1, true);
  g_pub_octomap = nh.advertise<sensor_msgs::PointCloud2> ("filtered_octomap_cloud", 1, true);

  // Initialise ROS Subscribers //
  image_sub_ = nh_.subscribe("/r200/camera/color/image_raw", 1, &cw2::colorImageCallback, this);
  // Create a ROS subscriber for the input point cloud
  cloud_sub_ = nh_.subscribe("/r200/camera/depth_registered/points", 1, &cw2::pointCloudCallback, this);
  // Create a ROS subscriber for the octomap output cloud
  octomap_pointcloud_sub_ = nh_.subscribe("/octomap_point_cloud_centers", 1, &cw2::octomapCallback, this);

  load_config();

  ROS_INFO("cw2 class initialised");
}

void cw2::load_config()
{
  // Define constants identified experimentally 

  // Pick and place constants
  inspection_distance_ = 0.6;
  // Angle offset to align gripper with cube
  angle_offset_ = pi_ / 4.0;

  drop_height_ = 0.30;
  cross_pick_grid_y_offset_ = 0;
  cross_pick_grid_x_offset_ = 2;
  naught_pick_grid_x_offset_ = 2;
  naught_pick_grid_y_offset_ = 2;

  // Defining the Robot scanning position for task 3
  scan_position_.x = 0.3773;
  scan_position_.y = -0.0015;
  scan_position_.z = 0.8773;

  g_vg_leaf_sz = 0.01; // VoxelGrid leaf size: Better in a config file
  g_pt_thrs_min = 0.0; // PassThrough min thres: Better in a config file
  g_pt_thrs_max = 0.5; // PassThrough max thres: Better in a config file
  g_k_nn = 50; // Normals nn size: Better in a config file

  // Positions of the gripper for the different tasks 
  camera_offset_ = 0.0425;

}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  bool success = task_1(request.object_point.point, request.goal_point.point, request.shape_type);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  int64_t mystery_object_num = task_2(request.ref_object_points, request.mystery_object_point);

  response.mystery_object_num = mystery_object_num;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  std::tuple<uint64_t, uint64_t> responses = task_3();

  response.total_num_shapes = std::get<0>(responses);
  response.num_most_common_shape = std::get<1>(responses);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

void
cw2::pointCloudCallback
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  // Extract inout point cloud info
  g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;
    
  // Convert to PCL data type
  pcl_conversions::toPCL (*cloud_input_msg, g_pcl_pc);
  pcl::fromPCLPointCloud2 (g_pcl_pc, *g_cloud_ptr);

  // Apply task specific filter
  if (task_1_filter)
  {
    nought_filter.setMin(Eigen::Vector4f(-0.13, 0.05, 0.45, 1.0));
    nought_filter.setMax(Eigen::Vector4f(0.13, 0.4, 0.51, 1.0));
    nought_filter.setInputCloud(g_cloud_ptr);
    nought_filter.filter(*g_cloud_filtered);

    cross_filter.setInputCloud(g_cloud_ptr);
    cross_filter.setFilterFieldName("z");
    cross_filter.setFilterLimits(0.45, 0.51);
    cross_filter.filter(*g_cloud_filtered2);

    pubFilteredPCMsg(g_pub_cloud, *g_cloud_filtered2);
  }
  else if (task_3_filter)
  {
    applyFilterTask3(g_cloud_ptr, g_cloud_filtered_octomap);
    pubFilteredPCMsg(g_pub_cloud_octomap, *g_cloud_filtered_octomap);
  }
  
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Point Cloud callback helper functions
////////////////////////////////////////////////////////////////////////////////

void
cw2::pubFilteredPCMsg (ros::Publisher &pc_pub,
                                PointC &pc)
{
  /* This function publishes the filtered pointcloud */

  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish (g_cloud_filtered_msg);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw2::applyFilterTask3(PointCPtr &in_cloud_ptr,
                      PointCPtr &out_cloud_ptr)
{
  /* Function applying the pass through filter */

  g_pt.setInputCloud(in_cloud_ptr);
  
  // Enlarging the x limits to include the whole plane
  g_pt.setFilterFieldName ("x");
  g_pt.setFilterLimits(-1.0, 1.0);

  // Restricting the z limits to the top of the cubes
  g_pt.setFilterFieldName("z");
  g_pt.setFilterLimits (g_pt_thrs_min, g_pt_thrs_max);
  
  g_pt.filter (*out_cloud_ptr);
  
  return;
}

////////////////////////////////////////////////////////////////////////////////

void
cw2::colorImageCallback(const sensor_msgs::Image& msg)
{
  /* This is the callback function for the RGB camera subscriber */ 
  
  // Setting up the static variables at the first callback 
  static bool setup = [&](){
        // Camera feed resolution
        cw2::color_image_width_ = msg.width;
        cw2::color_image_height_ = msg.height;

        // Computing the index of the middle pixel
        cw2::color_image_midpoint_ = cw2::color_channels_ * ((cw2::color_image_width_ * 
          (cw2::color_image_height_ / 2)) + (cw2::color_image_width_ / 2)) - cw2::color_channels_;

        return true;
    } ();

  this->color_image_data = msg.data;

  return;
}

///////////////////////////////////////////////////////////////////////////////

void
cw2::octomapCallback
  (const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  g_octomap_frame_id_ = cloud_input_msg->header.frame_id;

  // Convert ROS message to PCL point cloud
  pcl_conversions::toPCL(*cloud_input_msg, g_octomap_pc);
  pcl::fromPCLPointCloud2 (g_octomap_pc, *g_octomap_ptr);

  // Create the filtering object
  g_octomap_pt.setInputCloud(g_octomap_ptr);
  g_octomap_pt.setFilterFieldName("z");
  g_octomap_pt.setFilterLimits(0.04, 0.5);
  g_octomap_pt.filter(*g_octomap_filtered);

  // Convert PCL point cloud to ROS message
  pcl::toROSMsg(*g_octomap_filtered, g_octomap_filtered_msg);
  g_pub_octomap.publish(g_octomap_filtered_msg);
}

////////////////////////////////////////////////////////////////////////////////

bool 
cw2::task_1(geometry_msgs::Point object, 
            geometry_msgs::Point target, 
            std::string shape_type)
{
  // define the object pose and goal pose
  geometry_msgs::Pose object_pose, goal_pose;

  object_pose.position = object;
  object_pose.position.z += 0.3;

  double yaw = M_PI / 4;
  double pitch = M_PI;
  double roll = 0;

  tf2::Quaternion quaternion;
  quaternion.setEulerZYX(yaw, pitch, roll);
  object_pose.orientation = tf2::toMsg(quaternion);

  // move the robot to the detect pose
  bool success = false;
  success = moveArm(object_pose);

  // open the gripper
  moveGripper(1.2);

  // add the offset to the grasp pose
  if (shape_type == "nought")
  {
    object_pose.position.x = object_pose.position.x + 0.08;
    object_pose.position.y = object_pose.position.y;
  }
  else if (shape_type == "cross")
  {
    object_pose.position.x = object_pose.position.x;
    object_pose.position.y = object_pose.position.y + 0.06;
  }

  object_pose.position.z = 0.4;
  moveArm(object_pose);
  
  // move to the grasp point
  object_pose.position.z = 0.15;
  moveArm(object_pose);
  // close the gripper
  moveGripper(0);
  object_pose.position.z = 0.5;
  // lift the object
  moveArm(object_pose);

  goal_pose.orientation = tf2::toMsg(quaternion);
  goal_pose.position = target;
  goal_pose.position.z = 0.5;
  // move to the target point and release
  moveArm(goal_pose);
  moveGripper(1.0);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Task 1 helper functions
////////////////////////////////////////////////////////////////////////////////

geometry_msgs::Pose
cw2::point2Pose(geometry_msgs::Point point, double rotation){
  /* This function produces a "gripper facing down" pose given a xyz point */

  // Position gripper above point
  tf2::Quaternion q_x180deg(-1, 0, 0, 0);
  // Gripper Orientation
  tf2::Quaternion q_object;
  q_object.setRPY(0, 0, angle_offset_ + rotation);
  tf2::Quaternion q_result = q_x180deg * q_object;
  geometry_msgs::Quaternion orientation = tf2::toMsg(q_result);

  // set the desired Pose
  geometry_msgs::Pose pose;
  pose.position = point;
  pose.orientation = orientation;

  return pose;
}
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////

bool
cw2::moveArm(geometry_msgs::Pose target_pose)
{
  /* This function moves the move_group to the target position */

  // setup the target pose
  ROS_INFO("Setting pose target");
  arm_group_.setPoseTarget(target_pose);

  // create a movement plan for the arm
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::moveGripper(float width)
{
  /* this function moves the gripper fingers to a new position. Joints are:
      - panda_finger_joint1
      - panda_finger_joint2
  */

  // safety checks
  if (width > gripper_open_) width = gripper_open_;
  if (width < gripper_closed_) width = gripper_closed_;

  // calculate the joint targets as half each of the requested distance
  double eachJoint = width / 2.0;

  // create a vector to hold the joint target for each joint
  std::vector<double> gripperJointTargets(2);
  gripperJointTargets[0] = eachJoint;
  gripperJointTargets[1] = eachJoint;

  // apply the joint target
  hand_group_.setJointValueTarget(gripperJointTargets);

  // move the robot hand
  ROS_INFO("Attempting to plan the path");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (hand_group_.plan(my_plan) ==
    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  hand_group_.move();

  return success;
}


////////////////////////////////////////////////////////////////////////////////
// Task 2
////////////////////////////////////////////////////////////////////////////////

int64_t
cw2::task_2(std::vector<geometry_msgs::PointStamped> ref, geometry_msgs::PointStamped mystery){

  // Initialise output string
  std::string output_string = "Reference Shape 1: ";

  // Inspect first reference shape
  std::string ref_shape_1 = survey(ref[0].point);
  output_string += ref_shape_1;
  output_string += " | Reference Shape 2: ";

  // Infer second reference shape
  std::string ref_shape_2;
  if (ref_shape_1 == "nought"){
    ref_shape_2 = "cross";
  }else{
    ref_shape_2 = "nought";
  }
  output_string += ref_shape_2;
  output_string += " | Mystery Shape: ";

  // Inspect mystery shape
  std::string mystery_shape = survey(mystery.point);
  output_string += mystery_shape;
  int64_t mystery_object_num;
  if (mystery_shape == ref_shape_1){
    mystery_object_num = 1;
  }else{
    mystery_object_num = 2;
  }

  // Print results
  ROS_INFO("/////////////////////////////////////////////////////////////////////");
  ROS_INFO("%s", output_string.c_str());
  ROS_INFO("/////////////////////////////////////////////////////////////////////");

  return mystery_object_num;
}


////////////////////////////////////////////////////////////////////////////////
// Task 2 helper functions
////////////////////////////////////////////////////////////////////////////////


std::string
cw2::survey(geometry_msgs::Point point){

  // Define imaging pose
  geometry_msgs::Pose image_pose = point2Pose(point);
  image_pose.position.z += 0.3; //Offset above object
  image_pose.position.x -= 0.04; // Offset of camera from end-effector

  // Move camera above shape
  bool success = moveArm(image_pose);
  // Extract central pixel values from raw RGB data
  int redValue = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue = color_image_data[color_image_midpoint_ + 2];
  
  // Determine shape
  std::string shape;
  // Nought
  if (greenValue > redValue && greenValue > blueValue){
    shape = "nought";
    ROS_INFO("NOUGHT");}
  // Cross
  else{
    shape = "cross";
    ROS_INFO("CROSS");}
  return shape;
}

///////////////////////////////////////////////////////////////////////////////
// Task 3
///////////////////////////////////////////////////////////////////////////////

std::tuple<uint64_t, uint64_t>
cw2::task_3()
{
  // Scan the environment for objects and obstacles
  scanEnvironment();

  ROS_INFO("Starting to cluster");
  ROS_INFO("11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd11111111d43434343123123123123123111dfddddd");

  // Create snapshot of point cloud for processing
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*g_octomap_filtered, *cloud);

  // Cluster the point cloud into separate objects
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = clusterPointclouds(cloud);

  ROS_INFO("Finished clustering");

  // Initializing the object positions vectors
  std::vector<geometry_msgs::Point> object_positions;
  std::vector<geometry_msgs::Point> obstacle_positions;
  std::vector<geometry_msgs::Point> cross_positions;
  std::vector<geometry_msgs::Point> nought_positions;
  geometry_msgs::Point basket_position;

  for (auto cluster : clusters)
  {
    // Compute the centroid of the cluster
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // Create a point to store the centroid position
    geometry_msgs::Point centroid_position;

    // Round to 3 decimal places
    centroid_position.x = std::round(centroid[0] * position_precision_) / position_precision_;
    centroid_position.y = std::round(centroid[1] * position_precision_) / position_precision_;
    centroid_position.z = 0;

    // Ignore the centroid if it is at the origin
    if (centroid_position.x < 0.1 && centroid_position.x > -0.1 && centroid_position.y < 0.1 && centroid_position.y > -0.1)
    {
      continue;
    }

    ROS_INFO("Object centroid: (%f, %f, %f)", centroid_position.x, centroid_position.y, centroid_position.z);

    pcl::PointXYZ min_point, max_point;
    pcl::getMinMax3D(*cluster, min_point, max_point);
    ROS_INFO("Object dimensions: (%f, %f, %f)", max_point.x - min_point.x, max_point.y - min_point.y, max_point.z - min_point.z);

    // Identify the basket
    if (max_point.x - min_point.x > 0.25 && max_point.y - min_point.y > 0.25)
    {
      basket_position = centroid_position;
    }
    else
    {
      // Compute the width for pick and place
      double width = getClusterWidth(cluster)/5.0;

      // Add the centroid position to the vector
      object_positions.push_back(centroid_position);
    }

  }

  // Sort the object positions
  for (auto object : object_positions)
  {
    // Move the arm to the object
    geometry_msgs::Point target = object;
    target.x = target.x - camera_offset_; // Offset of camera from end-effector
    target.y = target.y;
    target.z = 0.5;
    geometry_msgs::Pose target_pose = point2Pose(target);
    moveArm(target_pose);

    // Identify the object
    Object target_obj = identifyObject();

    if (target_obj == Object::obstacle)
    {
      obstacle_positions.push_back(object);
    }
    else if (target_obj == Object::basket)
    {
      basket_position = object;
    }
    else if (target_obj == Object::cross)
    {
      cross_positions.push_back(object);
    }
    else if (target_obj == Object::nought)
    {
      nought_positions.push_back(object);
    }

    // add delay
    ros::Duration(1.0).sleep();
  }

  if (cross_positions.size() > nought_positions.size())
  {
    geometry_msgs::Point target = cross_positions.back();
    bool success = task_1(target, basket_position, "cross");

    ROS_INFO("/////////////////////////////////////////////////////////////////////");
    ROS_INFO_STREAM("Number of objects: " << cross_positions.size() + nought_positions.size());
    ROS_INFO_STREAM("Number of crosses: " << cross_positions.size());
    ROS_INFO("/////////////////////////////////////////////////////////////////////");


    return std::make_tuple(cross_positions.size() + nought_positions.size(), cross_positions.size());
  }
  else
  {
    geometry_msgs::Point target = nought_positions.back();
    bool success = task_1(target, basket_position, "nought");

    ROS_INFO("/////////////////////////////////////////////////////////////////////");
    ROS_INFO_STREAM("Number of objects: " << cross_positions.size() + nought_positions.size());
    ROS_INFO_STREAM("Number of noughts: " << nought_positions.size());
    ROS_INFO("/////////////////////////////////////////////////////////////////////");

    return std::make_tuple(cross_positions.size() + nought_positions.size(), nought_positions.size());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Task 3 helper functions
////////////////////////////////////////////////////////////////////////////////

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
cw2::clusterPointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  /* Function clustering point clouds in individual groups */

  // Vector to store the clusters
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);

  // Create a set of indices to be used in the extraction
  std::vector<pcl::PointIndices> cluster_indices;
  // Create the extraction object for the clusters
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
  // Set the extraction parameters
  cluster_extraction.setClusterTolerance(0.04); // 4cm
  cluster_extraction.setMinClusterSize(50);  
  cluster_extraction.setMaxClusterSize(10000);
  cluster_extraction.setSearchMethod(tree);
  cluster_extraction.setInputCloud(cloud);
  cluster_extraction.extract(cluster_indices);

  // Loop through each cluster and store it in the vector
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    ROS_INFO("PointCloud representing the Cluster: %lu data points.", cloud_cluster->size());
    clusters.push_back(cloud_cluster);
  }

  return clusters;
}

////////////////////////////////////////////////////////////////////////////////

void
cw2::scanEnvironment()
{
  geometry_msgs::Point reset_point;
  reset_point.x = 0.5;
  reset_point.y = 0.0;
  reset_point.z = 0.5;
  geometry_msgs::Pose reset_pose = point2Pose(reset_point);
  bool reset_success = moveArm(reset_pose);

  // set speed of arm
  // arm_group_.setMaxVelocityScalingFactor(0.05);

  // Create corners of scan area
  geometry_msgs::Point corner1;
  corner1.x = -0.50;
  corner1.y = -0.40;
  corner1.z = 0.6;
  geometry_msgs::Point corner2;
  corner2.x = 0.50;
  corner2.y = -0.40;
  corner2.z = 0.65;
  geometry_msgs::Point corner3;
  corner3.x = 0.50;
  corner3.y = 0.30;
  corner3.z = 0.6;
  geometry_msgs::Point corner4;
  corner4.x = -0.50;
  corner4.y = 0.30;
  corner4.z = 0.6;

  // Add corners to a vector
  std::vector<geometry_msgs::Point> corners;
  corners.push_back(corner1);
  corners.push_back(corner2);
  corners.push_back(corner3);
  corners.push_back(corner4);
  corners.push_back(corner1);

  // Set constant gripper angle
  double angle = -1.5708;
  int num_steps = 4;

  geometry_msgs::Pose pose;
  geometry_msgs::Point distance;
  geometry_msgs::Point step;
  bool success;

  for (int i = 0; i < corners.size() - 1; i++)
  { 
    // Move arm to corner position
    pose = point2Pose(corners.at(i), angle);
    success = moveArm(pose);

    // Enable filter when arm is in position
    if (i == 0)
      task_3_filter = true;

    // Initialise variable to store distance between points
    distance.x = corners.at(i).x - corners.at(i+1).x;
    distance.y = corners.at(i).y - corners.at(i+1).y;
    distance.z = 0;
    
    for (int j = 1; j < num_steps - 1; j++)
    {
      // Calculate step distance
      step.x = corners.at(i).x - (j * distance.x / num_steps);
      step.y = corners.at(i).y - (j * distance.y / num_steps);
      step.z = corners.at(i).z;

      ROS_INFO("Step: (%f, %f, %f)", step.x, step.y, step.z);

      // Move arm to step position
      pose = point2Pose(step, angle);
      success = moveArm(pose);

      if (i == 3 && j == 2)
        j++;
    }
  }
  ros::Duration(1.0).sleep();

  // Disable filter when scan is complete
  task_3_filter = false;
  // Reset the arm velocity
  // arm_group_.setMaxVelocityScalingFactor(0.1);

}

////////////////////////////////////////////////////////////////////////////////

cw2::Object
cw2::identifyObject()
{
  int redValue = color_image_data[color_image_midpoint_];
  int greenValue = color_image_data[color_image_midpoint_ + 1];
  int blueValue = color_image_data[color_image_midpoint_ + 2];

  ROS_INFO("red: %d, green: %d, blue: %d", redValue, greenValue, blueValue);

  if (redValue < 50 && greenValue < 50 && blueValue < 50)
  {
    return Object::obstacle;
  }
  else if (greenValue > redValue && greenValue > blueValue)
  {
    return Object::nought;
  }
  else 
  {
    return Object::cross;
  }
}

////////////////////////////////////////////////////////////////////////////////

double 
cw2::getClusterWidth(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster)
{
  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cluster);
  feature_extractor.compute();

  // Camera space bounding box
  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  
  feature_extractor.getAABB(min_point_AABB, max_point_AABB);
  return max_point_AABB.x - min_point_AABB.x; 
}