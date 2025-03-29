/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire
solution is contained within the cw2_team_<your_team_number> package */

#include <cw2_class.h> // change to your team name here!

///////////////////////////////////////////////////////////////////////////////

cw2::cw2(ros::NodeHandle nh) : g_cloud_ptr(new PointC),                                          // raw point cloud
                               g_cloud_filtered_vx(new PointC),                                  // Voxel Grid filtered point cloud
                               g_cloud_plane(new PointC),                                        // plane point cloud
                               g_cloud_filtered_plane(new PointC),                               // point cloud exclude plane
                               g_tree_ptr(new pcl::search::KdTree<PointT>),                      // KdTree
                               g_cloud_normals(new pcl::PointCloud<pcl::Normal>),                // segmentation
                               g_cloud_normals_filtered_plane(new pcl::PointCloud<pcl::Normal>), // segmentation
                               g_inliers_plane(new pcl::PointIndices),                           // plane seg
                               g_coeff_plane(new pcl::ModelCoefficients),                        // plane coeff
                               g_coeff_plane_project(new pcl::ModelCoefficients()),              // coeff for projection
                               g_cloud_projected_plane(new PointC)                               // projected object
{
  // Voxel Grid:
  g_vg_leaf_sz = 0.001f;

  // Normal Estimation:
  g_k_nn = 50;

  /* class constructor */

  nh_ = nh;

  // advertise solutions for coursework tasks
  t1_service_  = nh_.advertiseService("/task1_start", 
    &cw2::t1_callback, this);
  t2_service_  = nh_.advertiseService("/task2_start", 
    &cw2::t2_callback, this);
  t3_service_  = nh_.advertiseService("/task3_start",
    &cw2::t3_callback, this);

  // Define the publishers, which can visualize in rviz:
  // point cloud exclude plane
  g_pub_rm_plane = nh_.advertise<sensor_msgs::PointCloud2>("/1", 1, true); 
  g_pub_rm_plane_2 = nh_.advertise<sensor_msgs::PointCloud2>("/2", 1, true);

  // Transformation from camera to panda link 8
  try
  {
    TranCamera2Link8 = tfBuffer.lookupTransform("panda_link8", "rs200_camera", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  // Transformation from panda link_8 to camera
  try
  {
    TranLink82Cam = tfBuffer.lookupTransform("rs200_camera", "panda_link8", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }

  TranCamera2Link8.transform.translation.x = -TranLink82Cam.transform.translation.x;
  TranCamera2Link8.transform.translation.y = -TranLink82Cam.transform.translation.y;
  TranCamera2Link8.transform.translation.z = -TranLink82Cam.transform.translation.z;

  // cout << TranLink82Cam << endl;
  // cout << TranCamera2Link8 << endl;

  ROS_INFO("cw2 class initialised");
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
  cw2_world_spawner::Task1Service::Response &response) 
{
  /* function which should solve task 1 */

  ROS_INFO("The coursework solving callback for task 1 has been triggered");

  // get the object type
  object_shape = request.shape_type;

  // get the object, goal and shape from request
  geometry_msgs::Point object_point = request.object_point.point;
  geometry_msgs::Point goal_point = request.goal_point.point;

  // define the object pose and goal pose
  geometry_msgs::Pose object_pose, goal_pose;

  detect_pose.position = object_point;
  detect_pose.position.z += 0.3;

  double yaw = M_PI / 4;
  double pitch = M_PI;
  double roll = 0;

  tf2::Quaternion quaternion;
  quaternion.setEulerZYX(yaw, pitch, roll);
  detect_pose.orientation = tf2::toMsg(quaternion);

  // move the robot to the detect pose
  bool success = false;
  success = moveArm(detect_pose);

  // open the gripper
  moveGripper(1.2);

  // add the offset to the grasp pose
  if (object_shape == "nought")
  {
    detect_pose.position.x = detect_pose.position.x + 0.08;
    detect_pose.position.y = detect_pose.position.y;
  }
  else if (object_shape == "cross")
  {
    detect_pose.position.x = detect_pose.position.x;
    detect_pose.position.y = detect_pose.position.y + 0.06;
  }

  detect_pose.position.z = 0.2;
  moveArm(detect_pose);
  
  // move to the grasp point
  detect_pose.position.z = 0.15;
  moveArm(detect_pose);
  // close the gripper
  moveGripper(0);
  detect_pose.position.z = 0.5;
  // lift the object
  moveArm(detect_pose);

  goal_pose.orientation = tf2::toMsg(quaternion);
  goal_pose.position = goal_point;
  goal_pose.position.z = 0.3;
  // move to the target point and release
  moveArm(goal_pose);
  moveGripper(1.0);

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  /* function which should solve task 2 */

  ROS_INFO("The coursework solving callback for task 2 has been triggered");

  // retrieve the positions of references
  geometry_msgs::Pose refPose, mysteryPose;
  
  // object type of reference_1 and mytery
  std::string ref_type, mystery_type;

  double yaw = M_PI;
  double pitch = M_PI;
  double roll = 0;

  tf2::Quaternion quaternion;
  quaternion.setEulerZYX(yaw, pitch, roll);

  refPose.orientation = tf2::toMsg(quaternion);

  mysteryPose.orientation = tf2::toMsg(quaternion);

  // transfer the frame for link_8 to camera frame
  refPose.position = link2Cam(request.ref_object_points[0].point);
  mysteryPose.position = link2Cam(request.mystery_object_point.point);
  refPose.position.z += 0.6;
  mysteryPose.position.z += 0.6;

  // define the flag refsuccess to identify the trajectory
  bool refsuccess = false;

  // move to the myterty
  refsuccess = moveArm(mysteryPose);

  task_2_trigger = true; // trigger the task 2 callback
  while (!recog_task_2)
  {
    // wait until the recognization finished
  }
  mystery_type = task_2_objectType;
  refsuccess = false;
  recog_task_2 = false;


  // move to the ahead of the first reference
  refsuccess = moveArm(refPose);

  task_2_trigger = true; // trigger the task 2 callback
  while (!recog_task_2)
  {
    // wait until the recognization finished
  }
  ref_type = task_2_objectType;
  refsuccess = false;
  recog_task_2 = false;

  if (ref_type == mystery_type)
  {
    response.mystery_object_num = 1;
  }
  else
  {
    response.mystery_object_num = 2;
  }

  cout << "##################################" << endl;
  cout << "the response is " << response.mystery_object_num << endl;
  cout << "##################################" << endl;

  return true;
}

///////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  /* function which should solve task 3 */

  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  // initialize
  cluster_poses.clear();

  geometry_msgs::Pose detec_1, detec_2, detec_3, detec_4,
      detec_5, detec_6;

  std::vector<geometry_msgs::Pose> detection;

  // define the rotation
  tf2::Quaternion q_1(-1, 0, 0, 0), q_2, q_3;
  q_2.setRPY(0, 0, M_PI / 4);
  q_3.setRPY(0, 0, -M_PI / 4);
  geometry_msgs::Quaternion detect_Orien_1 = tf2::toMsg(q_1 * q_2);
  geometry_msgs::Quaternion detect_Orien_2 = tf2::toMsg(q_1 * q_3);

  // define the detect points
  detec_1.position.x = 0.32;
  detec_1.position.y = 0.12;
  detec_1.position.z = 0.8;
  detec_1.orientation = detect_Orien_1;
  detection.push_back(detec_1);

  detec_2.position.x = 0.32;
  detec_2.position.y = -0.12;
  detec_2.position.z = 0.8;
  detec_2.orientation = detect_Orien_1;
  detection.push_back(detec_2);

  detec_6.position.x = 0;
  detec_6.position.y = -0.3;
  detec_6.position.z = 0.8;
  detec_6.orientation = detect_Orien_2;
  detection.push_back(detec_6);

  detec_3.position.x = -0.32;
  detec_3.position.y = -0.12;
  detec_3.position.z = 0.8;
  detec_3.orientation = detect_Orien_1;
  detection.push_back(detec_3);

  detec_4.position.x = -0.32;
  detec_4.position.y = 0.12;
  detec_4.position.z = 0.8;
  detec_4.orientation = detect_Orien_1;
  detection.push_back(detec_4);

  detec_5.position.x = 0;
  detec_5.position.y = 0.3;
  detec_5.position.z = 0.8;
  detec_5.orientation = detect_Orien_2;
  detection.push_back(detec_5);

  bool success = false;
  int numb_detection = 1;
  // int numb_detection = detection.size();

  for (int i = 0; i < numb_detection; i++)
  {
    success = moveArm(detection[i]);
    if (success)
    {
      task_3_trigger = true; // trigger the task 3 callback
      success = false;
    }
    else
    {
      ROS_ERROR("Fail to achieve the trajectory!");
    }

    while (!cluster_task_3)
    {
      // wait until the cluster in task 3 finished
    }
    cluster_task_3 = false;

    for (int j = 0; j < cluster_poses.size(); j++)
    {
      // moveArm(cluster_poses[j]);
    }
  }

  // moveArm(detec_1);
  // moveArm(detec_2);
  // moveArm(detec_6);
  // moveArm(detec_3);
  // moveArm(detec_4);
  // moveArm(detec_5);

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// fuction definition

bool cw2::moveArm(geometry_msgs::Pose target_pose)
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

  // google 'c++ conditional operator' to understand this line
  ROS_INFO("Visualising plan %s", success ? "" : "FAILED");

  // execute the planned path
  arm_group_.move();

  return success;
}

////////////////////////////////////////////////////////////////////////////////

bool cw2::moveGripper(float width)
{
  // safety checks in case width exceeds safe values
  if (width > gripper_open_)
    width = gripper_open_;
  if (width < gripper_closed_)
    width = gripper_closed_;

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

  // move the gripper joints
  hand_group_.move();

  return success;
}

///////////////////////////////////////////////////////////////////////////////

void cw2::cloudCallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg)
{
  /////////////////////////////////////////////////////////////////////////
  //////////////////// callback for task 2 ////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  if (task_2_trigger)
  {
    // Extract inout point cloud info
    g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
    pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

    // downsampling the point cloud
    applyVX(g_cloud_ptr, g_cloud_filtered_vx);

    // segment the plane:
    findNormals(g_cloud_filtered_vx); // store in g_cloud_normals
    // g_cloud_filtered_plane inside this function is the PC without plane
    segPlane(g_cloud_filtered_vx);

    // project the filtered pointC on the plane
    projection(g_cloud_filtered_plane);

    // pass through filter
    double thres = 0.02;
    applyPT_x(g_cloud_projected_plane, g_cloud_projected_plane, thres);
    applyPT_y(g_cloud_projected_plane, g_cloud_projected_plane, thres);

    // publish the pointC
    pubFilteredPCMsg(g_pub_rm_plane, *g_cloud_projected_plane);

    task_2_objectType = getObjectType(g_cloud_projected_plane);

    task_2_trigger = false;
  }

  /////////////////////////////////////////////////////////////////////////
  //////////////////// callback for task 3 ////////////////////////////////
  /////////////////////////////////////////////////////////////////////////
  if (task_3_trigger)
  {
    // Extract inout point cloud info
    g_input_pc_frame_id_ = cloud_input_msg->header.frame_id;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_input_msg, g_pcl_pc);
    pcl::fromPCLPointCloud2(g_pcl_pc, *g_cloud_ptr);

    // downsampling the point cloud
    applyVX(g_cloud_ptr, g_cloud_filtered_vx);

    // segment the plane:
    findNormals(g_cloud_filtered_vx); // store in g_cloud_normals
    // g_cloud_filtered_plane inside this function is the PC without plane
    segPlane(g_cloud_filtered_vx);

    pubFilteredPCMsg(g_pub_rm_plane_2, *g_cloud_filtered_plane);

    // projection(g_cloud_filtered_plane);
    // pubFilteredPCMsg(g_pub_rm_plane, *g_cloud_projected_plane);

    Cluster(g_cloud_filtered_plane);

    task_3_trigger = false;
  }
}

///////////////////////////////////////////////////////////////////////////////

void cw2::applyVX(PointCPtr &in_cloud_ptr, PointCPtr &out_cloud_ptr)
{
  g_vx.setInputCloud(in_cloud_ptr);
  g_vx.setLeafSize(g_vg_leaf_sz, g_vg_leaf_sz, g_vg_leaf_sz);
  g_vx.filter(*out_cloud_ptr);
}

/////////////  /* find normal, seg plane and find pose *////////////////////

void cw2::findNormals(PointCPtr &in_cloud_ptr)
{
  // Estimate point normals
  g_ne.setInputCloud(in_cloud_ptr);
  g_ne.setSearchMethod(g_tree_ptr);
  g_ne.setKSearch(g_k_nn);
  g_ne.compute(*g_cloud_normals);
}

///////////////////////////////////////////////////////////////////////////////

void cw2::segPlane(PointCPtr &in_cloud_ptr)
{
  // Create the segmentation object for the planar model
  // and set all the params
  g_seg.setOptimizeCoefficients(true);
  g_seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  g_seg.setNormalDistanceWeight(0.0001); // bad style
  g_seg.setMethodType(pcl::SAC_RANSAC);
  g_seg.setMaxIterations(1000);     // bad style
  g_seg.setDistanceThreshold(0.03); // bad style
  g_seg.setInputCloud(in_cloud_ptr);
  g_seg.setInputNormals(g_cloud_normals);
  // Obtain the plane inliers and coefficients
  g_seg.segment(*g_inliers_plane, *g_coeff_plane);

  // Extract the planar inliers from the input cloud
  g_extract_pc.setInputCloud(in_cloud_ptr);
  g_extract_pc.setIndices(g_inliers_plane);
  g_extract_pc.setNegative(false);

  // Write the planar inliers to disk
  g_extract_pc.filter(*g_cloud_plane);

  // Remove the planar inliers, extract the rest
  g_extract_pc.setNegative(true);
  g_extract_pc.filter(*g_cloud_filtered_plane);
  g_extract_normals.setNegative(true);
  g_extract_normals.setInputCloud(g_cloud_normals);
  g_extract_normals.setIndices(g_inliers_plane);
  g_extract_normals.filter(*g_cloud_normals_filtered_plane);
}

///////////////////////////////////////////////////////////////////////////////

void cw2::pubFilteredPCMsg(ros::Publisher &pc_pub, PointC &pc)
{
  // Publish the data
  pcl::toROSMsg(pc, g_cloud_filtered_msg);
  pc_pub.publish(g_cloud_filtered_msg);
}

///////////////////////////////////////////////////////////////////////////////
void cw2::projection(PointCPtr &in_cloud_ptr)
{
  g_coeff_plane_project->values.resize(4);
  g_coeff_plane_project->values[0] = 0;    // plane normal x
  g_coeff_plane_project->values[1] = 0;    // plane normal y
  g_coeff_plane_project->values[2] = 1;    // plane normal z
  g_coeff_plane_project->values[3] = -0.6; // plane distance from origin

  proj.setInputCloud(in_cloud_ptr);
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setModelCoefficients(g_coeff_plane_project);
  proj.filter(*g_cloud_projected_plane);

  // pubFilteredPCMsg(g_pub_rm_plane, *g_cloud_projected_plane);
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point cw2::findCylPose(PointCPtr &in_cloud_ptr)
{
  Eigen::Vector4f centroid_in;
  pcl::compute3DCentroid(*in_cloud_ptr, centroid_in);

  g_cyl_pt_msg.header.frame_id = g_input_pc_frame_id_;
  g_cyl_pt_msg.header.stamp = ros::Time(0);
  g_cyl_pt_msg.point.x = centroid_in[0];
  g_cyl_pt_msg.point.y = centroid_in[1];
  g_cyl_pt_msg.point.z = centroid_in[2];

  // Transform the point to new frame
  geometry_msgs::PointStamped g_cyl_pt_msg_out;
  try
  {
    g_listener_.transformPoint("panda_link0", // bad styling
                               g_cyl_pt_msg,
                               g_cyl_pt_msg_out);
    // ROS_INFO ("trying transform...");
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("Received a trasnformation exception: %s", ex.what());
  }

  geometry_msgs::Point result;
  result.x = g_cyl_pt_msg_out.point.x;
  result.y = g_cyl_pt_msg_out.point.y;
  result.z = g_cyl_pt_msg_out.point.z;

  return result;
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point cw2::link2Cam(geometry_msgs::Point in_point)
{
  // initialise
  geometry_msgs::PointStamped point_link, point_cam;
  tf2::Quaternion q_0(0, 0, 0, 1);
  TranLink82Cam.transform.rotation = tf2::toMsg(q_0);
  point_link.point = in_point;
  tf2::doTransform(point_link, point_cam, TranLink82Cam);

  return point_cam.point;
}

///////////////////////////////////////////////////////////////////////////////
geometry_msgs::Point cw2::cam2Link(geometry_msgs::Point in_point)
{
  // intialise
  geometry_msgs::PointStamped point_link, point_cam;
  tf2::Quaternion q_0(0, 0, 0, 1);
  TranCamera2Link8.transform.rotation = tf2::toMsg(q_0);
  point_cam.point = in_point;
  tf2::doTransform(point_cam, point_link, TranCamera2Link8);

  return point_link.point;
}

////////////////////////////////////////////////////////////////////////////////

void cw2::applyPT_x(PointCPtr &in_cloud_ptr,
                    PointCPtr &out_cloud_ptr, double &threshold)
{
  double g_pt_thrs = threshold;
  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName("x");
  g_pt.setFilterLimits(-g_pt_thrs, g_pt_thrs);
  g_pt.filter(*out_cloud_ptr);

  return;
}

////////////////////////////////////////////////////////////////////////////////

void cw2::applyPT_y(PointCPtr &in_cloud_ptr,
                    PointCPtr &out_cloud_ptr, double &threshold)
{
  double g_pt_thrs = threshold;
  g_pt.setInputCloud(in_cloud_ptr);
  g_pt.setFilterFieldName("y");
  g_pt.setFilterLimits(-g_pt_thrs, g_pt_thrs);
  g_pt.filter(*out_cloud_ptr);

  return;
}



///////////////////////////////////////////////////////////////////////////////
std::string cw2::getObjectType(PointCPtr &cloud_input)
{
  std::string type;
  type = "error";

  // retrieve the number of points of input pointC
  double num_points = static_cast<double>(cloud_input->size());

  if (num_points == 0)
  {
    type = "nought";
  }
  else
  {
    type = "cross";
  }
  recog_task_2 = true;

  return type;
}

///////////////////////////////////////////////////////////////////////////////
void cw2::Cluster(PointCPtr &in_cloud_ptr)
{
  ROS_INFO("start clustering");

  // pcl::octree::OctreePointCloud<PointT> octree(0.0001);
  // boost::shared_ptr<const pcl::PointCloud<PointT>> input_cloud_boost_ptr =
  //     boost::const_pointer_cast<const pcl::PointCloud<PointT>>(in_cloud_ptr);
  // octree.setInputCloud(input_cloud_boost_ptr);
  // octree.addPointsFromInputCloud();
  // pcl::octree::OctreePointCloud<PointT>::AlignedPointTVector clusters;
  // octree.getOccupiedVoxelCenters(clusters);

  pcl::search::Octree<PointT>::Ptr Octree_ptr(new pcl::search::Octree<PointT>(0.005));
  Octree_ptr->setInputCloud(in_cloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices_task_3;
  cluster_indices_task_3.clear();

  // use k-means to cluster the filtered point cloud, and get the indices
  g_tree_ptr->setInputCloud(in_cloud_ptr);
  pcl::EuclideanClusterExtraction<PointT> g_ec;
  g_ec.setClusterTolerance(0.03);
  g_ec.setMinClusterSize(100);
  g_ec.setMaxClusterSize(5000);
  g_ec.setSearchMethod(Octree_ptr);
  g_ec.setInputCloud(in_cloud_ptr);
  g_ec.extract(cluster_indices_task_3);

  // pcl::RegionGrowingRGB<PointT, pcl::Normal> seg;
  // findNormals(in_cloud_ptr);
  // seg.setInputCloud(in_cloud_ptr);
  // seg.setInputNormals(g_cloud_normals);
  // seg.setSearchMethod(g_tree_ptr);
  // seg.setDistanceThreshold(0.0001f);
  // seg.setRegionColorThreshold(1);
  // seg.setMaxClusterSize(10000);
  // seg.setMinClusterSize(300);
  // seg.extract(cluster_indices_task_3);

  // findNormals(in_cloud_ptr);
  // pcl::RegionGrowing<PointT, pcl::Normal> reg;
  // reg.setMinClusterSize (50);
  // reg.setMaxClusterSize (10000);
  // reg.setSearchMethod (g_tree_ptr);
  // reg.setNumberOfNeighbours (30);
  // reg.setInputCloud (in_cloud_ptr);
  // //reg.setIndices (indices);
  // reg.setInputNormals (g_cloud_normals);
  // reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  // reg.setCurvatureThreshold (1.0);
  // reg.extract (cluster_indices_task_3);

  cout << "##################################" << endl;
  cout << "number of clusters is " << cluster_indices_task_3.size() << endl;
  // cout << "number of clusters is " << clusters.size() << endl;
  cout << "##################################" << endl;

  pcl::PointIndices::Ptr singal_cluster_ptr(new pcl::PointIndices);
  PointCPtr singal_cluster(new PointC);

  geometry_msgs::Pose detectPose;
  tf2::Quaternion q_1(-1, 0, 0, 0), q_2;
  q_2.setRPY(0, 0, M_PI / 4);
  geometry_msgs::Quaternion detectOrien = tf2::toMsg(q_1 * q_2);
  detectPose.orientation = detectOrien;

  *singal_cluster_ptr = cluster_indices_task_3[0];
  g_extract_pc.setInputCloud(in_cloud_ptr);
  g_extract_pc.setIndices(singal_cluster_ptr);
  g_extract_pc.filter(*singal_cluster);
  pubFilteredPCMsg(g_pub_rm_plane, *singal_cluster);

  *singal_cluster_ptr = cluster_indices_task_3[1];
  g_extract_pc.setInputCloud(in_cloud_ptr);
  g_extract_pc.setIndices(singal_cluster_ptr);
  g_extract_pc.filter(*singal_cluster);
  // pubFilteredPCMsg(g_pub_rm_plane_2, *singal_cluster);

  // for (int i = 0; i < cluster_indices_task_3.size(); i++)
  // {
  //   *singal_cluster_ptr = cluster_indices_task_3[i];
  //   g_extract_pc.setInputCloud(in_cloud_ptr);
  //   g_extract_pc.setIndices(singal_cluster_ptr);
  //   g_extract_pc.filter(*singal_cluster);

  //   pubFilteredPCMsg(g_pub_rm_plane, *singal_cluster);

  //   cout << "##################################" << endl;
  //   cout << "position " << findCylPose(singal_cluster) << endl;
  //   cout << "##################################" << endl;

  //   // initialize the r, g, b data
  //   int r = 0;
  //   int g = 0;
  //   int b = 0;
  //   uint32_t rgba;
  //   // retrieve the RGB data
  //   for (int j = 0; j < (*singal_cluster).points.size(); j++)
  //   {
  //     rgba = (*singal_cluster).points[j].rgba;
  //     uint8_t uint8_r = (rgba >> 16) & 0x0000ff;
  //     uint8_t uint8_g = (rgba >> 8) & 0x0000ff;
  //     uint8_t uint8_b = (rgba)&0x0000ff;
  //     uint8_t uint8_a = (rgba >> 24) & 0x000000ff;

  //     r = r + uint8_r;
  //     g = g + uint8_g;
  //     b = b + uint8_b;
  //   }
  //   // take the average number of rgb of the image area
  //   r = r / (*singal_cluster).points.size();
  //   g = g / (*singal_cluster).points.size();
  //   b = b / (*singal_cluster).points.size();

  //   std::cout << "////////////////////////////////////////////" << std::endl;
  //   std::cout << "cluster " << i + 1 << std::endl;
  //   std::cout << "number of points = " << (*singal_cluster).points.size() << std::endl;
  //   std::cout << "r = " << r / 255.0 << std::endl;
  //   std::cout << "g = " << g / 255.0 << std::endl;
  //   std::cout << "b = " << b / 255.0 << std::endl;
  //   std::cout << "////////////////////////////////////////////" << std::endl;

  //   if ((!(r / 255.0 < 0.12 && g / 255.0 < 0.12 && b / 255.0 < 0.12)) &&
  //       ((*singal_cluster).points.size() >= 9000))
  //   {
  //     // pubFilteredPCMsg(g_pub_rm_plane, *singal_cluster);
  //     projection(singal_cluster);
  //     detectPose.position.z = 0.4;
  //     cluster_poses.push_back(detectPose);
  //   }
  // }

  cluster_task_3 = true;
}


////////////////////////////////////////////////////////////////////////////////
