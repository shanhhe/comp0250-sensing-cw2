/**
  **********************************************************************************
  * @file     cw2_class.h
  * @author   Colin Laganier, Jacob Nash, Carl Parsons
  * @date     2023-04-14
  * @brief   This file contains the header information for the cw2 class.
  *          The goal of the class is to enable the robot to perform the tasks 
  *          in ROS using PCL and octomap.
  **********************************************************************************
  * @attention  Requires ros, moveit, pcl, octomap, tf2 to be installed.
  */

#ifndef cw2_CLASS_H_
#define cw2_CLASS_H_

// system includes
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_listener.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// Set default pointcloud types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointC;
typedef PointC::Ptr PointCPtr;

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

class cw2
{
public:

  double angle_offset_;
  double inspection_distance_;
  double camera_offset_;
  double gripper_open_ = 80e-3;
  double gripper_closed_ = 0.0;
  double cross_pick_grid_y_offset_;
  double cross_pick_grid_x_offset_;
  double naught_pick_grid_x_offset_;
  double naught_pick_grid_y_offset_;
  double drop_height_;
  double pi_ = 3.14159;

  /** \brief Parameters for Task 1 */
  /** \brief Crop box filter. */
  pcl::CropBox<pcl::PointXYZRGBA> nought_filter;
  /** \brief Pass Through filter. */
  pcl::PassThrough<pcl::PointXYZRGBA> cross_filter;

  /** \brief Parameters for Task 3 */
  /** \brief The octomap point cloud frame id. */
  std::string g_octomap_frame_id_;
  /** \brief The octomap point cloud. */
  pcl::PCLPointCloud2 g_octomap_pc;
  /** \brief The octomap point cloud pointer. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_octomap_ptr;
  /** \brief The filtered octomap point cloud pointer. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_octomap_filtered;
  /** \brief The filtered octomap point cloud message. */
  sensor_msgs::PointCloud2 g_octomap_filtered_msg;
  /** \brief ROS octomap publishers. */
  ros::Publisher g_pub_octomap;
  /** \brief ROS octomap publishers. */
  ros::Publisher g_pub_cloud_octomap;
  /** \brief ROS octomap subscriber. */
  ros::Subscriber octomap_pointcloud_sub_;
  /** \brief Task 3 filter flag. */
  bool task_1_filter = false;
  /** \brief Task 3 filter flag. */
  bool task_3_filter = false;
  /** \brief Octomap point cloud filter. */
  pcl::PassThrough<pcl::PointXYZ> g_octomap_pt;
  /** \brief Rounding precision value */
  double position_precision_ = 10000;
  /** \brief Object class for identification. */
  enum Object {cross, nought, basket, obstacle, unknown};
  Object Objects;
  /** \brief ROS subscriber for the color image */
  ros::Subscriber cloud_sub_;

  /** \brief Parameters for Camera image */
  /** \brief ROS subscriber for the color image */
  ros::Subscriber image_sub_;
  /** \brief Camera data */
  std::vector<unsigned char, std::allocator<unsigned char> >color_image_data;
  /** \brief Camera Resolution */
  int color_image_width_;
  int color_image_height_;
  /** \brief Pixel array midpoint */
  int color_image_midpoint_;
  /** \brief Number of colour channels (RGB) */
  int color_channels_ = 3;
  
  /** \brief Arm position to scan entire environment */
  geometry_msgs::Point scan_position_;

  /** \brief The input point cloud frame id. */
  std::string g_input_pc_frame_id_;

  /** \brief ROS publishers. */
  ros::Publisher g_pub_cloud;

  /** \brief ROS Normal publishers. */
  ros::Publisher g_pub_cloud_normals;

  /** \brief ROS pose publishers. */
  ros::Publisher g_pub_pose;
  /** \brief cw1Q1: TF listener definition. */
  tf::TransformListener g_listener_;

  /** \brief Voxel Grid filter's leaf size. */
  double g_vg_leaf_sz;

  /** \brief Point Cloud (input) pointer. */
  PointCPtr g_cloud_ptr;
  
  /** \brief Point Cloud (filtered) pointer. */
  PointCPtr g_cloud_filtered, g_cloud_filtered2, g_cloud_filtered_octomap;
  
  /** \brief Point Cloud (filtered) sensros_msg for publ. */
  sensor_msgs::PointCloud2 g_cloud_filtered_msg;

  /** \brief ROS geometry message point. */
  geometry_msgs::PointStamped g_cyl_pt_msg;
  
  /** \brief Point Cloud (input). */
  pcl::PCLPointCloud2 g_pcl_pc;
  
  /** \brief Voxel Grid filter. */
  pcl::VoxelGrid<PointT> g_vx;
  
  /** \brief Pass Through filter. */
  pcl::PassThrough<PointT> g_pt;

  /** \brief Nearest neighborhooh size for normal estimation. */
  double g_k_nn;
  
  /** \brief Pass Through min and max threshold sizes. */
  double g_pt_thrs_min, g_pt_thrs_max;

  ////////////////////////////////////////////////////////////////////////////////
  // Class member functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Constructor */
  cw2(ros::NodeHandle nh);

  /** \brief Function to load pre-defined constants values */
  void
  load_config();

  // service callbacks for tasks 1, 2, and 3
  bool 
  t1_callback(cw2_world_spawner::Task1Service::Request &request,
    cw2_world_spawner::Task1Service::Response &response);
  bool 
  t2_callback(cw2_world_spawner::Task2Service::Request &request,
    cw2_world_spawner::Task2Service::Response &response);
  bool 
  t3_callback(cw2_world_spawner::Task3Service::Request &request,
    cw2_world_spawner::Task3Service::Response &response);

  /** \brief Point cloud data callback function */
  void   
  pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);
  
  /** \brief RGB camera image callback function*/
  void 
  colorImageCallback(const sensor_msgs::Image& msg);

  /** \brief Octomap Pointcloud callback function */
  void
  octomapCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_input_msg);

  /* ----- class member variables ----- */

  ros::NodeHandle nh_;
  ros::ServiceServer t1_service_;
  ros::ServiceServer t2_service_;
  ros::ServiceServer t3_service_;
  
  /** \brief MoveIt interface to move groups to seperate the arm and the gripper,
  * these are defined in urdf. */
  moveit::planning_interface::MoveGroupInterface arm_group_{"panda_arm"};
  moveit::planning_interface::MoveGroupInterface hand_group_{"hand"};

  ////////////////////////////////////////////////////////////////////////////////
  // Task 1 functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Task 1 function Pick and place object from a current position
   * to a goal using the gripper. 
   *
   * \input[in] object location of object to pick object
   * \input[in] target location to place object in basket
   * \input[in] shape_type shape of given object
   * \input[in] width dimension of the object to pick
   *
   * \return true if object is picked and placed
  */
  bool 
  task_1(geometry_msgs::Point object, 
          geometry_msgs::Point target, 
          std::string shape_type);


  /** \brief MoveIt function for moving the move_group to the target position.
    *
    * \input[in] target_pose pose to move the arm to
    *
    * \return true if moved to target position 
    */
  bool
  moveArm(geometry_msgs::Pose target_pose);

  /** \brief MoveIt function for moving the gripper fingers to a new position. 
    *
    * \input[in] width desired gripper finger width
    *
    * \return true if gripper fingers are moved to the new position
    */
  bool
  moveGripper(float width);

  /** \brief Function to publish the filtered point cloud message.
    *
    * \input[in] pc_pub ROS publisher
    * \input[in] pc point cloud to publish
    */
  void 
  pubFilteredPCMsg(ros::Publisher & pc_pub, PointC & pc);

  /** \brief Function to convert a geometry_msgs point to a pose.
    *
    * \input[in] point point to convert to pose
    *
    * \return pose of the point
    */
  geometry_msgs::Pose
  point2Pose(geometry_msgs::Point point, double rotation = 0.0);

  /// @brief transform the point from link_8 frame to camera frame
  /// @param in_point point in link_8 frame.
  geometry_msgs::Point
  link2Cam (geometry_msgs::Point in_point);
  
  ////////////////////////////////////////////////////////////////////////////////
  // Task 2 functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Task 2 function to detect shapes at given positions
    *
    * \input[in] ref vector of coordinates of objects
    * \input[in] mystery coordinates of unidentified shape
    *
    * \return shape of mystery object (0 or 1)
    */
  int64_t
  task_2(std::vector<geometry_msgs::PointStamped> ref, geometry_msgs::PointStamped mystery);

  /** \brief Function to identify the shape of an object using
    * the current camera feed and a given position.
    *
    * \input[in] point location of the object of interest
    *
    * \return string of the shape
    */
  std::string
  survey(geometry_msgs::Point point);

  ////////////////////////////////////////////////////////////////////////////////
  // Task 3 functions
  ////////////////////////////////////////////////////////////////////////////////

  /** \brief Task 3 function, scan environment, identify objects, count 
    *        them and pick and place the most common object in the basket.
    * 
    * \return tuple of number of most common object and number of objects
    */
  std::tuple<uint64_t, uint64_t>
  task_3();

  /** \brief Function to apply a filter to the point cloud.
    *
    * \input[in] in_cloud_ptr input point cloud
    * \input[out] out_cloud_ptr output point cloud
    */
  void 
  applyFilterTask3(PointCPtr &in_cloud_ptr,
                    PointCPtr &out_cloud_ptr);

  /** \brief Function to scan the environment and create an octomap. */
  void
  scanEnvironment();

  /** \brief Function to cluster the octomap pointcloud into individual
    *        pointclouds for each object
    * 
    * \input[in] cloud PointXYZ pointcloud 
    * 
    * \return vector of PointXYZ pointclouds for each object
    */
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
  clusterPointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  /** \brief Function to identify an object at a given point.
    *  
    * \return Object object identified
    */
  Object
  identifyObject();

  /** \brief Function to get the width of a cluster.
    * 
    * \input[in] cluster pointcloud of cluster
    * 
    * \return double width of cluster
    */
  double 
  getClusterWidth(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster);

};

#endif // end of include guard for cw2_CLASS_H_
