#include <submap3d_optimizer/Submap3dOptimizer.h>
#include <sstream>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace submap3d_optimizer{

Submap3dOptimizer::Submap3dOptimizer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_pointCloudSub(NULL),
  m_poseArraySub(NULL),
  m_poseStampedSub(NULL),
  m_tfPointCloudSub(NULL),
  m_tfPoseArraySub(NULL),
  m_tfPoseStampedSub(NULL),

  m_submapSub(NULL),
  m_nodemapSub(NULL),
  m_tfSubmapSub(NULL),
  m_tfNodemapSub(NULL),

  m_SizePoses(0),
  m_global_pc_map_temp(new PCLPointCloud),
  m_global_pc_map(new PCLPointCloud),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_useHeightMap(true),
  m_useColoredMap(false),
  m_colorFactor(0.8),
  m_latchedTopics(true),
  m_publishFreeSpace(false),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_pointcloudMinX(-std::numeric_limits<double>::max()),
  m_pointcloudMaxX(std::numeric_limits<double>::max()),
  m_pointcloudMinY(-std::numeric_limits<double>::max()),
  m_pointcloudMaxY(std::numeric_limits<double>::max()),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_minSizeX(0.0), m_minSizeY(0.0),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07),
  m_compressMap(true),
  m_incrementalUpdate(false),
  m_initConfig(true)
{
  double probHit, probMiss, thresMin, thresMax;

  m_nh_private.param("frame_id", m_worldFrameId, m_worldFrameId); //must change: /map
  m_nh_private.param("base_frame_id", m_baseFrameId, m_baseFrameId); //must change: /base_footprint
  m_nh_private.param("height_map", m_useHeightMap, m_useHeightMap);
  m_nh_private.param("colored_map", m_useColoredMap, m_useColoredMap);
  m_nh_private.param("color_factor", m_colorFactor, m_colorFactor);

  m_nh_private.param("pointcloud_min_x", m_pointcloudMinX,m_pointcloudMinX);
  m_nh_private.param("pointcloud_max_x", m_pointcloudMaxX,m_pointcloudMaxX);
  m_nh_private.param("pointcloud_min_y", m_pointcloudMinY,m_pointcloudMinY);
  m_nh_private.param("pointcloud_max_y", m_pointcloudMaxY,m_pointcloudMaxY);
  m_nh_private.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  m_nh_private.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  m_nh_private.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  m_nh_private.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  m_nh_private.param("min_x_size", m_minSizeX,m_minSizeX);
  m_nh_private.param("min_y_size", m_minSizeY,m_minSizeY);

  m_nh_private.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  m_nh_private.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  m_nh_private.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  m_nh_private.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  m_nh_private.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

  m_nh_private.param("sensor_model/max_range", m_maxRange, m_maxRange);

  m_nh_private.param("resolution", m_res, m_res);
  m_nh_private.param("sensor_model/hit", probHit, 0.7);
  m_nh_private.param("sensor_model/miss", probMiss, 0.4);
  m_nh_private.param("sensor_model/min", thresMin, 0.12);
  m_nh_private.param("sensor_model/max", thresMax, 0.97);
  m_nh_private.param("compress_map", m_compressMap, m_compressMap);
  m_nh_private.param("incremental_2D_projection", m_incrementalUpdate, m_incrementalUpdate);

  if (m_filterGroundPlane && (m_pointcloudMinZ > 0.0 || m_pointcloudMaxZ < 0.0)){
    ROS_WARN_STREAM("You enabled ground filtering but incoming pointclouds will be pre-filtered in ["
              <<m_pointcloudMinZ <<", "<< m_pointcloudMaxZ << "], excluding the ground level z=0. "
              << "This will not work.");
  }

  if (m_useHeightMap && m_useColoredMap) {
    ROS_WARN_STREAM("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_INFO_STREAM("Using RGB color registration (if information available)");
#else
    ROS_ERROR_STREAM("Colored map requested in launch file - node not running/compiled to support colors, please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }

  // initialize octomap object & params
  previousTime = ros::WallTime::now();


  m_maxTreeDepth = m_treeDepth;
  m_gridmap.info.resolution = m_res;

  double r, g, b, a;
  m_nh_private.param("color/r", r, 0.0);
  m_nh_private.param("color/g", g, 0.0);
  m_nh_private.param("color/b", b, 1.0);
  m_nh_private.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  m_nh_private.param("color_free/r", r, 0.0);
  m_nh_private.param("color_free/g", g, 1.0);
  m_nh_private.param("color_free/b", b, 0.0);
  m_nh_private.param("color_free/a", a, 1.0);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;

  m_nh_private.param("publish_free_space", m_publishFreeSpace, m_publishFreeSpace);

  m_nh_private.param("latch", m_latchedTopics, m_latchedTopics);
  if (m_latchedTopics){
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  } else
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");

  // publisher
  m_map3dPub = m_nh.advertise<sensor_msgs::PointCloud2>("map3d", 1, m_latchedTopics);

  // subscriber
  m_pointCloudSub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);
  m_poseArraySub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);
  m_poseStampedSub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);

  m_submapSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "nodemap3d", 5);
  m_nodemapSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "submap3d", 5);

  // tf listener
  m_tfPointCloudSub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPointCloudSub->registerCallback(boost::bind(&Submap3dOptimizer::insertCloudCallback, this, _1));
  m_tfPoseArraySub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseArraySub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseArraySub->registerCallback(boost::bind(&Submap3dOptimizer::subNodePoseCallback, this, _1));
  m_tfPoseStampedSub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseStampedSub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseStampedSub->registerCallback(boost::bind(&Submap3dOptimizer::insertSubmap3dposeCallback, this, _1));

  m_tfSubmapSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_submapSub, m_tfListener, m_worldFrameId, 5);
  m_tfSubmapSub->registerCallback(boost::bind(&Submap3dOptimizer::subSubmapMapCallback, this, _1));
  m_tfNodemapSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_nodemapSub, m_tfListener, m_worldFrameId, 5);
  m_tfNodemapSub->registerCallback(boost::bind(&Submap3dOptimizer::subNodeMapCallback, this, _1));

}

Submap3dOptimizer::~Submap3dOptimizer(){
  if (m_tfPointCloudSub){
    delete m_tfPointCloudSub;
    m_tfPointCloudSub = NULL;
  }

  if (m_pointCloudSub){
    delete m_pointCloudSub;
    m_pointCloudSub = NULL;
  }

  if (m_poseArraySub){
    delete m_poseArraySub;
    m_poseArraySub = NULL;
  }

  if (m_tfPoseArraySub){
    delete m_tfPoseArraySub;
    m_tfPoseArraySub = NULL;
  }

  if (m_tfPoseStampedSub){
    delete m_tfPoseStampedSub;
    m_tfPoseStampedSub = NULL;
  }

  if (m_tfNodemapSub){
    delete m_tfNodemapSub;
    m_tfNodemapSub = NULL;
  }

  if (m_tfSubmapSub){
    delete m_tfSubmapSub;
    m_tfSubmapSub = NULL;
  }
  
  if (m_submapSub){
    delete m_submapSub;
    m_submapSub = NULL;
  }

  if (m_nodemapSub){
    delete m_nodemapSub;
    m_nodemapSub = NULL;
  }

}

// pointcloud map building
void Submap3dOptimizer::insertSubmap3dposeCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("pointcloud get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  //update global map, tuning with live pose_array from cartographer
  for (unsigned i=0; i<node_id_now; ++i){
    auto nodeExist = NodeGraph.find(i+1);
    if (nodeExist!=NodeGraph.end()){
      PCLPointCloud temp = *(nodeExist->second.second);
      Pose nowPose = pose_array->poses[i];
      Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
      Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
      Eigen::Matrix4d Trans; 
      Trans.setIdentity();
      Trans.block<3,3>(0,0) = nowPose_R;
      Trans.block<3,1>(0,3) = nowPose_v;
      pcl::transformPointCloud(temp, temp, Trans);

      //insert nodemap to pointcloud map
      *m_global_pc_map += temp;
    }
  }

  //publish pointcloud globalmap
  if ((ros::WallTime::now()-previousTime).toSec() > 2){
    previousTime = ros::WallTime::now();
    publishPCmap3d(pose_array->header.stamp);
  }
  m_global_pc_map->clear();
  ROS_INFO("Cleared pointcloud map");
}

// octomap map building
void Submap3dOptimizer::insertCloudCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("octomap get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  //update global map, tuning with live pose_array from cartographer
  for (unsigned i=0; i<node_id_now; ++i){
    auto nodeExist = NodeGraph.find(i+1);
    if (nodeExist!=NodeGraph.end()){
      PCLPointCloud temp = *(nodeExist->second.second);
      Pose nowPose = pose_array->poses[i];
      Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
      Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
      Eigen::Matrix4d Trans; 
      Trans.setIdentity();
      Trans.block<3,3>(0,0) = nowPose_R;
      Trans.block<3,1>(0,3) = nowPose_v;
      pcl::transformPointCloud(temp, temp, Trans);

      //insert submap to octomap
      //tf::Point sensorOrigin(nowPose.position.x, nowPose.position.y, nowPose.position.z);
      //insertScan(sensorOrigin, temp);
      total_elapsed = (ros::WallTime::now() - startTime).toSec();
      ROS_INFO("Octomap insertion is done, %f sec)", total_elapsed);
    }
  }

  return;
}

// temp map building
void Submap3dOptimizer::subNodePoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
/*
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("Get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  //update global map, tuning with live pose_array from cartographer
  auto nodeExist = NodeGraph.find(node_id_now);
  if (nodeExist!=NodeGraph.end()){
    PCLPointCloud temp = *(nodeExist->second.second);
    Pose nowPose = pose_array->poses[node_id_now-1];
    Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
    Eigen::Vector3d nowPose_v(nowPose.position.x, nowPose.position.y, nowPose.position.z);
    Eigen::Matrix3d nowPose_R = nowPose_q.toRotationMatrix();
    Eigen::Matrix4d Trans; 
    Trans.setIdentity();
    Trans.block<3,3>(0,0) = nowPose_R;
    Trans.block<3,1>(0,3) = nowPose_v;
    pcl::transformPointCloud(temp, temp, Trans);

    //insert nodemap to pointcloud map
    *m_global_pc_map_temp += temp;

    //insert submap to octomap
    tf::Point sensorOrigin(nowPose.position.x, nowPose.position.y, nowPose.position.z);
    insertScan(sensorOrigin, temp);
    total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO("Octomap insertion is done, %f sec)", total_elapsed);
  }

  //publish octomap/pointcloud globalmap
  if ((ros::WallTime::now()-previousTime).toSec() > 1){
    previousTime = ros::WallTime::now();
    publishPCmap3d(pose_array->header.stamp);
    publishOCTmap3d();
  }
  return;
*/
}

void Submap3dOptimizer::PairwiseICP(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, PCLPointCloud::Ptr &output )
{
  ros::WallTime startTime = ros::WallTime::now();
	PCLPointCloud::Ptr src(new PCLPointCloud);
	PCLPointCloud::Ptr tgt(new PCLPointCloud);
 
	tgt = cloud_target;
	src = cloud_source;
 
	pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
	icp.setMaxCorrespondenceDistance(0.3); //ignore the point out of distance(m)
	icp.setTransformationEpsilon(1e-6); //converge criterion
	icp.setEuclideanFitnessEpsilon(1); //diverge threshold
	icp.setMaximumIterations (5);
	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align (*output);
	
  if (icp.hasConverged()){
    output->resize(tgt->size()+output->size());
    for (int i=0;i<tgt->size();i++)
    {
      output->push_back(tgt->points[i]);
    }
  }else{
    *output += *tgt;
  }
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("After registration using ICP pointcloud size: %d, time cost: %f(s), converged: %d", output->size(), total_elapsed, icp.hasConverged());
}

void Submap3dOptimizer::subSubmapMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ros::WallTime startTime = ros::WallTime::now();
  return;
}


// posepointcloud input callback,  insert Node to NodeGraph
void Submap3dOptimizer::subNodeMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ros::WallTime startTime = ros::WallTime::now();
  PCLPointCloud::Ptr pc (new PCLPointCloud);
  pcl::fromROSMsg(pose_pointcloud->cloud, *pc);
  if (pc->size()<100){
    ROS_WARN("NodeMap ignore size=%zu ", pc->size());
    return;
  }
  NodeGraph.insert(Node(pose_pointcloud->node_id, PoseCloud(pose_pointcloud->pose, pc)));
  ROS_INFO("Get node %d, insert to graph size= %d", pose_pointcloud->node_id, NodeGraph.size());
  return;
}

// publish stored poses and optimized local submap
void Submap3dOptimizer::publishPCmap3d(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  bool publishPCmap3d = (m_latchedTopics || m_map3dPub.getNumSubscribers() > 0);

  if (publishPCmap3d){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (*m_global_pc_map, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_map3dPub.publish(cloud);
  }

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("PointClouldMap3d publishing in visualizer took %f sec", total_elapsed);
}

}