#include <submap3d_server/Submap3dServer.h>
#include <sstream>

using namespace octomap;
using octomap_msgs::Octomap;

bool is_equal (double a, double b, double epsilon = 1.0e-7)
{
    return std::abs(a - b) < epsilon;
}

namespace submap3d_server{

Submap3dServer::Submap3dServer(const ros::NodeHandle private_nh_, const ros::NodeHandle &nh_)
: m_nh(nh_),
  m_nh_private(private_nh_),
  m_pointCloudSub(NULL),
  m_poseArraySub(NULL),
  m_tfPointCloudSub(NULL),
  m_tfPoseArraySub(NULL),

  m_icp(false),
  m_SizePoses(0),
  m_local_pc_map(new PCLPointCloud),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), 
  m_baseFrameId("base_footprint"),
  m_trackingFrameId("imu_link"),
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
  m_posePub = m_nh.advertise<geometry_msgs::PoseStamped>("submap3d_pose", 1, m_latchedTopics);
  m_submap3dPub = m_nh.advertise<sensor_msgs::PointCloud2>("submap3d_map", 1, m_latchedTopics);
  m_posePointCloudPub = m_nh.advertise<octomap_server::PosePointCloud2>("submap3d", 1, m_latchedTopics);

  // subscriber
  m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
  m_poseArraySub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);

  // tf listener
  m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
  m_tfPointCloudSub->registerCallback(boost::bind(&Submap3dServer::insertCloudCallback, this, _1));
  m_tfPoseArraySub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseArraySub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseArraySub->registerCallback(boost::bind(&Submap3dServer::insertSubmap3dCallback, this, _1));
}

Submap3dServer::~Submap3dServer(){
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

}

// pose_array input callback
void Submap3dServer::insertSubmap3dCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time latest_pa_stamp = pose_array->header.stamp;

  if (pose_array->poses.size() > m_SizePoses){    
    m_SizePoses = pose_array->poses.size();
    last_pose = pose_array->poses[m_SizePoses-1];

    // transfer to the local ref frame
    Eigen::Quaterniond last_pose_q(last_pose.orientation.w, last_pose.orientation.x, last_pose.orientation.y, last_pose.orientation.z);
    Eigen::Vector3d last_pose_v(last_pose.position.x, last_pose.position.y, last_pose.position.z);
    Eigen::Matrix3d last_pose_R = last_pose_q.toRotationMatrix();
    //Eigen::Matrix4d Trans; 
    //Trans.setIdentity();
    //Trans.block<3,3>(0,0) = last_pose_R;
    //Trans.block<3,1>(0,3) = last_pose_v;
    Eigen::Matrix4d Trans_inverse; 
    Trans_inverse.setIdentity();
    Trans_inverse.block<3,3>(0,0) = last_pose_R.transpose();
    Trans_inverse.block<3,1>(0,3) = -last_pose_R.transpose()*last_pose_v;
    pcl::transformPointCloud(*m_local_pc_map, *m_local_pc_map, Trans_inverse);
    //pcl::transformPointCloud(*m_local_pc_map, *m_local_pc_map, Trans);

    // voxel filter to submap3d
    pcl::VoxelGrid<PCLPoint> voxel_filter;
    voxel_filter.setLeafSize( 0.05, 0.05, 0.05);
    PCLPointCloud::Ptr vx_pc (new PCLPointCloud);
    voxel_filter.setInputCloud(m_local_pc_map);
    voxel_filter.filter(*vx_pc);
    vx_pc->swap(*m_local_pc_map);
    ROS_DEBUG("Pointcloud final pub (%zu pts (local_pc_map))", m_local_pc_map->size());

    // store pose and submap pair
    //m_local_pc_maps.push_back(m_local_pc_map); 
    //m_Poses.push_back(last_pose);

    //publish submap3d_list (submap pair with pose)
    publishSubmap3d(latest_pa_stamp);

    // update to new cycle
    PCLPointCloud::Ptr blank_map (new PCLPointCloud);
    blank_map->swap(*m_local_pc_map);
    m_icp = false;
    //ROS_INFO("Receive new pose!! PoseArray len = %d, m_local_pc_map len = %d, m_Poses len = %d", (int)m_SizePoses, (int)m_local_pc_maps->size(), (int)m_Poses.size());
  }

  return;
}

// pointcloud input callback
void Submap3dServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  ros::WallTime startTime = ros::WallTime::now();
  ros::Time latest_pc_stamp = cloud->header.stamp;

  // transform cloud from ros to pcl
  PCLPointCloud::Ptr pc(new PCLPointCloud);
  pcl::fromROSMsg(*cloud, *pc);

  //depth filter, as kinectv1 spec max depth range is around 3.5~4m, min is 0.8m
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(-4, 4);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(-4, 4);
  pcl::PassThrough<PCLPoint> pass_z_p;
  pass_z_p.setFilterFieldName("z");
  pass_z_p.setFilterLimits(-4, 4);
  pcl::PassThrough<PCLPoint> pass_z_n;
  pass_z_n.setFilterFieldName("z");
  pass_z_n.setFilterLimits(-0.8, 0.8);
  pass_z_n.setFilterLimitsNegative (true);
  pass_x.setInputCloud(pc);
  pass_x.filter(*pc);
  pass_y.setInputCloud(pc);
  pass_y.filter(*pc);
  pass_z_p.setInputCloud(pc);
  pass_z_p.filter(*pc);
  pass_z_n.setInputCloud(pc);
  pass_z_n.filter(*pc);

  // transform pc from cloud frame to global frame
  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  pcl::transformPointCloud(*pc, *pc, sensorToWorld);

  //statistical filter, filtering outlier
  PCLPointCloud::Ptr st_pc(new PCLPointCloud);
  pcl::StatisticalOutlierRemoval<PCLPoint> statistical_filter;
  statistical_filter.setMeanK(50);
  statistical_filter.setStddevMulThresh(1.0);
  statistical_filter.setInputCloud(pc);
  statistical_filter.filter(*st_pc);

  //accumulate pc to build a local pc map  ref to global frame
  PCLPointCloud::Ptr icp_pc(new PCLPointCloud);
  /*
  if(m_local_pc_map->size()==0) {
    *m_local_pc_map += *st_pc;
  }else if (m_icp) {
    ROS_INFO("Pass");
  }else {
    PairwiseICP(st_pc, m_local_pc_map, icp_pc);
    m_local_pc_map = icp_pc;
    m_icp = true;
  }*/
  *m_local_pc_map += *st_pc;
  
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("Pointcloud insertion in OctomapServer done (%zu/%zu pts (pc/local_pc_map), %f sec)", pc->size(), m_local_pc_map->size(), total_elapsed);

  return;
}

// SVD based ICP to align each scan make a submap
void Submap3dServer::PairwiseICP(const PCLPointCloud::Ptr &cloud_target, const PCLPointCloud::Ptr &cloud_source, PCLPointCloud::Ptr &output )
{
	PCLPointCloud::Ptr src(new PCLPointCloud);
	PCLPointCloud::Ptr tgt(new PCLPointCloud);
 
	tgt = cloud_target;
	src = cloud_source;
 
	pcl::IterativeClosestPoint<PCLPoint, PCLPoint> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations (100);
 
	icp.setInputSource (src);
	icp.setInputTarget (tgt);
	icp.align (*output);
//	std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
		
	output->resize(tgt->size()+output->size());
	for (int i=0;i<tgt->size();i++)
	{
		output->push_back(tgt->points[i]);
	}
//ROS_INFO("After registration using ICP: %d", output->size());
}

// publish stored poses and optimized local submap
void Submap3dServer::publishSubmap3d(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  //bool publishSubmap3d = (m_latchedTopics || m_submap3dPub.getNumSubscribers() > 0);
  //bool publishPose = (m_latchedTopics || m_posePub.getNumSubscribers() > 0);
  bool publishPoseSubmap3d = (m_latchedTopics || m_posePointCloudPub.getNumSubscribers() > 0);

  int index_now = (int)(m_SizePoses); // start from 1 = corresponding len of pose array
  //std::stringstream frame_id_now; 
  //frame_id_now << "submap3d_" << index_now;
  
  /*
  if (publishSubmap3d){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (*m_local_pc_map, cloud);
    cloud.header.frame_id = m_worldFrameId;//frame_id_now.str();//
    cloud.header.stamp = rostime;
    m_submap3dPub.publish(cloud);
    ROS_INFO("Published maps");
  }
  */

  /*
  if (publishPose){
    geometry_msgs::PoseStamped poseS;
    poseS.header.frame_id = frame_id_now.str();//m_worldFrameId;//
    poseS.header.stamp = rostime;
    poseS.pose = last_pose;
    m_posePub.publish(poseS);
    ROS_INFO("Published poses");
  }
  */

  if (publishPoseSubmap3d){
    octomap_server::PosePointCloud2 pcloud;
    pcloud.header.frame_id = m_worldFrameId;//m_worldFrameId;//
    pcloud.header.stamp = rostime;
    pcloud.node_id = index_now;
    pcl::toROSMsg (*m_local_pc_map, pcloud.cloud);
    m_posePointCloudPub.publish(pcloud);
    ROS_INFO("Published posePointCloud");
  }
}

}

