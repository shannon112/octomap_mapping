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

  m_poseArraySub(NULL),
  m_poseStampedSub(NULL),
  m_submapSub(NULL),
  m_nodemapSub(NULL),

  m_tfPoseArraySub(NULL),
  m_tfPoseStampedSub(NULL),
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
  m_poseArrayNewPub = m_nh.advertise<geometry_msgs::PoseArray>("trajectory_pose_array_new", 1, m_latchedTopics);
  m_constraintNewPub = m_nh.advertise<visualization_msgs::MarkerArray>("constraint_list_new", 1, m_latchedTopics);
  m_pointCloudSub = m_nh.subscribe<visualization_msgs::MarkerArray>("constraint_list", 5, &Submap3dOptimizer::constraintCallback, this);

  // subscriber
  m_poseArraySub = new message_filters::Subscriber<geometry_msgs::PoseArray> (m_nh, "trajectory_pose_array", 5);
  m_poseStampedSub = new message_filters::Subscriber<cartographer_ros_msgs::SubmapList> (m_nh, "submap_list", 5);
  m_submapSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "submap3d", 5);
  m_nodemapSub = new message_filters::Subscriber<octomap_server::PosePointCloud2> (m_nh, "nodemap3d", 5);

  // tf listener
  m_tfPoseArraySub = new tf::MessageFilter<geometry_msgs::PoseArray> (*m_poseArraySub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseArraySub->registerCallback(boost::bind(&Submap3dOptimizer::subNodePoseCallback, this, _1));
  m_tfPoseStampedSub = new tf::MessageFilter<cartographer_ros_msgs::SubmapList> (*m_poseStampedSub, m_tfListener, m_worldFrameId, 5);
  m_tfPoseStampedSub->registerCallback(boost::bind(&Submap3dOptimizer::subSubmapPoseCallback, this, _1));
  m_tfSubmapSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_submapSub, m_tfListener, m_worldFrameId, 5);
  m_tfSubmapSub->registerCallback(boost::bind(&Submap3dOptimizer::subSubmapMapCallback, this, _1));
  m_tfNodemapSub = new tf::MessageFilter<octomap_server::PosePointCloud2> (*m_nodemapSub, m_tfListener, m_worldFrameId, 5);
  m_tfNodemapSub->registerCallback(boost::bind(&Submap3dOptimizer::subNodeMapCallback, this, _1));

}

Submap3dOptimizer::~Submap3dOptimizer(){
  // tf subscriber
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
  
  // subscriber
  if (m_poseArraySub){
    delete m_poseArraySub;
    m_poseArraySub = NULL;
  }
  if (m_poseStampedSub){
    delete m_poseStampedSub;
    m_poseStampedSub = NULL;
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

// no used now
void Submap3dOptimizer::subSubmapPoseCallback(const cartographer_ros_msgs::SubmapList::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("enter subSubmapPoseCallback");
  return;
}

// no used now
void Submap3dOptimizer::subSubmapMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("enter subSubmapMapCallback");
  return;
}

// constraint building
void Submap3dOptimizer::constraintCallback(const visualization_msgs::MarkerArray::ConstPtr& constraint_list){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("enter constraintCallback");

  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);
  ceres::examples::SolveOptimizationProblem(&problem);
  ceres::examples::OutputPoses("poses_optimized.txt", poses);

  return;
}

// sub nodemap poses callback, constraint tag to ConstraintGraph
void Submap3dOptimizer::subNodePoseCallback(const geometry_msgs::PoseArray::ConstPtr& pose_array){
  ros::WallTime startTime = ros::WallTime::now();
  ROS_INFO("get pose array sized %zu", pose_array->poses.size());
  unsigned node_id_now = pose_array->poses.size();
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();

  for (unsigned i=0; i<node_id_now; ++i){
    // search now pose
    auto nodeNow = NodeGraph.find(i+1);
    if (nodeNow==NodeGraph.end()) continue;
    PCLPointCloud nowMap = *(nodeNow->second.second);
    Pose nowPose = pose_array->poses[i];

    //search close neighbor pose
    for (unsigned j=i+1; j<node_id_now; ++j){
      auto nodeNeighbor = NodeGraph.find(j+1);
      if (nodeNeighbor==NodeGraph.end()) continue;
      PCLPointCloud neighborMap = *(nodeNeighbor->second.second);

      Pose neighborPose = pose_array->poses[j];
      if (distance(nowPose,neighborPose)>1.0) continue; //distance threshold = 1m

      ROS_INFO("Node %zu neighbor to %zu", i+1, j+1);
    }
  }

  //initialize file i/o
  std::fstream fout;
  fout.open("mit_cartographer.g2o",std::ios::out); //write
  if (!fout) return;
  for (unsigned i=0; i<node_id_now; ++i){
    Pose nowPose = pose_array->poses[i];
    fout<<"VERTEX_SE3:QUAT "<<i<<" "<< nowPose.position.x<<" "<<nowPose.position.y<<" "<<nowPose.position.z<<" "<<
          nowPose.orientation.x<<" "<<nowPose.orientation.y<<" "<<nowPose.orientation.z<<" "<<nowPose.orientation.w<<std::endl;
  }
  for (unsigned i=0; i<node_id_now; ++i){
    if (i==0) continue; //no edge
    Pose nowPose = pose_array->poses[i];
    Pose prePose = pose_array->poses[i-1];
    Pose deltaPose = Pose();

    Eigen::Quaterniond nowPose_q(nowPose.orientation.w, nowPose.orientation.x, nowPose.orientation.y, nowPose.orientation.z);
    Eigen::Quaterniond prePose_q(prePose.orientation.w, prePose.orientation.x, prePose.orientation.y, prePose.orientation.z);
  	Eigen::Matrix3d prePose_R = prePose_q.toRotationMatrix();
	  Eigen::Vector3d deltaPose_v (nowPose.position.x - prePose.position.x, 
        nowPose.position.y - prePose.position.y, nowPose.position.z - prePose.position.z);
    Eigen::Vector3d result_v = prePose_R.transpose()*deltaPose_v;
    deltaPose.position.x = result_v.x();
    deltaPose.position.y = result_v.y();
    deltaPose.position.z = result_v.z();

    Eigen::Quaterniond deltaPose_q = prePose_q.inverse()*nowPose_q;
    deltaPose.orientation.x = deltaPose_q.x();
    deltaPose.orientation.y = deltaPose_q.y();
    deltaPose.orientation.z = deltaPose_q.z();
    deltaPose.orientation.w = deltaPose_q.w();

    fout<<"EDGE_SE3:QUAT "<<i-1<<" "<<i<<" "<< deltaPose.position.x<<" "<<deltaPose.position.y<<" "<<deltaPose.position.z<<" "<<deltaPose.orientation.x<<" "<<deltaPose.orientation.y<<" "<<deltaPose.orientation.z<<" "<<deltaPose.orientation.w;
    fout<<" 1 0 0 0 0 0 "<<"1 0 0 0 0 "<<"1 0 0 0 "<<"4000 0 0 "<<"4000 0 "<<"4000"<<std::endl;
  }

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

  fout.close();
}

// sub nodemap callback, insert Nodemap to NodeGraph
void Submap3dOptimizer::subNodeMapCallback(const octomap_server::PosePointCloud2::ConstPtr& pose_pointcloud){
  ROS_INFO("enter subNodeMapCallback");
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
void Submap3dOptimizer::publishPoseArray(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  bool publishPoseArray = (m_latchedTopics || m_poseArrayNewPub.getNumSubscribers() > 0);

  /*
  if (publishPoseArray){
    geometry_msgs::PoseArray pose_array;

    pose_array.poses.resize(node_poses.size());
    //ROS_INFO("pose array size: %zu",node_poses.size());

    int i = 0;
    for (const auto& node_id_data : node_poses.trajectory(0)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        continue;
      }
      //ROS_INFO("nodeid %d",node_id_data.id.node_index);
      latest_pose_stamp = ToRos(node_id_data.data.constant_pose_data.value().time);
      pose_array.poses[i] = ToGeometryMsgPose(node_id_data.data.global_pose);
      ++i;
    }
    //if(i>0){ROS_INFO("last nodeid %d",(--node_poses.trajectory(0).end())->id.node_index);}

    pose_array.header.stamp = rostime;
    pose_array.header.frame_id = "/map";
    return pose_array;

    m_poseArrayNewPub.publish(pose_array);
  }
  */
  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO("PointClouldMap3d publishing in visualizer took %f sec", total_elapsed);
  return;
}

void Submap3dOptimizer::publishConstriant(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();

}

float Submap3dOptimizer::distance(const Pose &pose_target, const Pose &pose_source){
  float x_delta = pow(pose_target.position.x - pose_source.position.x, 2);
  float y_delta = pow(pose_target.position.y - pose_source.position.y, 2);
  float z_delta = pow(pose_target.position.z - pose_source.position.z, 2);
  return sqrt(x_delta+y_delta+z_delta);
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

}