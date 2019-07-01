#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//#include <obj_recognition/SegmentedClustersArray.h>
//#include <obj_recognition/ClusterData.h>

ros::Publisher pub;
std::string frame_name_min = "object_min";
std::string frame_name_max = "object_max";
std::string frame_name_head = "object_head";
std::string frame_name_cent = "object_cent";
int GLOBAL_COUNTER = 0;
void findHighest(const pcl::PointCloud<pcl::PointXYZRGB> &pclObj, std::vector<pcl::PointXYZRGB>& vecs)
{
  std::vector<double> max = {-100000,-100000,-100000};
  std::vector<double> min = {100000,100000,100000};
  for (int i = 0; i < pclObj.size(); i++)
  {
    auto a = pclObj.points[i];
    std::vector<double> x = {a.x,a.y,a.z};
    for (int j = 0; j < 3; j++)
    {
      max[j] = std::max(max[j], x[j]);
      min[j] = std::min(min[j], x[j]);
    }
  }
  pcl::PointXYZRGB ptMax, ptMin;
  ptMax.x = max[0];ptMax.y = max[1];ptMax.z = max[2];
  ptMin.x = min[0];ptMin.y = min[1];ptMin.z = min[2];
  vecs = {ptMin, ptMax};
}

void publishTFs(const pcl::PointCloud<pcl::PointXYZRGB> &pclObj, std_msgs::Header header, int cont)
{
  static tf::TransformListener listener;
  tf::Transform tf_point;
  static tf::TransformBroadcaster br;
  geometry_msgs::PointStamped point_root, point_camera;
  ROS_INFO ("POINT: %d", cont);
  pcl::PointXYZRGB point = pclObj.points[cont % pclObj.size()];
  try{
    point_camera.header = header;
    point_camera.point.x = point.x;
    point_camera.point.y = point.y;
    point_camera.point.z = point.z;
    listener.transformPoint("root",point_camera, point_root);
    tf_point.setOrigin( tf::Vector3(point_root.point.x,
    point_root.point.y, point_root.point.z) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    tf_point.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf_point,
    ros::Time::now(), point_root.header.frame_id, frame_name_min));
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
}

void publishTFObject(const pcl::PointCloud<pcl::PointXYZRGB> &pclObj_const, std_msgs::Header header)
{

  pcl::PointCloud<pcl::PointXYZRGB> pclObj;
  static tf::TransformListener listener;
  tf::StampedTransform tf_cloud;


  try{


ROS_INFO("Hey now.");
  listener.lookupTransform( "root", header.frame_id, ros::Time(0), tf_cloud);
  ROS_INFO("You are an all star");
  pcl_ros::transformPointCloud(pclObj_const, pclObj, tf_cloud);
  
  ROS_INFO("Put your game on");
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
    }
  pcl::PointXYZRGB min_pt, max_pt, centroid;
  //pcl::computeCentroid(pclObj, centroid);
  //pcl::getMinMax3D(pclObj, min_pt, max_pt);
  tf::StampedTransform stf;
  geometry_msgs::PointStamped pt_min, pt_max, pt_min2, pt_max2;
  geometry_msgs::PointStamped pt_cen, pt_cen2;
  
  try{
    header.frame_id = "root";
    pt_min.header = header;

    std::vector<pcl::PointXYZRGB> vecs_found;
    findHighest(pclObj, vecs_found);
    min_pt = vecs_found[0];
    max_pt = vecs_found[1];
    pt_min.point.x = min_pt.x;
    pt_min.point.y = min_pt.y;

    pt_min.point.z = min_pt.z;
    pt_max.header = header;
    pt_max.point.x = max_pt.x;
    pt_max.point.y = max_pt.y;

    pt_max.point.z = max_pt.z;// listener.lookupTransform("root", header.frame_id,ros::Time::now(), stf);
    pt_cen.header = header;
  /*  pt_cen.point.x = centroid.x;
    pt_cen.point.y = centroid.y;
    pt_cen.point.z = centroid.z;
    listener.transformPoint("root",pt_cen, pt_cen2);*/

    listener.transformPoint("root",pt_min, pt_min2);
    listener.transformPoint("root",pt_max, pt_max2);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

  static tf::TransformBroadcaster br;
  tf::Transform transform_min, transform_max, tf_head_centroid, tf_cent;
  transform_min.setOrigin( tf::Vector3(pt_min2.point.x,pt_min2.point.y, pt_min2.point.z) );
  transform_max.setOrigin( tf::Vector3(pt_max2.point.x,pt_max2.point.y, pt_max2.point.z) );
  //tf_cent.setOrigin(tf::Vector3(pt_cen2.point.x, pt_cen2.point.y, pt_cen2.point.z));

  float cx,cy;
  cx = (pt_max2.point.x + pt_min2.point.x)/2;
  cy = (pt_max2.point.y + pt_min2.point.y)/2;


  tf_head_centroid.setOrigin(tf::Vector3(cx, cy, pt_max2.point.z) );


  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform_min.setRotation(q);
  transform_max.setRotation(q);
  tf_head_centroid.setRotation(q);
  //tf_cent.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform_min,
   ros::Time::now(), pt_min2.header.frame_id, frame_name_min));
  br.sendTransform(tf::StampedTransform(transform_max,
   ros::Time::now(), pt_max2.header.frame_id, frame_name_max));
  br.sendTransform(tf::StampedTransform(tf_head_centroid,
   ros::Time::now(), pt_max2.header.frame_id, frame_name_head));
  //br.sendTransform(tf::StampedTransform(tf_cent,ros::Time::now(), pt_cen2.header.frame_id, frame_name_cent));
  
  


}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  bool cond = false;
  bool ejercicio_a = true;
  sensor_msgs::PointCloud2 output;
  
  if (!ejercicio_a) {
  pcl::PointCloud<pcl::PointXYZRGB> pclObj;
  pcl::fromROSMsg(*input, pclObj);
  for (auto k : pclObj) {
    if (k.g > 150 && k.r < 50 && k.b < 50 && !cond) {
      ROS_INFO("Veo algo muy verde");
      break;
    }
  }
  // Do data processing here...
  pcl::toROSMsg(pclObj, output);
  
  // Publish the data.
  pub.publish (output);
  } else {
      // Container for original & filtered data
      pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

      pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
      pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);

      // Convert to PCL data type
      pcl_conversions::toPCL( * input, * cloud );


        // Perform voxel grid downsampling filtering
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud (cloudPtr);
      sor.setLeafSize (0.01, 0.01, 0.01);
      sor.filter (* cloudFilteredPtr);



  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);

  
  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

// perform ransac planar filtration to remove table top
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
// Create the segmentation object
pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
// Optional
seg1.setOptimizeCoefficients (true);
// Mandatory
seg1.setModelType (pcl::SACMODEL_PLANE);
seg1.setMethodType (pcl::SAC_RANSAC);
seg1.setDistanceThreshold (0.04);

seg1.setInputCloud (xyzCloudPtr);
seg1.segment (*inliers, *coefficients);



  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtr);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);


// perform euclidean cluster segmentation to seporate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);



    pcl::PCLPointCloud2* outputPCL = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2* outputPCL2 = new pcl::PCLPointCloud2;
 // declare an instance of the SegmentedClustersArray message
 // obj_recognition::SegmentedClustersArray CloudClusters;
  long unsigned int cont = 0, contmax=0;

    double gr = 0, gb=0;
    bool cluster_copied=false;
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {

    // create a new clusterData message object
    //obj_recognition::ClusterData clusterData;


    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
      

    }


    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,* outputPCL);
    pcl::PointCloud<pcl::PointXYZRGB> pclObj;
    pcl::fromPCLPointCloud2(*outputPCL, pclObj);
    double rmed=0, gmed=0, bmed=0;
    for (auto k : pclObj) {
        rmed += k.r;
        gmed += k.b;
        bmed += k.g;
      }
      rmed /= pclObj.size();
      gmed /= pclObj.size();
      bmed /= pclObj.size();
      double aux1 = gmed - rmed;
      double aux2 = gmed - bmed;
      if (aux1 > 0 && aux2 > 0 &&
       gr*gr + gb*gb < aux1*aux1 + aux2*aux2)
        {
            gr = aux1;
            gb = aux2;
            pcl::copyPointCloud(* outputPCL, *outputPCL2);
            contmax = cont;
            cluster_copied = true;
        }
        ROS_INFO("Cluster %d, average: [%f,%f,%f] [%f, %f]",
        rmed, gmed, bmed,
         cont, gr, gb);
      cont++;
    }

    
    
    // Convert to ROS data type
    
    
    if (cluster_copied) {
          ROS_INFO("HERE IT IS %d", contmax);
        pcl_conversions::fromPCL(*outputPCL2, output);
        output.header = input->header;
        pub.publish(output);
      {
      pcl::PointCloud<pcl::PointXYZRGB> pclObj;
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pclObjPtr;
      pcl::fromPCLPointCloud2(*outputPCL2, pclObj);

      publishTFObject(pclObj, output.header);
      //publishTFs(pclObj, output.header, GLOBAL_COUNTER);

  GLOBAL_COUNTER++;
      }
    } else {
      ROS_INFO("Useful cluster not found.");
    }
    // add the cluster to the array message
    //clusterData.cluster = output;

}
    
  }


int main(int argc, char** argv) 
{
	// Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  //for (int i = 0; i < 10; i++) {
  //std::ostringstream stringStream;
  //stringStream << "output"<<i;
  //std::string copyOfStr = stringStream.str();
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  //}
  // Spin
  while (ros::ok())
  {
  ros::Rate(10).sleep();
  ros::spin ();
  }
	return 0;
}
