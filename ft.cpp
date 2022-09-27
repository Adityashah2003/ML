#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub;
ros::Publisher markerpub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 output;
    // std::cout << "publishing\n";
    pcl_conversions::toPCL(*input, output);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterring_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_clusterring_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    new_clusterring_cloud->header.frame_id = "/velodyne_left";
    pcl::PCDWriter writer;

    pcl::fromPCLPointCloud2(output, *pcl_cloud);
    sensor_msgs::PointCloud2 converted_cloud;

    // Ransac
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pcl_cloud);
    sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*cloud_filtered);

    // segmentation_object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.2);
    seg.setAxis(Eigen::Vector3f(1.0, 0.0, 0.0));
    seg.setEpsAngle(20 / 57.2958);
    seg.setMaxIterations(1000);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl_conversions::fromPCL(*coefficients, ros_coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> neg_extract;
    neg_extract.setInputCloud(cloud_filtered);
    neg_extract.setIndices(inliers);
    neg_extract.setNegative(true);
    neg_extract.filter(*clusterring_cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(clusterring_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.2);
    ec.setMinClusterSize(25);
    ec.setMaxClusterSize(2000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(clusterring_cloud);
    ec.extract(cluster_indices);

    std::vector<pcl::PointIndices>::const_iterator it;
    std::vector<int>::const_iterator pit;

    for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        for (pit = it->indices.begin(); pit != it->indices.end(); pit++)
        {
            cloud_cluster->points.push_back(clusterring_cloud->points[*pit]);
        }
        Eigen::Vector4f minPt, maxPt;
        Eigen::Vector4f centroid;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
        pcl::compute3DCentroid(*cloud_cluster, centroid);
        // std::vector<int>::const_iterator markerarr;
        visualization_msgs::Marker box;
        visualization_msgs::Marker CUBE;
        visualization_msgs ::MarkerArray markerarr;
        box.header.stamp = ros::Time();
        // box.ns = "my_namespace";
        box.lifetime = ros::Duration(0.3);
        box.header.frame_id = "/velodyne_left";
        box.action = visualization_msgs::Marker::ADD;
        box.scale.x = (maxPt[0] - minPt[0]);
        box.scale.y = (maxPt[1] - minPt[1]);
        box.scale.z = (maxPt[2] - minPt[2]);
        box.pose.position.x = centroid[0];
        box.pose.position.y = centroid[1];
        box.pose.position.z = centroid[2];
        box.pose.orientation.x = 0;
        box.pose.orientation.y = 0;
        box.pose.orientation.z = 0;
        box.pose.orientation.w = 1.0;
        box.color.r = 0;
        box.color.g = 1;
        box.color.b = 0;
        box.color.a = 1;
        // markerarr-> points.push_back(box);// markerpub.publish(box);
        *new_clusterring_cloud += *cloud_cluster;
    }

    pcl::toROSMsg(*new_clusterring_cloud, converted_cloud);
    pub.publish(converted_cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PCL");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/hdl32e_left/velodyne_points", 1, cloud_cb);
    // ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("outpub", 1);
    // markerpub = nh.advertise<visualization_msgs::Marker>("out", 1);
    ros::spin();
}