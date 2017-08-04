#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/io/pcd_io.h>

ros::Publisher pub;
geometry_msgs::TransformStamped transform1, transform2, transform3, transform4;
sensor_msgs::PointCloud2 output1, output2, output3, output4;

double z_min = -0.05;
double z_max = 0.10;
double y_min = -0.90 ;
double y_max = 1.00;
double x_min = -0.90;
double x_max = 1.00;


void filter_1(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Apply Transform

    sensor_msgs::PointCloud2 cloud_trans;
    tf2::doTransform (*input, cloud_trans, transform1);

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtrX(cloud);
    pcl::PCLPointCloud2* cloud_x_filtered = new pcl::PCLPointCloud2;

    //Convert msg to PCL
    pcl_conversions::toPCL(cloud_trans, *cloud);

    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> passX;
    passX.setInputCloud (cloudPtrX);
    passX.setFilterFieldName("x");
    passX.setFilterLimits (x_min, x_max);
    passX.filter (*cloud_x_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrY(cloud_x_filtered);
    pcl::PCLPointCloud2* cloud_y_filtered = new pcl::PCLPointCloud2;

    pcl::PassThrough<pcl::PCLPointCloud2> passY;
    passY.setInputCloud (cloudPtrY);
    passY.setFilterFieldName("y");
    passY.setFilterLimits (y_min, y_max);
    passY.filter (*cloud_y_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrZ(cloud_y_filtered);
    pcl::PCLPointCloud2* cloud_z_filtered = new pcl::PCLPointCloud2;
    
    pcl::PassThrough<pcl::PCLPointCloud2> passZ;
    passZ.setInputCloud (cloudPtrZ);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits (z_min, z_max);
    passZ.filter (*cloud_z_filtered);

    //Convert PCL to msg  
    pcl_conversions::fromPCL(*cloud_z_filtered, output1);
    
    sensor_msgs::PointCloud2 combined1, combined2, combined;
    pcl::concatenatePointCloud(output1, output2, combined1); 
    pcl::concatenatePointCloud(output3, output4, combined2); 
    pcl::concatenatePointCloud(combined1, combined2, combined); 

    pub.publish (combined);
}

void filter_2(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Apply Transform

    sensor_msgs::PointCloud2 cloud_trans;
    tf2::doTransform (*input, cloud_trans, transform2);

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtrX(cloud);
    pcl::PCLPointCloud2* cloud_x_filtered = new pcl::PCLPointCloud2;

    //Convert msg to PCL
    pcl_conversions::toPCL(cloud_trans, *cloud);

    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> passX;
    passX.setInputCloud (cloudPtrX);
    passX.setFilterFieldName("x");
    passX.setFilterLimits (x_min, x_max);
    passX.filter (*cloud_x_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrY(cloud_x_filtered);
    pcl::PCLPointCloud2* cloud_y_filtered = new pcl::PCLPointCloud2;

    pcl::PassThrough<pcl::PCLPointCloud2> passY;
    passY.setInputCloud (cloudPtrY);
    passY.setFilterFieldName("y");
    passY.setFilterLimits (y_min, y_max);
    passY.filter (*cloud_y_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrZ(cloud_y_filtered);
    pcl::PCLPointCloud2* cloud_z_filtered = new pcl::PCLPointCloud2;
    
    pcl::PassThrough<pcl::PCLPointCloud2> passZ;
    passZ.setInputCloud (cloudPtrZ);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits (z_min, z_max);
    passZ.filter (*cloud_z_filtered);

    //Convert PCL to msg  
    pcl_conversions::fromPCL(*cloud_z_filtered, output2);
    
    sensor_msgs::PointCloud2 combined1, combined2, combined;
    pcl::concatenatePointCloud(output1, output2, combined1); 
    pcl::concatenatePointCloud(output3, output4, combined2); 
    pcl::concatenatePointCloud(combined1, combined2, combined); 

    pub.publish (combined);
}

void filter_3(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Apply Transform

    sensor_msgs::PointCloud2 cloud_trans;
    tf2::doTransform (*input, cloud_trans, transform3);

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtrX(cloud);
    pcl::PCLPointCloud2* cloud_x_filtered = new pcl::PCLPointCloud2;

    //Convert msg to PCL
    pcl_conversions::toPCL(cloud_trans, *cloud);

    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> passX;
    passX.setInputCloud (cloudPtrX);
    passX.setFilterFieldName("x");
    passX.setFilterLimits (x_min, x_max);
    passX.filter (*cloud_x_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrY(cloud_x_filtered);
    pcl::PCLPointCloud2* cloud_y_filtered = new pcl::PCLPointCloud2;

    pcl::PassThrough<pcl::PCLPointCloud2> passY;
    passY.setInputCloud (cloudPtrY);
    passY.setFilterFieldName("y");
    passY.setFilterLimits (y_min, y_max);
    passY.filter (*cloud_y_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrZ(cloud_y_filtered);
    pcl::PCLPointCloud2* cloud_z_filtered = new pcl::PCLPointCloud2;
    
    pcl::PassThrough<pcl::PCLPointCloud2> passZ;
    passZ.setInputCloud (cloudPtrZ);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits (z_min, z_max);
    passZ.filter (*cloud_z_filtered);

    //Convert PCL to msg  
    pcl_conversions::fromPCL(*cloud_z_filtered, output3);
    
    sensor_msgs::PointCloud2 combined1, combined2, combined;
    pcl::concatenatePointCloud(output1, output2, combined1); 
    pcl::concatenatePointCloud(output3, output4, combined2); 
    pcl::concatenatePointCloud(combined1, combined2, combined); 

    pub.publish (combined);
}

void filter_4(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Apply Transform

    sensor_msgs::PointCloud2 cloud_trans;
    tf2::doTransform (*input, cloud_trans, transform4);

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtrX(cloud);
    pcl::PCLPointCloud2* cloud_x_filtered = new pcl::PCLPointCloud2;

    //Convert msg to PCL
    pcl_conversions::toPCL(cloud_trans, *cloud);

    // Perform the actual filtering
    pcl::PassThrough<pcl::PCLPointCloud2> passX;
    passX.setInputCloud (cloudPtrX);
    passX.setFilterFieldName("x");
    passX.setFilterLimits (x_min, x_max);
    passX.filter (*cloud_x_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrY(cloud_x_filtered);
    pcl::PCLPointCloud2* cloud_y_filtered = new pcl::PCLPointCloud2;

    pcl::PassThrough<pcl::PCLPointCloud2> passY;
    passY.setInputCloud (cloudPtrY);
    passY.setFilterFieldName("y");
    passY.setFilterLimits (y_min, y_max);
    passY.filter (*cloud_y_filtered);
    
    pcl::PCLPointCloud2ConstPtr cloudPtrZ(cloud_y_filtered);
    pcl::PCLPointCloud2* cloud_z_filtered = new pcl::PCLPointCloud2;
    
    pcl::PassThrough<pcl::PCLPointCloud2> passZ;
    passZ.setInputCloud (cloudPtrZ);
    passZ.setFilterFieldName("z");
    passZ.setFilterLimits (z_min, z_max);
    passZ.filter (*cloud_z_filtered);

    //Convert PCL to msg  
    pcl_conversions::fromPCL(*cloud_z_filtered, output4);
    
    sensor_msgs::PointCloud2 combined1, combined2, combined;
    pcl::concatenatePointCloud(output1, output2, combined1); 
    pcl::concatenatePointCloud(output3, output4, combined2); 
    pcl::concatenatePointCloud(combined1, combined2, combined); 

    pub.publish (combined);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "filterPoints");
    ros::NodeHandle nh;
 
    ros::Subscriber sub1 = nh.subscribe ("/kinect_1/sd/points", 1, filter_1);
    ros::Subscriber sub2 = nh.subscribe ("/kinect_2/sd/points", 1, filter_2);
    //ros::Subscriber sub3 = nh.subscribe ("/kinect_3/sd/points", 1, filter_3);
    ros::Subscriber sub4 = nh.subscribe ("/kinect_4/sd/points", 1, filter_4);
    
    pub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_points", 1);
    
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);    
    transform1 = tfBuffer.lookupTransform("map", "kinect_1_ir_optical_frame", ros::Time(0), ros::Duration(1));
    transform2 = tfBuffer.lookupTransform("map", "kinect_2_ir_optical_frame", ros::Time(0), ros::Duration(1));
    transform3 = tfBuffer.lookupTransform("map", "kinect_3_ir_optical_frame", ros::Time(0), ros::Duration(1));
    transform4 = tfBuffer.lookupTransform("map", "kinect_4_ir_optical_frame", ros::Time(0), ros::Duration(1));

    ros::spin();
    return 0;
}
