#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;
int flag;

void segment(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Convert to PCL Pointcloud2 to Point XYZRGB
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for(int i = 0; i < cloud->points.size(); ++i)
    {
        int r = cloud->points.at(i).r;
        int g = cloud->points.at(i).g;
        int b = cloud->points.at(i).b;
        //Remove black points
        if(b > 75)
        {
            cloud->points.at(i).x = -20.0;
        }
	if(r < 100 || g < 100)
	{
           cloud->points.at(i).x = -20.0;
	}
    }

    //Filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-10, 10);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*color_filtered);

    if(color_filtered->points.size() == 0)
    {
        return;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (color_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
 
    //Convert back to PCL Pointcloud2 to Sensor_msg
    pcl::toPCLPointCloud2(*cloud_filtered, pcl_pc2);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(pcl_pc2, output);

    pub.publish(output);

}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "segmentYellow");
    ros::NodeHandle nh;

    flag = 1;
 
    ros::Subscriber sub1 = nh.subscribe ("/filtered_points", 1, segment);
    
    pub = nh.advertise<sensor_msgs::PointCloud2> ("segmented_yellow", 1);
    
    ros::spin();
    return 0;
}
