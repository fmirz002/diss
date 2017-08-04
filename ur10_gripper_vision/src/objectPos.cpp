#include <ros/ros.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/Point.h>

int flag;
ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;

double b_x_offset = -0.02;
double b_y_offset = -0.04;
double b_z_offset = 0.02;
double r_x_offset = -0.02;
double r_y_offset = -0.04;
double r_z_offset = 0.03;
double g_x_offset = -0.02;
double g_y_offset = -0.04;
double g_z_offset = 0.01;
double y_x_offset = -0.02;
double y_y_offset = -0.04;
double y_z_offset = 0.03;

geometry_msgs::Point obj_1;
geometry_msgs::Point obj_2;
geometry_msgs::Point obj_3;
geometry_msgs::Point obj_4;

void blue_duck(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Convert to PCL Pointcloud2 to Point XYZRGB
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    float x = 0;
    float y = 0;
    float z = 0;
    for(int i = 0; i < cloud->points.size(); ++i)
    {
        x += cloud->points.at(i).x;
        y += cloud->points.at(i).y;
        z += cloud->points.at(i).z;
    }
    x /= float(cloud->points.size());
    y /= float(cloud->points.size());
    z /= float(cloud->points.size());

    x += b_x_offset;
    y += b_y_offset;
    z += b_z_offset;
 
    obj_1.x = y;
    obj_1.y = -x;
    obj_1.z = z;
    
    pub1.publish(obj_1);

}

void red_ball(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Convert to PCL Pointcloud2 to Point XYZRGB
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    double x = 0;
    double y = 0;
    double z = 0;
    for(int i = 0; i < cloud->points.size(); ++i)
    {
        x += cloud->points.at(i).x;
        y += cloud->points.at(i).y;
        z += cloud->points.at(i).z;
    }
    x /= double(cloud->points.size());
    y /= double(cloud->points.size());
    z /= double(cloud->points.size());

    x += r_x_offset;
    y += r_y_offset;
    z += r_z_offset;
    
    obj_2.x = y;
    obj_2.y = -x;
    obj_2.z = z;
    
    pub2.publish(obj_2);
}

void green_block(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Convert to PCL Pointcloud2 to Point XYZRGB
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    double x = 0;
    double y = 0;
    double z = 0;
    for(int i = 0; i < cloud->points.size(); ++i)
    {
        x += cloud->points.at(i).x;
        y += cloud->points.at(i).y;
        z += cloud->points.at(i).z;
    }
    x /= double(cloud->points.size());
    y /= double(cloud->points.size());
    z /= double(cloud->points.size());

    x += g_x_offset;
    y += g_y_offset;
    z += g_z_offset;

    obj_3.x = y;
    obj_3.y = -x;
    obj_3.z = z;
    
    pub3.publish(obj_3);
}


void yellow_lego(const sensor_msgs::PointCloud2ConstPtr& input)
{
    //Convert to PCL Pointcloud2 to Point XYZRGB
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

    double x = 0;
    double y = 0;
    double z = 0;
    for(int i = 0; i < cloud->points.size(); ++i)
    {
        x += cloud->points.at(i).x;
        y += cloud->points.at(i).y;
        z += cloud->points.at(i).z;
    }
    x /= double(cloud->points.size());
    y /= double(cloud->points.size());
    z /= double(cloud->points.size());

    x += y_x_offset;
    y += y_y_offset;
    z += y_z_offset;
    
    obj_4.x = y;
    obj_4.y = -x;
    obj_4.z = z;
    
    pub4.publish(obj_4);
}

int main(int argc, char** argv)
{
    ros::init (argc, argv, "objectPos");
    ros::NodeHandle nh;

    ros::Subscriber sub_blue = nh.subscribe ("/segmented_blue", 1, blue_duck);
    ros::Subscriber sub_red = nh.subscribe ("/segmented_red", 1, red_ball);
    ros::Subscriber sub_green = nh.subscribe ("/segmented_green", 1, green_block);
    ros::Subscriber sub_yellow = nh.subscribe ("/segmented_yellow", 1, yellow_lego);
    
    pub1 = nh.advertise<geometry_msgs::Point>("blue_duck", 1);
    pub2 = nh.advertise<geometry_msgs::Point>("red_ball", 1);
    pub3 = nh.advertise<geometry_msgs::Point>("green_block", 1);
    pub4 = nh.advertise<geometry_msgs::Point>("yellow_lego", 1);

    ros::spin();
    return 0;
}
