// https://blog.csdn.net/Jeff_zjf/article/details/124255916

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <iostream>
using namespace std;
 
ros::Publisher pub_point_cloud2;
 
bool is_K_empty = 1;
double K[9];
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
 
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Read depth map
    int height = img_msg->height;
    int width = img_msg->width;
    // data converting...unit: mm
    unsigned short *depth_data = (unsigned short*)&img_msg->data[0];
    
    // Step2: depth map to point cloud
    sensor_msgs::PointCloud2 point_cloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int uy=0; uy<height; uy++)
    {
        for(int ux=0; ux<width; ux++)
        {
            float x, y, z;
            z = *(depth_data + uy*width + ux) / 1000.0;
            if(z!=0)
            {
                x = z * (ux - K[2]) / K[0];
                y = z * (uy - K[5]) / K[4];
                pcl::PointXYZ p(x, y, z);
                cloud->push_back(p);
            }
        }
    }
    // Pub to topic
    pcl::toROSMsg(*cloud, point_cloud2);
    point_cloud2.header.frame_id = "camera_depth_frame";
    pub_point_cloud2.publish(point_cloud2);
}
 
 
void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info_msg)
{
    // get camera info
    if(is_K_empty)
    {
        for(int i=0; i<9; i++)
        {
            K[i] = camera_info_msg->K[i];
        }
        is_K_empty = 0;
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_node");
    ros::NodeHandle n;

    ros::Subscriber sub_img = n.subscribe("/camera/depth/image_rect_raw", 100, img_callback);
    // subscribe to the camera's parameters
    ros::Subscriber sub_cmara_info = n.subscribe("/camera/depth/camera_info", 1, camera_info_callback);
    pub_point_cloud2 = n.advertise<sensor_msgs::PointCloud2>("/rs_point_cloud", 1000);
    
    ROS_INFO("Runing ...");
    ros::spin();
    return 0;
}