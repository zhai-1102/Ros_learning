// 1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"

/* 
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建静态坐标转换广播器
        4.创建坐标系信息
        5.广播器发布坐标系信息
        6.spin()
*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"static_pub");
    ros::NodeHandle nh;   //unnecessary
    //creating a publication object
    tf2_ros::StaticTransformBroadcaster pub;
    //organize the messages that are published
    geometry_msgs::TransformStamped tfs;
    //content
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "based_link";
    tfs.child_frame_id = "laser";
    tfs.transform.translation.x  = 0.2;
    tfs.transform.translation.y  = 0.2;
    tfs.transform.translation.z  = 0.5;
    tf2::Quaternion qtn;       //creat quaternion object
    qtn.setRPY(0,0,0);           // roll pitch yaw     (rad)
    //from euler to quaternion 
    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();
     //publish DATA
    pub.sendTransform(tfs);
    //spin()
    ros::spin();
    return 0;
}
