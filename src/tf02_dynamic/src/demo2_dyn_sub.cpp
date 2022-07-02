#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件
/*  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin()
*/
int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"dynamic_sub");
    ros::NodeHandle nh;              //necessary
    // 3.创建 TF 订阅节点
    tf2_ros::Buffer buffer;      //cache the sub.information
    tf2_ros::TransformListener listener(buffer);

    //ros::Duration(2).sleep();
    ros::Rate r(10);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "turtle1";
        point_laser.header.stamp = ros::Time(0.0);
        point_laser.point.x = 2.0;
        point_laser.point.y = 3.0;
        point_laser.point.z = 5.0;
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"world");
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是: %s",point_base.point.x,
                                                                                                                                        point_base.point.y,
                                                                                                                                        point_base.point.z,
                                                                                                                                        point_base.header.frame_id.c_str()
                                                                                                                                        );

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }

        r.sleep();  
        ros::spinOnce();
    }


    return 0;
}
