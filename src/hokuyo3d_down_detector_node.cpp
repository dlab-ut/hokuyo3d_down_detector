#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
class Hokuyo3dDownDetector
{
private:
    ros::Subscriber hokuyo_sub;
    ros::Publisher hokuyo_pub;
    ros::Publisher status_pub;
    ros::Time prev_time;
    sensor_msgs::PointCloud2 tmp_msg;
    std_msgs::Bool status;

    void initializePointCloud();
    void hokuyoSubCB(const sensor_msgs::PointCloud2::ConstPtr msg);

public:
    void mainLoop();
};

void Hokuyo3dDownDetector::initializePointCloud()
{
    for (auto point : tmp_msg.data)
    {
        point = 0.0f;
    }
    tmp_msg.header.stamp = ros::Time::now();
    tmp_msg.header.seq = tmp_msg.header.seq >= 0 ? tmp_msg.header.seq + 1 : 0;
}

void Hokuyo3dDownDetector::hokuyoSubCB(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    prev_time = msg->header.stamp;
    tmp_msg = *msg;
}

void Hokuyo3dDownDetector::mainLoop()
{
    ros::Rate r(10);
    ros::NodeHandle nh("");
    hokuyo_sub = nh.subscribe("hokuyo3d", 10, &hokuyoSubCB, this);
    hokuyo_pub = nh.advertise<sensor_msgs::PointCloud2>("hokuyo3d_checked", 1);
    status_pub = nh.advertise<std_msgs::Bool>("hokuyo3d_status", 1);
    prev_time = ros::Time::now();
    while (ros::ok())
    {
        if ((ros::Time::now() - prev_time) > ros::Duration(0.2))
        {
            initializePointCloud();
            status.data = false;
            hokuyo_pub.publish(tmp_msg);
            status_pub.publish(status);
        }
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hokuyo3d_down_detector");
    Hokuyo3dDownDetector node;
    node.mainLoop();

    return 0;
}