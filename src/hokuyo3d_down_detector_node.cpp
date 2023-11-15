#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
class Hokuyo3dDownDetector
{
private:
    ros::Subscriber hokuyo_sub;
    ros::Publisher hokuyo_pub;
    ros::Publisher status_pub;
    ros::Time prev_time;
    sensor_msgs::PointCloud2 tmp_msg;

    void initializePointCloud();
    void fillPointCloudWith0();
    void setPointCloud(sensor_msgs::PointCloud2 msg);
};

void Hokuyo3dDownDetector::initializePointCloud()
{

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hokuyo3d_down_detector");

    ros::Rate r(10);
    while (ros::ok())
    {

        r.sleep();
    }
    return 0;
}