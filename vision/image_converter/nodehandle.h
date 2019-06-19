#include <cstdio>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <math.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <ros/package.h>
#include <omp.h>
#include "vision/color.h"

#define TEST "/home/shengru/catkin_ws/src/vision/1.bmp"
#define YAML_PATH ros::package::getPath("vision") + "/config/tb3.yaml"

using namespace cv;
using namespace std;

enum Color
{
    Red,
    Blue,
    Yellow,
    White,
    Black,
    None
};
class Object
{
  public:
    Object()
    {
        upleft = Point(0, 0);
        downright = Point(0, 0);
        center = Point(0, 0);
        offset = 0;
        distance = 9999;
        angle = 999;
        size = -1;
        radius = 0;
    }

    Point upleft;
    Point downright;
    Point center;
    Point dis_point;
    int offset;
    int distance;
    int size;
    int radius;
    double angle;
    Mat mask;
};

class NodeHandle
{
  public:
    NodeHandle();
    ~NodeHandle();
    //================center====================
    int CenterXMsg;
    int CenterYMsg;
    int CatchDistanceMsg;
	int SizeFilterMsg;
    //================color====================
    int HSV_mode;
    vector<int> HSV_red;
    vector<int> HSV_blue;
    vector<int> HSV_yellow;
    vector<int> HSV_white;
    vector<int> HSV_black;
    //========================================
    void pub_monitor(Mat Monitor);
    void pub_mask(Mat Mask);
    void pub_src(Mat Src);
    void pub_fps(double fps);
    void pub_object(Object red, Object blue, Object yellow, Object white, Object black);
    void pub_catch(Object red, Object blue, Object yellow, Object white, Object black);
	void pub_ball(Object red, Object blue, Object yellow, Object white, Object black);

  private:
    ros::NodeHandle nh;

    void read_yaml();
    void save_yaml();
    void get_param();
    //void default_param();

    //===============color===================
    ros::Subscriber color_sub;
    void colorcall(const vision::color msg);
    //==============save param===============
    ros::ServiceServer save_srv;
    bool savecall(std_srvs::Empty::Request &req,
                  std_srvs::Empty::Response &res);
    ros::ServiceServer connect_srv;
    bool connectcall(std_srvs::Empty::Request &req,
                 std_srvs::Empty::Response &res);
    //void savecall(const std_msgs::Int32 msg);
    //==============center====================
    ros::Subscriber center_sub;
    void centercall(const std_msgs::Int32MultiArray msg);
    //======================================
    ros::Publisher src_pub;
    ros::Publisher monitor_pub;
    ros::Publisher mask_pub;
    ros::Publisher fps_pub;
    ros::Publisher obj_pub;
    ros::Publisher catch_pub;
	ros::Publisher ball_pub;
};
