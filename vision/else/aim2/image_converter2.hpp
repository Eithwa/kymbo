#include <cstdio>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <highgui.h>
#include <math.h>
#include <signal.h>
#include <std_msgs/Int32.h>
#include "vision/bin.h"
#include "vision/aim2.h"
#define VISION_TOPIC "/usb_cam/image_raw"
//define VISION_TOPIC "/raspicam_node/image/compressed"
#define test "/home/shengru/catkin_ws/src/vision/1.bmp"
//#include <sensor_msgs/CompressedImage.h>

using namespace cv;
using namespace std;

class NodeHandle
{
public:
	NodeHandle();
	bool Parameter_Update;
	vector<vector<int> > Object_HSV;
	vector<int> Sent_HSV(int index);
	int center;
	int ang_para;
	int height;
	int scale;
	Mat Monitor;
	vector<int> Setting;
	vector<Mat> Mask;

private:
	ros::NodeHandle nh;
	ros::Subscriber param_sub;

	ros::Subscriber setting_sub;
	



	void Sub_param(const std_msgs::Int32 msg);
	
	

};
class Object: NodeHandle
{
public:
	Object();
	void Reset();
	string name;
	int index;
	double angle;
	int offset;
	cv::Point2f _center;
	vector<int> HSV;
	int extLeft,extRight,extTop,extBot;
	Point distance_point;
	float distance;
	float area;
	Mat mask;
	vector<Vec3f> circles;
	void Get_HSV(vector<int> HSV);
	Mat ColorModel(Mat frame);
	Mat Monitor(Mat Frame);
private:
	vector<int> HSVrange;	
	vector<vector<Point> > contours;
	int largest_contour_index;
	void Findoffset(int center);
	void FindAngle(Mat frame);
	void FindDistance(Mat frame);
	void FindContours(Mat frame, Mat mask);
	Point FindCenter();
	
	
	void calcCircles(const Mat &input, vector<Vec3f> &circles);
	void drawCircle(Mat &input, const vector<Vec3f> &circles);
};
class Vision
{
public:

	Vision();
	Vision(string sub);
	~Vision();
	int maskindex;
	void release();
	cv::Mat get_image();
	cv::Mat Original_image;
	void Pub(Mat frame,Object red,Object blue,Object yellow, Object white, Object black);
	void Sub_MaskIndex(const std_msgs::Int32 msg);
	bool update;
	Mat Monitor;
	vector<Mat> Mask;
	void Pubvision();
	void Pubmask();
private:
	ros::NodeHandle nh;
	ros::Subscriber image_sub;
	ros::Publisher aim2_pub;
	ros::Publisher monitor_pub;
	ros::Publisher mask_pub;
	ros::Subscriber mode_sub;
	void imageCb(const sensor_msgs::CompressedImageConstPtr& msg);
	//void imageCb(const sensor_msgs::ImageConstPtr& msg);
	Mat CutFrame(Mat frame, int upx, int upy, int downx, int downy);
	cv::Mat main_image;
};
