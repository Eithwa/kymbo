#include "image_converter.hpp"

#define test "/home/shengru/catkin_ws/src/vision/1.bmp"

//cv::Mat img = cv::imread("/home/shengru/catkin_ws/src/vision/2.bmp",CV_LOAD_IMAGE_COLOR);
Vision::Vision(){
	//image_sub = nh.subscribe("/raspicam_node/image/compressed", 1,&Vision::imageCb,this);
	image_sub = nh.subscribe("/usb_cam/image_raw", 1,&Vision::imageCb,this);
}
Vision::~Vision(){
	main_image.release();
	destroyWindow("view"); 
}
void Vision::imageCb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat

		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //convert image data
		//cv::flip(cv_ptr->image, cv_ptr->image, 1); // reverse image
		main_image = cv_ptr->image.clone();
		//cv::imshow("view", main_image);
		//cv::waitKey(10);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert to image!");
	}
}
cv::Mat Vision::get_image(){
	//ros::spin();
	Mat input = main_image;
	if(main_image.empty()){
		cout<<"no frame\n";
	}
  return main_image;
}	
