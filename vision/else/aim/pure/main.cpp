#include "image_converter.hpp"

int main(int argc,char **argv){
  
	ros::init(argc, argv, "positioningaid");
	
	Vision usb_cam;
    Mat frame;
	while(ros::ok()){
		frame = usb_cam.get_image();
		if(!frame.empty()){
			imshow("test",usb_cam.get_image());
			cv::waitKey(10);
		}
		ros::spinOnce();
	}
ros::spin();
	ros::shutdown();	
	cv::destroyWindow("view");
	
	return 0;
}
