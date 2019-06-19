#include "image_converter.hpp"

int main(int argc,char **argv){
  
	ros::init(argc, argv, "positioningaid");
	
	Vision rpicamera;
	//Object test;
    
    
	while(ros::ok()){
	
		rpicamera.show_image();
		//test.show_mask();
        //ros::spinOnce();
		//loop_rate.sleep();    
	}
	ros::shutdown();	
	//cv::destroyWindow("view");

	return 0;
}
