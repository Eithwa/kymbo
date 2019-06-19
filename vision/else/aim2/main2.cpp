#include "image_converter2.hpp"

void SigintHandler(int sig){
	cv::destroyAllWindows();
	ROS_INFO("shutting down!");
	ros::shutdown();
}

int main(int argc,char **argv){
	ros::init(argc, argv, "aim2");
	ros::NodeHandle h_node;
	signal(SIGINT, SigintHandler);

	NodeHandle Node;
	Vision usb_cam;
	Object Black,Red,Blue,Yellow,White;

	while(ros::ok()){
		Mat source;
		Mat frame;
		Mat monitor;
		
		source = usb_cam.get_image();
		usb_cam.release();
		Black.Reset();
		Red.Reset();
		Blue.Reset();
		Yellow.Reset();
		White.Reset();
		
		if(usb_cam.update&&!source.empty()){

			double scale = 0.5;
			Size dsize = Size(source.cols*scale, source.rows*scale);		
			frame = Mat(dsize, CV_32S);
			resize(source,frame,dsize);
			monitor = frame.clone();

			if(Node.Parameter_Update){
				cout<<"Parameter_Update\n";
		
				Red.Get_HSV(Node.Sent_HSV(0));
				Blue.Get_HSV(Node.Sent_HSV(1));
				Yellow.Get_HSV(Node.Sent_HSV(2));
				//White.Get_HSV(Node.Sent_HSV(3));
				//Black.Get_HSV(Node.Sent_HSV(4));
				Node.Parameter_Update = false;
			}
					                       
			//usb_cam.Mask.clear();
			usb_cam.Mask.push_back(Red.ColorModel(frame));
			usb_cam.Mask.push_back(Blue.ColorModel(frame));
			usb_cam.Mask.push_back(Yellow.ColorModel(frame));
			//usb_cam.Mask.push_back(White.ColorModel(frame));
			//usb_cam.Mask.push_back(Black.ColorModel(frame));

	
			Red.Monitor(monitor);
			Blue.Monitor(monitor);
			Yellow.Monitor(monitor);
			//White.Monitor(monitor);
			//Black.Monitor(monitor);

			usb_cam.Monitor=monitor;

			usb_cam.Pub(monitor,Red,Blue,Yellow,White,Black);
			usb_cam.Pubvision();
			usb_cam.Pubmask();
			//imshow("mask2",usb_cam.Mask[usb_cam.maskindex]);
			//imshow("monitor2",monitor);
			//cv::waitKey(1);
			usb_cam.update = false;
		}
		ros::spinOnce();
	}
	ROS_INFO("Node exit");
	printf("Process exit\n");	
	return 0;
}
