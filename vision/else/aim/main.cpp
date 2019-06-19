#include "image_converter.h"

Mat CutFrame(Mat frame, int upx, int upy, int downx, int downy){
	Mat output(downy-upy, downx-upx, CV_8UC3);
	for(int i = 0 ; i < downy - upy; i++){
		for(int j = 0; j< downx - upx; j++){
			output.data[(i * output.cols * 3) + (j * 3) + 0] = frame.data[((i+upy) * frame.cols * 3) + ((j+upx) * 3) + 0];
			output.data[(i * output.cols * 3) + (j * 3) + 1] = frame.data[((i+upy) * frame.cols * 3) + ((j+upx) * 3) + 1];
			output.data[(i * output.cols * 3) + (j * 3) + 2] = frame.data[((i+upy) * frame.cols * 3) + ((j+upx) * 3) + 2];
		}	
	}
	return output;
}
int main(int argc,char **argv){
	ros::init(argc, argv, "aim");
	
	NodeHandle Node;

	Vision usb_cam;
	Object Black,Red,Blue,Yellow,White;

	int frame_counter = 0;
	double ALPHA = 0.5;
	double dt;
	long int EndTime;

	Mat source;
	Mat frame;
	Mat monitor;
	Mat mblack,mred,mblue,myellow,mwhite;
	while(ros::ok()){
/////////////////////////////
//fps
/*
		frame_counter++;
		//static long int StartTime = time(NULL);//ros::Time::now().toNSec();
		static long int StartTime = ros::Time::now().toNSec();
		//static long int EndTime;
		static long double FrameRate = 0.0;

		if (frame_counter == 10) {
			EndTime = ros::Time::now().toNSec();
			dt = (EndTime - StartTime) / frame_counter;
			StartTime = EndTime;
			if ( dt != 0 )
			{
				FrameRate = ( 1000000000.0 / dt ) * ALPHA + FrameRate * ( 1.0 - ALPHA );
				cout << "rate: " << FrameRate << " Hz"<<endl;
			}
			frame_counter = 0;
		}
*/
////////////////////////////
		source = usb_cam.get_image();
		//frame = usb_cam.get_image();
		//Mat test222=imread("/home/aa/wrs_ws/src/vision/aim/1.jpg");

		Black.Reset();
		Red.Reset();
		Blue.Reset();
		Yellow.Reset();
		White.Reset();
		
		if(usb_cam.update&&!source.empty()){

			double scale = 0.5;
			Size dsize = Size(source.cols*scale, source.rows*scale);		
			//Mat test3 = Mat(dsize, CV_32S);		
			//monitor = frame.clone();
			Mat test;
			frame = Mat(dsize, CV_32S);
			resize(source,frame,dsize);
			frame = CutFrame(frame, 0, frame.rows*0.4, frame.cols, frame.rows);
			//cout<<"!!!!!!!!!!!!!!"<<endl;
			monitor = frame.clone();
			//imshow("2",monitor);
			//imshow("1",frame);


			if(Node.Get_Param){
				cout<<"Get_Param\n";
				
				//cout<<Ball.HSVrange[0]<<endl;
				Red.Get_HSV(Node.Sent_HSV(0));
				Blue.Get_HSV(Node.Sent_HSV(1));
				Yellow.Get_HSV(Node.Sent_HSV(2));
				//White.Get_HSV(Node.Sent_HSV(3));
				//Black.Get_HSV(Node.Sent_HSV(4));
				Node.Get_Param = false;
			}
			                               
			usb_cam.Mask.clear();
			usb_cam.Mask.push_back(Red.ColorModel(frame));
			usb_cam.Mask.push_back(Blue.ColorModel(frame));
			usb_cam.Mask.push_back(Yellow.ColorModel(frame));
			//usb_cam.Mask.push_back(White.ColorModel(frame));
			//usb_cam.Mask.push_back(Black.ColorModel(frame));
/*
			mred = Red.ColorModel(frame);
			mblue = Blue.ColorModel(frame);
			myellow = Yellow.ColorModel(frame);
			mwhite = White.ColorModel(frame);
			mblack = Black.ColorModel(frame);

			imshow("mblack",mblack);
			imshow("mred",mred);
			imshow("mblue",mblue);
			imshow("myellow",myellow);
			imshow("mwhite",mwhite);
	*/		
			
			Red.Monitor(monitor);
			Blue.Monitor(monitor);
			Yellow.Monitor(monitor);
			//White.Monitor(monitor);
			//Black.Monitor(monitor);
	
			usb_cam.Monitor=monitor;

			usb_cam.Pub(monitor,Red,Blue,Yellow,White,Black);
			usb_cam.Pubvision();
			usb_cam.Pubmask();
			//imshow("mask",usb_cam.Mask[usb_cam.maskindex]);
			imshow("monitor",monitor);
			cv::waitKey(1);
			usb_cam.update = false;
		}
		ros::spinOnce();
	}
	ros::shutdown();	
	cv::destroyWindow("view");
	
	return 0;
}
