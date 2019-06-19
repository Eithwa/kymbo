#include "image_converter.h"

#define test "/home/shengru/catkin_ws/src/vision/1.bmp"

NodeHandle::NodeHandle(){
	param_sub = nh.subscribe("/tb3/save", 1,&NodeHandle::Set_Param,this);
	
	Get_Param = false;
}
void NodeHandle::Set_Param(const std_msgs::Int32 msg){
	string HSVParam[5]={"/tb3/HSV/Red",
			"/tb3/HSV/Blue",
			"/tb3/HSV/Yellow",
			"/tb3/HSV/White",
			"/tb3/HSV/Black"
			};
	vector<int> HSV;
	vector<int> Set;
	Object_HSV.clear();
	//Setting.clear();
	for(int i=0; i<sizeof(HSVParam)/sizeof(string); i++){
		nh.getParam(HSVParam[i], HSV);
		Object_HSV.push_back(HSV);
	}
	nh.getParam("/tb3/vision_param",Set);
	//if(Set.size())cout<<Set[1]<<endl;
	Setting=Set;
	
	Get_Param = true;
}

	
vector<int> NodeHandle::Sent_HSV(int index){
	vector<int> HSV;
	if(Object_HSV.size()){
		HSV = Object_HSV[index];
	}
	return HSV;
}

Vision::Vision(){
	image_sub = nh.subscribe(VISION_TOPIC, 1,&Vision::imageCb,this);
	mode_sub = nh.subscribe("/tb3/VideoMode", 1,&Vision::MaskIndex,this);
	
	aim_pub = nh.advertise<vision::aim>("/tb3/ball", 1);
	mask_pub = nh.advertise<sensor_msgs::Image>("/tb3/mask",1);
	monitor_pub = nh.advertise<sensor_msgs::Image>("/tb3/monitor",1);
	maskindex = 0;
}
Vision::Vision(string sub){
	image_sub = nh.subscribe(sub, 1,&Vision::imageCb,this);
}
Vision::~Vision(){
	main_image.release();
	destroyWindow("view"); 
}
void Vision::MaskIndex(const std_msgs::Int32 msg){
	maskindex=msg.data;

}
	//int frame_counter = 0;
	//const double ALPHA = 0.5;
	//double dt;
	//long int EndTime;

void Vision::imageCb(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
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
//////////////
	update = true;
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
///////////////////////////////
}
cv::Mat Vision::get_image(){
	//ros::spin();
	Mat input = main_image.clone();
	if(main_image.empty()){
		//cout<<"no frame\n";
	}
	return input;
}	
void Vision::Pub(Mat frame,Object red,Object blue,Object yellow, Object white, Object black){
	int x = frame.cols;
	vector<string> color;
	vector<float> dis;
	vector<float> ang;
	vector<float> area; 
	vision::aim aim_msg;
	
	
	if(red._center.x!=0&red._center.y!=0){
		color.push_back("red");
		dis.push_back(red.distance);
		ang.push_back(red.angle);
//cout<<red.angle<<endl;
		area.push_back(red.area);
	}	
	if(blue._center.x!=0&blue._center.y!=0){
		color.push_back("blue");
		dis.push_back(blue.distance);
		ang.push_back(blue.angle);
		area.push_back(blue.area);
		//aim_msg.Blueaim =  blue.angle;
	}	
	if(yellow._center.x!=0&yellow._center.y!=0){
		color.push_back("yellow");
		dis.push_back(yellow.distance);
		ang.push_back(yellow.angle);
		area.push_back(yellow.area);
		//aim_msg.Yellowaim =  yellow.angle;
	}	
	if(white._center.x!=0&white._center.y!=0){
		//aim_msg.Whiteaim =  white.angle;
		color.push_back("white");
		dis.push_back(white.distance);
		ang.push_back(white.angle);
		area.push_back(white.area);
	}	
	if(black._center.x!=0&black._center.y!=0){
		//aim_msg.Whiteaim =  white.angle;
		color.push_back("black");
		dis.push_back(black.distance);
		ang.push_back(black.angle);
		area.push_back(black.area);
	}
	aim_msg.color = color;
	aim_msg.dis =  dis;
	aim_msg.ang = ang;
	aim_msg.area = area;
	aim_pub.publish(aim_msg);

}
void Vision::Pubvision(){
    sensor_msgs::ImagePtr monitormsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Monitor ).toImageMsg();
	monitor_pub.publish(monitormsg);
}
void Vision::Pubmask(){
	if(Mask.size()){
		sensor_msgs::ImagePtr maskmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", Mask[maskindex] ).toImageMsg();
		mask_pub.publish(maskmsg);
	}
}
Object::Object(){
	cv::Point2f _point = Point(999,999);
	//int _angle = 999;
}

Mat Object::Monitor(Mat frame){
	std::string x;
	std::stringstream x_out;
	std::string ang;
	std::stringstream ang_out;
	std::string dis;
	std::stringstream dis_out;
	std::string area;
	std::stringstream area_out;

	offset = 999;
	mask = ColorModel(frame);
	FindContours(frame,mask);
	_center = FindCenter();
	Findoffset(frame.cols/2);
	FindAngle(frame);
	FindDistance(frame);
	//cout<<offset<<endl;
	x_out << offset;
	x = x_out.str();
	ang_out << angle;
	ang = ang_out.str();
	dis_out << distance;
	dis = dis_out.str();
	area_out << area;
	area = area_out.str();
	
	cv::putText(frame, "off"+x , Point(_center.x,_center.y-80), 0, 0.5,Scalar(0, 0, 0), 1);
	cv::putText(frame, "ang"+ang , Point(_center.x,_center.y-60), 0, 0.5,Scalar(0, 0, 0), 1);
	cv::putText(frame, "dis"+dis ,  Point(_center.x,_center.y-40), 0, 0.5,Scalar(0, 0, 0), 1);
	cv::putText(frame, "area"+area , Point(_center.x,_center.y-20), 0, 0.5,Scalar(0, 0, 0), 1);
	line(frame, _center, _center, Scalar(0,0,255), 3);
	return frame;
}
void Object::Reset(){
	_center = Point(0,0);
	offset = 999;
}

void Object::Get_HSV(vector<int> HSV){
	HSVrange = HSV;
}

Mat Object::ColorModel(Mat frame){
	Mat inputMat = frame.clone();
	Mat hsv(inputMat.rows, inputMat.cols,  CV_8UC3, Scalar(0,0,0));
	Mat mask(inputMat.rows, inputMat.cols,  CV_8UC1, Scalar(0,0,0));
	Mat mask2(inputMat.rows, inputMat.cols,  CV_8UC1, Scalar(0,0,0));
	vector<int> HSV = HSVrange;
	//cout<<HSV.size()<<endl;

	int hmin,hmax,smin,smax,vmin,vmax;
	if(HSV.size() == 6){
		hmin = HSV[0] ;
		hmax = HSV[1] ;
		smin = HSV[2] ;
		smax = HSV[3] ;
		vmin = HSV[4] ;
		vmax = HSV[5] ;
		//cout<<hmin<<"    "<<hmax<<"    "<<smin<<"    "<<smax<<"    "<<vmin<<"    "<<vmax<<"    \n";
		//Mat dst(inputMat.rows, inputMat.cols,  CV_8UC3, Scalar(0,0,0));
		cvtColor(inputMat, hsv, CV_BGR2HSV);
		
		if(HSV[0] <= HSV[1]){
			inRange(hsv,Scalar(hmin,smin,vmin) , Scalar(hmax,smax,vmax), mask); 
		}else{
			inRange(hsv,Scalar(hmin,smin,vmin) , Scalar(255,smax,vmax), mask); 
			inRange(hsv,Scalar(0,smin,vmin) , Scalar(hmax,smax,vmax), mask2); 
			mask = mask + mask2;	
		}

		//开操作 (去除一些噪点)
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
		morphologyEx(mask, mask, MORPH_OPEN, element);

		//闭操作 (连接一些连通域)
		morphologyEx(mask, mask, MORPH_CLOSE, element);

		//inputMat.copyTo(dst,mask);
		//cv::namedWindow("mask", CV_WINDOW_NORMAL);
		//cv::imshow("mask", mask);
	}
	//this ->mask = mask;
	return mask;
}

void Object::FindContours(Mat frame, Mat mask){
	Mat thr = mask.clone();
	vector<Vec4i> hierarchy;
	contours.clear();
	largest_contour_index=0;
	findContours( thr, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image

	Rect bounding_rect;
	Mat dst(frame.rows,frame.cols,CV_8UC1,Scalar::all(0));
	//vector<Vec4i> hierarchy;
	int largest_area=0;

	for( int i = 0; i< contours.size(); i++ ) // iterate through each contour. 
	{
		double a=contourArea( contours[i],false);  //  Find the area of contour
		if(a>largest_area){
			largest_area=a;
			largest_contour_index=i;                //Store the index of largest contour
			bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
	}
	extRight = bounding_rect.x + bounding_rect.width;
	extLeft = bounding_rect.x;
	extTop = bounding_rect.y + bounding_rect.height;
	extBot = bounding_rect.y;
	rectangle(frame, bounding_rect,  Scalar(0,255,0),1, 8,0);  
	Scalar color( 255,255,255);
	area = bounding_rect.area();
	drawContours( dst, contours,largest_contour_index, color, CV_FILLED, 8, hierarchy ); // Draw the largest contour using previously stored index.
	//imshow( "ball", frame );
}
Point Object::FindCenter(){
/*	
	Moments mu;
	Point mc;
	if(contours.size()){
		mu = moments( contours[largest_contour_index], false ); 
		mc = Point( mu.m10/mu.m00 , mu.m01/mu.m00 );
	}
	_center = mc;
	return mc;
*/	
	_center.x=(extLeft+extRight)/2;
	_center.y=extTop;
	return _center;
}
void Object::Findoffset(int center){
	//offset = _center.x-center; 
	offset = _center.x-center; 
	if(Setting.size()){
		offset = _center.x-center + Setting[0];
	}
}
void Object::FindAngle(Mat frame){
	double PI = 3.14159;

	if(Setting.size()){
		int y = -(_center.x - (frame.cols/2 - Setting[0]));
		int x = -(_center.y - (frame.rows + Setting[1]*10));
		angle = atan2(y,x)*180/PI;
	}
}
void Object::FindDistance(Mat frame){
	int height;
	float dis;
	float real_dis;
	if(Setting.size()){
		height = Setting[2];
		dis = sqrt(pow(_center.x-frame.cols/2,2) + pow(frame.rows - _center.y,2));
		real_dis = sqrt(pow(height,2) + pow(dis*((double)Setting[3]/(double)100),2));
		distance = (float)real_dis/(float)100;
	}
}
void Object::Set_Mask(const std_msgs::Int32 msg){
	
}
