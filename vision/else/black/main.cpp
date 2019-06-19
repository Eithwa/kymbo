#include <cstdio>
#include <opencv2/opencv.hpp>
#include <cmath>

#include <vector>
using std::vector;

#include <queue>
using std::queue;

#include <iostream>

#define IMAGE_TEST1 "src/vision/black/1.bmp"


class Coord{
public:
	Coord(int x_,int y_):x(x_), y(y_){}
	Coord operator+(const Coord& addon) const { return Coord(x + addon.x, y + addon.y); }
	int get_x() const { return x; }
	int get_y() const { return y; }
private:
  int x, y;
};

Coord directions[8]={
	Coord( 0,  1),
	Coord( 0, -1),
	Coord(-1,  0),
	Coord( 1,  0),
	Coord(-1,  1),
	Coord( 1,  1),
	Coord(-1, -1),
	Coord( 1, -1)
};

using namespace cv;
bool is_black(Mat& img, const Coord& c) { return img.data[3 * (c.get_y() * img.cols + c.get_x())] == 0; }

int main(){
	Mat img = imread( IMAGE_TEST1 , CV_LOAD_IMAGE_COLOR );   
	int inner=36, outer=230, centerx=270,centery=270;
	int width=589,length=478;
	int obj_size=500;
	
	vector<vector<Coord> > obj; //物件列表

	bool check[width][length];//確認搜尋過的pix

	//清理標記與物件
	for(int i=0;i<width;i++){
		for(int j=0;j<length;j++){
			check[i][j] = false;
		}
	}

	obj.clear();

	//二值化
	for(int i=0;i<img.rows;i++){
	  for(int j=0;j<img.cols;j++){
		  unsigned char gray= img.data[(i*img.cols*3)+(j*3)+0]*0.114
												+ img.data[(i*img.cols*3)+(j*3)+1]*0.587
												+ img.data[(i*img.cols*3)+(j*3)+2]*0.299;
					  					 //B*0.114+G*0.587+R*0.299
			if(gray>90)gray = 255;
			else gray = 0;
			img.data[(i*img.cols*3)+(j*3)+0] = gray;
			img.data[(i*img.cols*3)+(j*3)+1] = gray;
			img.data[(i*img.cols*3)+(j*3)+2] = gray;
		}	
	}

	//inner中心塗白 outer切斷與外面相連黑色物體 
	circle(img, Point(centerx,centery), inner, Scalar(255,255,255), -1);
	circle(img, Point(centerx,centery), outer, Scalar(255,0,0), 2);
	//搜尋範圍顯示
	rectangle(img, Point(centerx-outer,centery-outer), Point(centerx+outer,centery+outer), Scalar(255,0,0), 2);

	//選取的範圍做搜尋
	//0為黑
	for(int i = centerx - outer; i < centerx + outer; i++) {
		for(int j = centery - outer; j < centery + outer; j++) {
			//std::cout << i << " "	<< j << std::endl
			if(!check[i][j]) {
				check[i][j] = true;
				if(pow(centerx-i,2)+pow(centery-j,2)>pow(outer-5,2))continue;
				if(is_black(img, Coord(i, j))){

					queue<Coord> bfs_list;
					bfs_list.push(Coord(i, j));

					vector<Coord> dot;
					dot.push_back(Coord(i, j));
					
					//放入佇列
					while(!bfs_list.empty()) {

						Coord ori = bfs_list.front();
						bfs_list.pop();
					
						//搜尋八方向
						for(int k = 0; k < 8; k++){
							Coord dst = ori + directions[k];

							//處理邊界
							if((dst.get_x() < 0)||(dst.get_x() >= width)||(dst.get_y() < 0)||(dst.get_y() >= length)) continue;

							if(check[dst.get_x()][dst.get_y()]) continue;
							
							if(is_black(img, dst)) {
								bfs_list.push(dst);
								dot.push_back(dst);
							}
							check[dst.get_x()][dst.get_y()] = true;
						}
					}
					obj.push_back(dot);        
				}
			}
		}
	}

	//上色		
	for(int i = 0; i < obj.size(); i++){
		//需要塗色的物體大小
		if(obj[i].size()>obj_size) continue;
		for(int j = 0; j < obj[i].size(); j++){
			Coord point = obj[i][j];
			line(img, Point(point.get_x(), point.get_y()), Point(point.get_x(), point.get_y()) ,Scalar(0,0,255), 1);
		}
	}

	imshow("window", img);
	waitKey(0); 

	return 0;
}

