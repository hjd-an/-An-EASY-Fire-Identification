#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>

#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include<vector>
#include <errno.h>
#include <string.h>
#include<algorithm>
#include<iostream> 


using namespace cv;
using namespace std;
using namespace ros;



Mat CheckColor(Mat &inImg, Mat &foreImg);
Mat DrawFire(Mat &inputImg, Mat &foreImgint);



int main(int argc,char** argv)
{
  init(argc, argv, "img_publisher");
  NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
 
  VideoCapture cap;
  Mat frame;
  Mat tempframe, currentframe, previousframe, difframe;
  int framenum = 0;

  int deviceID=0;
  if(argc>1)
    deviceID=argv[1][0]-'0';
  int apiID=cv::CAP_ANY;
  cap.open(deviceID+apiID);
  if(!cap.isOpened()){
	cerr<<"ERROR! Unable to open camera"<<std::endl;
	return -1;
  }
 
  Rate loop_rate(30);
  while (nh.ok()) {
	 cap.read(frame);          // 读取当前图像到frame
     

         previousframe = frame.clone();
         cap.read(frame); 
         currentframe = frame.clone();
         //imshow("veiwer", frame);  // 将图像输出到窗口
         sensor_msgs::ImagePtr  msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  // 图像格式转换
         //pub.publish(msg);         // 发布图像信息
         spinOnce();                // 没什么卵用，格式像样
         loop_rate.sleep();              // 照应上面设置的频率

       cvtColor(previousframe, previousframe, CV_BGR2GRAY);//帧差法
       cvtColor(currentframe, currentframe, CV_BGR2GRAY);
       absdiff(currentframe, previousframe, difframe);//做差求绝对值    
      
       dilate(tempframe, tempframe, Mat());//膨胀  
       erode(tempframe, tempframe, Mat());//腐蚀

       //imshow("运动目标", tempframe);
       //frame=CheckColor(frame,tempframe);
       CheckColor(frame,tempframe);
       if(!frame.empty()){
	     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	     pub.publish(msg);} 
       pub.publish(msg);         // 发布图像信息

       if(cv::waitKey(2) >= 0)         // 延时ms,按下任何键退出(必须要有waitKey，不然是看不到图像的)
           { break; }      
		
   //if (cvWaitKey(30) == 32)//空格键停止
    //{
   // break;
   //       }	
  }
  return 0;
}






Mat DrawFire(Mat &inputImg,Mat &foreImg)  
{   
       
        vector<vector<Point>> contours_set;//保存轮廓提取后的点集及拓扑关系  
	findContours(foreImg, contours_set, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);	
	Point point1;
	Point point2;	
	float a = 0.4, b = 0.75;
	float xmin1 = a*inputImg.cols, ymin1 = inputImg.rows, xmax1 = 0, ymax1 = 0;
	float xmin2 = b*inputImg.cols, ymin2 = inputImg.rows, xmax2 = a*inputImg.cols, ymax2 = 0;
	float xmin3 = inputImg.cols, ymin3 = inputImg.rows, xmax3 = b*inputImg.cols, ymax3 = 0;
	Rect finalRect1;
	Rect finalRect2;
	Rect finalRect3;	
	vector<vector<Point> >::iterator iter = contours_set.begin();
	for (; iter != contours_set.end();)
	{
		Rect rect = boundingRect(*iter);
		float radius;
		Point2f center;
		minEnclosingCircle(*iter, center, radius);
 
		if (rect.area()> 0)
		{			
			point1.x = rect.x;
			point1.y = rect.y;
			point2.x = point1.x + rect.width;
			point2.y = point1.y + rect.height;
		
			if (point2.x< a*inputImg.cols)
			{
				if (point1.x < xmin1)				
					xmin1 = point1.x;
				if (point1.y < ymin1)
					ymin1 = point1.y;				
				if (point2.x > xmax1 && point2.x < xmax2)				
					xmax1 = point2.x;
				if (point2.y > ymax1)
					ymax1 = point2.y;				
			}
 
			if (point2.x < b*inputImg.cols&&point2.x > a*inputImg.cols)
			{
				if (point1.x < xmin2 && point1.x>xmin1)				
					xmin2 = point1.x;
				if (point1.y < ymin2)
					ymin2 = point1.y;
				if (point2.x > xmax2 && point2.x < xmax3)				
					xmax2 = point2.x;
				if (point2.y > ymax2)
					ymax2 = point2.y;				
			}
 
			if (point2.x < inputImg.cols&&point2.x > b*inputImg.cols)
			{
				if (point1.x < xmin3 && point1.x>xmin2)				
					xmin3 = point1.x;
				if (point1.y < ymin3)
					ymin3 = point1.y;				
				if (point2.x > xmax3)				
					xmax3 = point2.x;
				if (point2.y > ymax3)
					ymax3 = point2.y;				
			}
			
			++iter;
		}
		else
		{
			iter = contours_set.erase(iter);
		}
		
	}
 
	
	if (xmin1 == a*inputImg.cols&& ymin1 == inputImg.rows&&xmax1 == 0 && ymax1== 0)
	{
		xmin1 = ymin1 = xmax1 = ymax1 = 0;
	}
	if (xmin2 == b*inputImg.cols&& ymin2 == inputImg.rows&& xmax2 == a*inputImg.cols&& ymax2 == 0)
	{
		xmin2 = ymin2 = xmax2 = ymax2 = 0;
	}
	if (xmin3 == inputImg.cols&&ymin3 == inputImg.rows&& xmax3 == b*inputImg.cols&& ymax3 == 0)
	{
		xmin3 = ymin3 = xmax3 = ymax3 = 0;
	}
	finalRect1= Rect(xmin1, ymin1, xmax1 - xmin1, ymax1 - ymin1);
	finalRect2 = Rect(xmin2, ymin2, xmax2 - xmin2, ymax2 - ymin2);
	finalRect3 = Rect(xmin3, ymin3, xmax3 - xmin3, ymax3 - ymin3);
	rectangle(inputImg, finalRect1, Scalar(0, 255, 0));
	rectangle(inputImg, finalRect2, Scalar(0, 255, 0));
	rectangle(inputImg, finalRect3, Scalar(0, 255, 0));

       
        //imshow("showFire",inputImg); 
        return inputImg;
}  



//The Color Check is According to "An Early Fire-Detection Method Based on Image Processing"  
//The Author is:Thou-Ho (Chao-Ho) Chen, Ping-Hsueh Wu, and Yung-Chuen Chiou  

Mat CheckColor(Mat &inImg,Mat &foreImg)  
{  
    Mat fireImg;  
    fireImg.create(inImg.size(),CV_8UC1);  
      

    vector<vector<Point>> contours_set;//保存轮廓提取后的点集及拓扑关系  
    findContours(foreImg,contours_set,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);    
    Mat result0;  
    vector<vector<Point> >::iterator iter = contours_set.begin() ;  
 

    int redThre = 110; // 115~135  
    int saturationTh = 45; //55~65  
    Mat multiRGB[3];  
    int a = inImg.channels();  
    split(inImg,multiRGB); //将图片拆分成R,G,B,三通道的颜色  
  
    for (int i = 0; i < inImg.rows; i ++)  
    {  
      for (int j = 0; j < inImg.cols; j ++)  
       {  
         if(iter != contours_set.end())
           { Rect rect = boundingRect(*iter );  
        
            float B,G,R;  
            B = multiRGB[0].at<uchar>(i,j); //每个像素的R,G,B值  
            G = multiRGB[1].at<uchar>(i,j);  
            R = multiRGB[2].at<uchar>(i,j);     
  
            /*B = inImg.at<uchar>(i,inImg.channels()*j + 0); //另一种调用图片中像素RGB值的方法 
            G = inImg.at<uchar>(i,inImg.channels()*j + 1); 
            R = inImg.at<uchar>(i,inImg.channels()*j + 2);*/  
  
            int maxValue = max(max(B,G),R);  
            int minValue = min(min(B,G),R);  
  
            double S = (1-3.0*minValue/(R+G+B));  
  
            //R > RT  R>=G>=B  S>=((255-R)*ST/RT)  
            if(R > redThre && R >= G && G >= B && S >0.20 && S >((255 - R) * saturationTh/redThre) && rect.area()> 0)  
            {  
                fireImg.at<uchar>(i,j) = 255;  
            }  
            else  
            {  
                fireImg.at<uchar>(i,j) = 0;  
            }  
          }
           
        }  
    }  
  
    dilate(fireImg,fireImg,Mat(5,5,CV_8UC1));  
    //imshow("fire",fireImg);   
  
    DrawFire(inImg,fireImg); 
      
    return inImg;  
}  

