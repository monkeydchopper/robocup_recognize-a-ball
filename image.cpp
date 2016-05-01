/*
 * image.cpp
 *
 *  Created on: 2015年4月6日
 *      Author: monkeyd
 */

/*
 *1.找球
 找球分为两种方法去找,两种方法都要对原图匹配颜色表,将绿色(场地)匹配为黑色,其他颜色都置为白色,
 并将匹配到的白色都置到white_cvtImage中,用于之后的检测.然后对匹配完颜色表的图像HSVImage(在hsv颜色空间下匹配的颜色表)
 提轮廓,并对轮廓进行筛选.
 第一种,对提完轮廓的图像直接调用库函数(cvMinEnclosingCircle)对所有轮廓拟合最小外接圆,再根据拟合出的圆内白点数绿点数是否符合条件判断
 是不是需求的那个圆,第一种找不到球时,调用第二种方法.
 第二种,对提完轮廓的图像进行霍夫圆变换找圆,找到的圆再根据内部的白点绿点数是否满足条件来判定是不是球.
 */

//显示图像，颜色表读取并与图片匹配
/*
#include <opencv2/opencv.hpp>
#include<stdio.h>
#include <math.h>
#include<cxmisc.h>
#include <sys/time.h>
#include <unistd.h>

CvMemStorage* stor = cvCreateMemStorage(0);				//这个用于存储第一种方法中的轮廓
CvSeq *cont;
int main(){
	IplImage *input= cvLoadImage("/home/monkeyd/picture/9.bmp",0);
	IplImage *output = cvCreateImage(cvGetSize(input),input->depth,input->nChannels);
	//cvSmooth(input,input,CV_GAUSSIAN,5);
	//cvDilate(input, input, 0, 1);
	//cvErode(input,input,0,1);
	//cvCanny(input,output,50,100,3);
	cvFindContours(input, stor, &cont, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
	for (; cont; cont = cont->h_next){
			if(cont->total < 800 ){
				continue;
			}
	//		cout<<"cont_total="<<cont->total<<endl;
			cvDrawContours(output,cont,cvScalar(255,255,255),cvScalar(255,255,255),0);
	}
			cvShowImage("output",output);
			cvWaitKey();
	cvSaveImage("/home/monkeyd/picture/11.bmp",output);
}*/


















#include <opencv2/opencv.hpp>
#include<stdio.h>
#include <math.h>
#include<cxmisc.h>
#include <sys/time.h>
#include <unistd.h>
#include "tcp.h"
using namespace std;
//const double PI = 2*asin(1);

//#define GONG_KONG
//#define method1



#define hough_cmp_gt(l1,l2) (aux[l1] > aux[l2])
static CV_IMPLEMENT_QSORT_EX( icvHoughSortDescent32s, int, hough_cmp_gt, const int* );
unsigned char H,S,V;
unsigned char HSV[64][64][64]={0};
CvCapture* capture;
IplImage *Image,*ball_Image,* pImage,*white_Image,*white_cvtImage,*HSVImage,*HSVImagegray; 						//HSVImagegray和white_cvtImage分别是匹配完绿色和白色的灰度图
//whiteImage用来起匹配场地中的白色,全局变量直接在Color_Table()中赋值
//但是whiteImage要在大循环中至零,不然匹配的白点会累积
//cv::Ptr<CvMat> edges;										//第一种方法将轮廓画入这张图,用于第二种方法处理
CvMat* edges;
CvMemStorage* stor = cvCreateMemStorage(0);				//这个用于存储第一种方法中的轮廓
CvSeq *cont;
CvMat temp;
CvMat* mat;									//用于拷贝的矩阵,传入函数icvHoughCirclesGradient

typedef struct{
	int x;
	int y;
	float r;
	bool ball_find;
}Ball_info;

Ball_info ball_info;



int white_points_low_2 = 50;
int white_points_up_2 = 350;
int green_points_low_2 = 5;
int green_points_up_2= 200;
int green_circle_low_2 = 5;
int green_circle_up_2= 170;
int max_count_best_2 = 160;
int white_points_low_1 = 5;
int white_points_up_1 = 250;
int green_points_low_1 = 5;
int green_points_up_1= 150;
double t1,t2;

int Image_Init();
void Image_Run();
void Image_Show();
void Image_Clear();
IplImage* Color_Table(IplImage *src );
int Read_ColorTable();
bool get_ball_pos(CvMat *src);
bool get_ball_pos_1(CvMat *src);
//输入为单通道图像,我们用的时二值化图像
void print(IplImage* img, int x, int y, const char* text);
bool get_ball_pos_2( CvMat* img, float dp,
                         int min_radius, int max_radius,
                          float acc_threshold,float max_count_best_threshold);



double get_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double t;
	t = tv.tv_sec + tv.tv_usec * pow(10, -6);
	return t;
}





//发送数据， 成功则返回未发送完的字节数， 失败则返回-1
int write_sock(int sock_fd, char *buf, int length)
{
	int byte_send;
	if((byte_send = write(sock_fd, buf, length)) == -1){
		std::cout<<"error in write sock."<<std::endl;
		return -1;
	}
	else{
		return length - byte_send;
	}
}

//发送一幅图像
bool image_send_one_frame(IplImage *img)
{
	int length = img->imageSize;
	int bytes_remain;
	char *img_data = img->imageData;
	while((bytes_remain = write_sock(client_sock, img_data, length)) != 0)
	{
		if(bytes_remain == -1)
			return false;
		img_data += length - bytes_remain;
		length = bytes_remain;
	}
	return true;
}

bool image_update()
{
	image_send_one_frame(ball_Image);
//	cvShowImage("simple-demo", show_Image);
	char ch = cvWaitKey(1);
	if(ch == ' '){
		return false;
	}
	return true;
}




int main ()
{
	int frames = 0;
	int detected = 0;
	if(Image_Init() == 0)
	{
		return 0;
	}
	cvWaitKey();
	t1 = get_time();
//	server_sock_init(&client_sock, 8888, "192.168.1.190");
    	while(1){
    	Image_Run();
    	frames++;
    	get_ball_pos(mat);

#ifdef GONG_KONG
    	image_send_one_frame(ball_Image);
#endif
	    Image_Show();
	char c=cvWaitKey(20);
//	if (ball_info.ball_find == true)
//		c = cvWaitKey();
	if (ball_info.ball_find == true)
		detected ++;
	if(get_time( ) - t1 >= 10.0)
		break;
	if(c=='s'){
				cvSaveImage("/home/monkeyd/picture/28.bmp",ball_Image);
				cvSaveImage("/home/monkeyd/picture/29.bmp",HSVImage);
				cvSaveImage("/home/monkeyd/picture/30.bmp",white_cvtImage);
				cvSaveImage("/home/monkeyd/picture/31.bmp",edges);
			}
	else if(c == 'c') continue;
	else if(c== 'b')
		break;
    	}
		cout<<"frames = "<<frames<<endl;
		cout<<"detected = "<<detected<<endl;
    	Image_Clear();
}







int Image_Init(){
	if(Read_ColorTable() == 0)
		{
			return 0;
		}
#ifdef GONG_KONG
	capture = cvCreateCameraCapture(0);
#else
		capture = cvCreateCameraCapture(1);
#endif
		Image = cvQueryFrame(capture);
#ifndef GONG_KONG
		cvNamedWindow("Image",1);
		cvNamedWindow("HSVImage",1);
#endif
		CvSize sz;																						//这里对图像改变大小
		sz.height = Image->height*0.5;
		sz.width = Image->width*0.5;
		pImage = cvCreateImage( sz, Image->depth, 3); //构造目标图象
		cvResize(Image,pImage,CV_INTER_LINEAR);
		ball_Image = cvCreateImage(cvGetSize(pImage),8,3);
		HSVImage=cvCreateImage(cvGetSize(pImage),8,3);
		white_Image=cvCreateImage(cvGetSize(pImage),pImage->depth,pImage->nChannels);
		white_cvtImage=cvCreateImage(cvGetSize(pImage),pImage->depth,1);
		HSVImagegray = cvCreateImage(cvGetSize(HSVImage),HSVImage->depth,1);
		cvCvtColor(pImage, pImage, CV_RGB2HSV);											//这些操作在while(1)中也有,这是对第一帧的操作,因为edges需要用mat来初始化,而mat需要HSVImagegray
		HSVImage = Color_Table(pImage);
		cvCvtColor(HSVImage,HSVImagegray,CV_RGB2GRAY);
		cvCvtColor(white_Image,white_cvtImage,CV_RGB2GRAY);
#ifndef GONG_KONG
		cvCreateTrackbar("white_points_low","Image",&white_points_low_2,300,NULL);
		cvCreateTrackbar("white_points_up","Image",&white_points_up_2,300,NULL);
		cvCreateTrackbar("green_points_low","Image",&green_points_low_2,300,NULL);
		cvCreateTrackbar("green_points_up","Image",&green_points_up_2,300,NULL);
		cvCreateTrackbar("green_circle_low","Image",&green_circle_low_2,300,NULL);
		cvCreateTrackbar("green_circle_up","Image",&green_circle_up_2,300,NULL);
		cvCreateTrackbar("max_count_best","Image",&max_count_best_2,300,NULL);
		cvCreateTrackbar("white_points_low","HSVImage",&white_points_low_1,300,NULL);
		cvCreateTrackbar("white_points_up","HSVImage",&white_points_up_1,300,NULL);
		cvCreateTrackbar("green_points_low","HSVImage",&green_points_low_1,300,NULL);
		cvCreateTrackbar("green_points_up","HSVImage",&green_points_up_1,300,NULL);
#endif
		mat = cvGetMat(HSVImagegray, &temp);  //深拷贝
		//canny算子求单像素二值化边缘，保存在edges变量中
		edges = cvCreateMat( mat->rows, mat->cols, CV_8UC1 );
		return 1;
}


void Image_Run(){
	Image = cvQueryFrame(capture);
	cvResize(Image,ball_Image,CV_INTER_LINEAR);
	cvResize(Image,pImage,CV_INTER_LINEAR);
	cvCvtColor(pImage, pImage, CV_RGB2HSV);
	HSVImage = Color_Table(pImage);
	cvCvtColor(HSVImage,HSVImagegray,CV_RGB2GRAY);
	cvCvtColor(white_Image,white_cvtImage,CV_RGB2GRAY);
	mat = cvGetMat(HSVImagegray, &temp);  //深拷贝
}

void Image_Show(){
//	if(ball_info.ball_find == true)cvCircle(ball_Image,cvPoint(ball_info.x,ball_info.y),ball_info.r,cvScalar(0,0,255),2,8,0);
#ifndef GONG_KONG
	cvShowImage("edges",edges);
	cvShowImage("Image",ball_Image);
	cvShowImage("HSVImage",HSVImage);
	cvShowImage("HSV",HSVImagegray);
	cvShowImage("white_cvtimage",white_cvtImage);
#endif
	cvSetZero(white_Image);
	cvSetZero(edges);
}

void Image_Clear(){
#ifndef GONG_KONG
	cvDestroyWindow("Image");
	cvDestroyWindow("HSVImage");
#endif
	cvClearMemStorage(stor);
	cvReleaseImage(&ball_Image);
	cvReleaseImage(&HSVImage);
	cvReleaseImage(&white_cvtImage);
	cvReleaseCapture(&capture);
}

IplImage* Color_Table(IplImage *src )
{
	IplImage *HSVImage = cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
	CvSize  size=cvGetSize(src);
//	long i;
	CvScalar s;
	int x,y;
	int nstep=src->widthStep;
	int nchannel=src->nChannels;
    for(y=0;y<size.height;y++)
    {
    	for(x=0;x<size.width;x++)
	    {
	    	//得到HS值
		    H=src->imageData[y*nstep+x*nchannel+1];
		    S=src->imageData[y*nstep+x*nchannel+2];
		    V=src->imageData[y*nstep+x*nchannel+0];
            H=H/4; S=S/4;V=V/4;
		  // 查表

		   switch (HSV[H][S][V])
		   {   case  0:														//代表其他颜色
			   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
			   s.val[0]=0;
				s.val[1]=0;
				s.val[2]=0;
				cvSet2D(white_Image,y,x,s);//set the (i,j) pixel value
			   break;
   	   case  1:																//代表绿色
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
			   s.val[0]=0;
				   s.val[1]=0;
				   s.val[2]=0;
				   cvSet2D(white_Image,y,x,s);//set the (i,j) pixel value
		       break;
	   	   case  2:															//代表白色
	   	   	   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
				   s.val[0]=255;
				   s.val[1]=255;
				   s.val[2]=255;
				   cvSet2D(white_Image,y,x,s);//set the (i,j) pixel value
			       break;
		   	   default:
//		   		     cout<<"                                default     colour="<<HSV[H][S][V]+0<<endl;
		   		      break;
		   }
	    }
    }
    cvSmooth(HSVImage,HSVImage,CV_MEDIAN,3);
   // cvSmooth(white_Image,white_Image,CV_MEDIAN,3);
    return HSVImage;
}


int Read_ColorTable()
{
	long x,y,z;
	long fp_position;
		FILE  *fp;
		  if((fp = fopen("ColourTable_HSV.txt", "r+")) == NULL){
			  cout<<"can't open the file\n"<<endl;
		  	  return 0;
		  }
		        	 for( x=0;x<64;x++)
		        		  		{
		        				for( y=0;y<64;y++)
		        					{
		        					for( z=0;z<64;z++)
		        					{
		                            rewind(fp);
		        					fp_position=ftell(fp);
//		        					cout<<"重置指针位置"<<fp_position<<endl;
		                            fseek(fp,4*long(x*64*64+y*64+z),0);
		        						fread(&HSV[x][y][z],sizeof(int), 1,fp) ;
//		        						cout<<"colour="<<HSV[x][y][z]+0<<endl;
		        					}
		        					}
		        		  		}

				fclose(fp);
				return 1;
}


bool get_ball_pos(CvMat *src){
#ifdef	method1
			if(get_ball_pos_1(src) == true){
					ball_info.ball_find = true;
	//		    	cout<<"t1 = "<<get_time()-t1<<endl;
	//				cout<<"ball.r = "<<ball_info.r<<endl;
					 cvCircle(ball_Image,cvPoint(ball_info.x,ball_info.y),ball_info.r,cvScalar(0,0,255),2,8,0);

					 return true;
				}
#else
				if(get_ball_pos_2(src,1.2,(ball_Image->width/40),ball_Image->width/4,1.0,((float)max_count_best_2/100)) == true){
					ball_info.ball_find = true;
	//		    	cout<<"t2 = "<<get_time()-t2<<endl;
	//				cout<<"ball.r = "<<ball_info.r<<endl;
					 cvCircle(ball_Image,cvPoint(ball_info.x,ball_info.y),ball_info.r,cvScalar(0,255,255),2,8,0);
					 return true;
				}
#endif
		    else{
		    	    	 ball_info.ball_find = false;
		   // 	    	 cout<<"can't find the ball"<<endl;
		    	    	 return false;
		    	    }


}

bool get_ball_pos_1(CvMat *src){
	const double area_ratio_limit = 0.4;	//可调
	CvPoint2D32f center;
	float real_radius=0;
	double cont_area;
	double circle_area;
	double area_ratio = 0;
	double max_area_ratio = 0;
	float max_radius = 0;
	int x, y, r;
	//t1 = get_time();
	cvFindContours(src, stor, &cont, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
	for (; cont; cont = cont->h_next)
	{

		if(cvMinEnclosingCircle(cont, &center, &real_radius) <= 0){
			continue;
		}
		if(cont->total < 10 || cont->total > 300){
			continue;
		}
//		cout<<"cont_total="<<cont->total<<endl;
		if(real_radius < 8 || real_radius > 150){
			continue;
		}
		cvDrawContours(edges,cont,cvScalar(255,255,255),cvScalar(255,255,255),0);
		cont_area = cvContourArea(cont);
			circle_area = CV_PI * real_radius * real_radius;
			area_ratio = cont_area/circle_area;
			if(area_ratio < area_ratio_limit){
				continue;
			}
			if(area_ratio < max_area_ratio || real_radius < max_radius){
				continue;
			}
			max_area_ratio = area_ratio;
						max_radius = real_radius;
						   int white_points_in_circle=0;
							int green_points_in_circle = 0;
							int nchannel = white_cvtImage->nChannels;
							int nstep = white_cvtImage->widthStep;
						        	for(int l = -real_radius;l  <= real_radius ;l++){
						        		int d = sqrt(real_radius*real_radius - l*l);
						        		for(int d_tmp = -d;d_tmp < d ; d_tmp++){
						        			 int cy_l = center.y+l;
						        			 int cx_d = center.x+d_tmp;
						        			 if(cy_l<1 || cy_l>white_cvtImage->height-1) continue;
						        			if(cx_d<1 || cx_d>white_cvtImage->width-1) continue;
						        			if(white_cvtImage->imageData[cy_l*nstep+cx_d*nchannel]!=0){
						        				white_points_in_circle++;
			//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
						        			}
						        			if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0){
						        				green_points_in_circle++;
			//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
						        			}
						        		}
						        	}
						        	if((white_points_in_circle>real_radius*real_radius*((float)white_points_low_1/100))&&(white_points_in_circle<real_radius*real_radius*((float)white_points_up_1/100))&&
						        		(green_points_in_circle>real_radius*real_radius*((float)green_points_low_1/100))&&(green_points_in_circle<real_radius*real_radius*((float)green_points_up_1/100)))
						        								        	{
						        												ball_info.x = (center.x + 0.5);
						        												ball_info.y =  (center.y + 0.5);
						        												ball_info.r = (real_radius+0.5);
//						        												cout<<"r1="<<ball_info.r<<endl;
						        												return true;
						        								        	}
						        			}

						        				return false;




}

bool get_ball_pos_2( CvMat* img, float dp,
                         int min_radius, int max_radius,
                          float acc_threshold,float max_count_best_threshold)
{
	//参数：
	//img: 输入图像
	//dp: 识别精度,1.0表示按照原图精度
	//min_dist: 圆心点位置识别精度
	//min_radius: 所需要找的圆的最小半径
	//max_radius：所需要找的圆的最大半径
	//canny_threshold：canny算子的高阀值
	//acc_threshold：累加器阀值，计数大于改阀值的点即被认为是可能的圆心
	//circles: 保存找到的符合条件的所有圆
	//circles_max: 最多需要的找到的圆的个数
    const int SHIFT = 10, ONE = 1 << SHIFT;
    cv::Ptr<CvMat> dx, dy;
 //   cv::Ptr<CvMat> edges, edge_initial,accum, dist_buf;
    cv::Ptr<CvMat> accum, dist_buf;
    std::vector<int> sort_buf;
    cv::Ptr<CvMemStorage> storage;

    int x, y, i, j, k, center_count, nz_count;
    float min_radius2 = (float)min_radius*min_radius;
    float max_radius2 = (float)max_radius*max_radius;
    int rows, cols, arows, acols;
    int astep, *adata;
    float* ddata;
    CvSeq *nz, *centers;
    float idp, dr;
    CvSeqReader reader;



  //  t2 = get_time();
    cvFindContours(img, stor, &cont, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

    for (; cont; cont = cont->h_next)
    {
    	if(cont->total<50){
    		continue;
    	}
    	cvDrawContours(edges,cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0);
    }

    //sobel算子求水平和垂直方向的边缘，用于计算边缘点的法线方向
    dx = cvCreateMat( img->rows, img->cols, CV_16SC1 );
    dy = cvCreateMat( img->rows, img->cols, CV_16SC1 );
    cvSobel( img, dx, 1, 0, 3 );
    cvSobel( img, dy, 0, 1, 3 );

    //dp表示识别精度
    if( dp < 1.f )
        dp = 1.f;
    idp = 1.f/dp;
    //accum用作累加器，包含图像中每一个点的计数。图像中每一个点都有一个计数，点的计数表示每一个canny边缘点法线方向上，
    //到该点距离为R的边缘点的个数，初始化为0
    accum = cvCreateMat( cvCeil(img->rows*idp)+2, cvCeil(img->cols*idp)+2, CV_32SC1 );
    cvZero(accum);

    storage = cvCreateMemStorage();
    nz = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
    //centers用于保存可能的圆心点
    centers = cvCreateSeq( CV_32SC1, sizeof(CvSeq), sizeof(int), storage );

    rows = img->rows;
    cols = img->cols;
    arows = accum->rows - 2;
    acols = accum->cols - 2;
    adata = accum->data.i;
    astep = accum->step/sizeof(adata[0]);
    // Accumulate circle evidence for each edge pixel
    //以下这个循环用于获取所有可能的圆边缘点，存储在nz中，同时设置
    //累加器中的值
    for( y = 0; y < rows; y++ )
    {
        const uchar* edges_row = edges->data.ptr + y*edges->step;
        const short* dx_row = (const short*)(dx->data.ptr + y*dx->step);
        const short* dy_row = (const short*)(dy->data.ptr + y*dy->step);

        for( x = 0; x < cols; x++ )
        {
            float vx, vy;
            int sx, sy, x0, y0, x1, y1, r;
            CvPoint pt;
     //vx,vy均为梯度
            vx = dx_row[x];
            vy = dy_row[x];

            if( !edges_row[x] || (vx == 0 && vy == 0) )
                continue;

            float mag = sqrt(vx*vx+vy*vy);
            assert( mag >= 1 );
            //sx表示cos, sy表示sin
            sx = cvRound((vx*idp)*ONE/mag);
            sy = cvRound((vy*idp)*ONE/mag);

            x0 = cvRound((x*idp)*ONE);
            y0 = cvRound((y*idp)*ONE);
            // Step from min_radius to max_radius in both directions of the gradient
            //循环两次表示需要计算两个方向，法线方向和法线的反方向
            for(int k1 = 0; k1 < 2; k1++ )
            {
                //半径方向的水平增量和垂直增量
                x1 = x0 + min_radius * sx;
                y1 = y0 + min_radius * sy;
                //在法线方向和反方向上，距离边缘点的距离为输入的最大半径和最小半径范围内找点
                //每找到一个点，该点的累加器计数就加1
                for( r = min_radius; r <= max_radius; x1 += sx, y1 += sy, r++ )
                {
                    int x2 = x1 >> SHIFT, y2 = y1 >> SHIFT;
                    if( (unsigned)x2 >= (unsigned)acols || (unsigned)y2 >= (unsigned)arows)
                        break;
                    adata[y2*astep + x2]++;
                }
                //反方向
                sx = -sx; sy = -sy;
            }
            //保存可能的圆边缘点
            pt.x = x; pt.y = y;
            cvSeqPush( nz, &pt );
        }
    }

    nz_count = nz->total;
//    cout<<"边缘点个数"<<nz_count<<endl;
    if( !nz_count )
    {
    	cout<<"problem 1"<<endl;
        return false;
    }
        //Find possible circle centers
    //累加器中，计数大于阀值的点，被认为可能的圆心点。因为计算各点计数过程中，距离有误差，所以
   //在与阀值比较时，该点计数先要与4邻域内的各个点的计数比较，最大者才能和阀值比较。可能的圆心
   //点保存在centers中
    for( y = 1; y < arows - 1; y++ )
    {
        for( x = 1; x < acols - 1; x++ )
        {
            int base = y*(acols+2) + x;
            if( adata[base] > acc_threshold &&
                adata[base] > adata[base-1] && adata[base] > adata[base+1] &&
                adata[base] > adata[base-acols-2] && adata[base] > adata[base+acols+2]
				&&adata[base] > adata[base-acols]&&adata[base] > adata[base-acols-2]&&
				adata[base] > adata[base+acols+1]&&adata[base] > adata[base+acols])									//扫四临域,不扫八临域
              {
//			    cout<<"满足条件的Center的累加器"<<adata[base]<<endl;
            	if (adata[base]> 10)
                cvSeqPush(centers, &base);
              }
        }
    }
    center_count = centers->total;
 //   cout<< "满足条件的center共有"<<center_count<<endl;
    if( !center_count||center_count>2000 )
    {
    	cout<<"problem 2"<<endl;
        return false;
    }
    sort_buf.resize( MAX(center_count,nz_count) );
    //链表结构的certers转化成连续存储结构sort_buf
    cvCvtSeqToArray( centers, &sort_buf[0] );
    //经过icvHoughSortDescent32s函数后，以sort_buf中元素作为adata数组下标,
   //adata中的元素降序排列, 即adata[sort_buf[0]]是adata所有元素中最大的,
   //adata[sort_buf[center_count-1]]是所有元素中最小的
    icvHoughSortDescent32s( &sort_buf[0], center_count, adata );
    cvClearSeq( centers );
    //经过排序后的元素，重新以链表形式存储到centers中
    cvSeqPushMulti( centers, &sort_buf[0], center_count );

    dist_buf = cvCreateMat( 1, nz_count, CV_32FC1 );
    ddata = dist_buf->data.fl;

    dr = dp;
    // For each found possible center
    // Estimate radius and check support
    //对于每一个可能的圆心点，计算所有边缘点到该圆心点的距离。由于centers中的
   //元素已经经过前面排序，所以累加器计数最大的可能圆心点最先进行下面的操作




//    if(center_count>50){												//扫的圆心点数不能太少,特别是距离球比较远时
  //  	center_count = 50;
 //  }

    int max_count_best = 0;
    for(i=0;i< center_count;i++)

    {
        if(center_count>500) i=i+1;														//如果圆心点数太多,运算量就会很大,就让它隔几个点找一次.
        if(center_count>1000) i=i+1;
        int ofs = *(int*)cvGetSeqElem( centers, i );

        y = ofs/(acols+2);
        x = ofs - (y)*(acols+2);

        //Calculate circle's center in pixels
        float cx = (float)((x + 0.5f)*dp), cy = (float)(( y + 0.5f )*dp);

        float start_dist, dist_sum;
        float r_best = 0;
        int max_count = 0;
        // Check distance with previously detected circles
        //如果该可能的圆心点和已经确认的圆心点的距离小于阀值，则表示
       //这个圆心点和已经确认的圆心点是同一个点


        // Estimate best radius
        cvStartReadSeq( nz, &reader );

        //求所有边缘点到当前圆心点的距离，符合条件的距离值保存在ddata中
        for( j = k = 0; j < nz_count; j++ )
        {
            CvPoint pt;
            float _dx, _dy, _r2;
            CV_READ_SEQ_ELEM( pt, reader );
            _dx = cx - pt.x; _dy = cy - pt.y;
            _r2 = _dx*_dx + _dy*_dy;
            if(min_radius2 <= _r2 && _r2 <= max_radius2 )
            {
                ddata[k] = _r2;
                sort_buf[k] = k;
                k++;
            }
        }

        int nz_count1 = k, start_idx = nz_count1 - 1;
        if( nz_count1 == 0 )
            continue;
        dist_buf->cols = nz_count1;
        cvPow( dist_buf, dist_buf, 0.5 );
        //经过如下处理后，以sort_buf中元素作为ddata数组下标,ddata中的元素降序排列,
       //即ddata[sort_buf[0]]是ddata所有元素中最大的, ddata[sort_buf[nz_count1-1]]
       //是所有元素中最小的
        icvHoughSortDescent32s( &sort_buf[0], nz_count1, (int*)ddata );
        //对所有的距离值做处理，求出最可能圆半径值，max_count为到圆心的距离为最可能半径值的点的个数
        dist_sum = start_dist = ddata[sort_buf[nz_count1-1]];

        for( j = nz_count1 - 2; j >= 0; j-- )
        {
            float d = ddata[sort_buf[j]];

            if( d > max_radius )
                break;

            if( d - start_dist > dr )
            {
                float r_cur = ddata[sort_buf[(j + start_idx)/2]];
                if( (start_idx - j)*r_best >= max_count*r_cur ||
                    (r_best < FLT_EPSILON && start_idx - j >= max_count) )
                {
                    r_best = r_cur;
                    max_count = start_idx - j;
                }
                start_dist = d;
                start_idx = j;
                dist_sum = 0;
            }
            dist_sum += d;
        }
        // Check if the circle has enough support
        //max_count大于阀值，表示这几个边缘点构成一个圆
        int white_points_in_circle=0;
        int green_points_in_circle = 0;
        int green_circle_in_circle = 0;
    	int nchannel = white_cvtImage->nChannels;
    	int nstep = white_cvtImage->widthStep;

        if( max_count > int(acc_threshold*r_best) )
        {
//        	CvScalar s;
        	for(int l = -r_best;l  <= r_best ;l++){
        		int d = sqrt(r_best*r_best - l*l);
        		for(int d_tmp = -d;d_tmp <= d ; d_tmp++){
 //       			s.val[2] = 255;
//					s.val[1] = 0;
//					s.val[0] = 0;
					 int cy_l = cy+l;
					 int cx_d = cx+d_tmp;
					 if(cy_l<3 || cy_l>white_cvtImage->height-3) continue;
					 if(cx_d<3 || cx_d>white_cvtImage->width-3) continue;
        			if((d_tmp == -d)||(d_tmp == d)){
//        				if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0)	green_circle_in_circle++;
        				if(HSVImagegray->imageData[cy_l*nstep+(cx_d-2)*nchannel] ==0) green_circle_in_circle++;
        				if(HSVImagegray->imageData[cy_l*nstep+(cx_d+2)*nchannel] ==0) green_circle_in_circle++;
        			}

        			if(cy_l<1 || cy_l>white_cvtImage->height-1) continue;
        			if(cx_d<1 || cx_d>white_cvtImage->width-1) continue;
        			if(white_cvtImage->imageData[cy_l*nstep+cx_d*nchannel]!=0){
        				white_points_in_circle++;
 //       				cvSet2D(ball_Image,cy_l,cx_d,s);
        			}
//        			s.val[2] = 0;
 //       			s.val[1] = 255;
//					s.val[0] = 0;
        			if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0){
        				green_points_in_circle++;
//        				cvSet2D(ball_Image,cy_l,cx_d,s);
        			}
        		}
        	}
//	 		 if((white_points_in_circle>r_best*r_best*((float)white_points_low_2/100))&&(white_points_in_circle<r_best*r_best*((float)white_points_up_2/100))&&
//	 				 (green_points_in_circle>r_best*r_best*((float)green_points_low_2/100))&&(green_points_in_circle<r_best*r_best*((float)green_points_up_2/100))&&
//					 (green_circle_in_circle > r_best*CV_PI*2*((float)green_circle_low_2/100))&&(green_circle_in_circle < r_best*CV_PI*2*((float)green_circle_up_2/100)))
	 		 {
//	 			cout<<"max_count/r_best="<<(float)max_count/r_best<<endl;
	 			if(max_count > max_count_best){
	 								  ball_info.x = cx;
	 								  ball_info.y = cy;
	 								  ball_info.r = r_best;
	 								  max_count_best = max_count;
//					cout<<"i="<<i<<endl;
//				  cout<<"r_best="<<r_best<<endl;
//					cout<<"max_count_best/r_best="<<(float)max_count_best/r_best<<endl;
//				  cout<<"white_points_in_circle_ratio="<<((float)white_points_in_circle)/(r_best*r_best)<<endl;
//				  cout<<"green_points_in_circle_ratio="<<((float)green_points_in_circle)/(r_best*r_best)<<endl;
//				  cout<<"green_circle_in_circle="<<((float)green_circle_in_circle)/(r_best*CV_PI*2)<<endl;

				  }
	 		 }
//	 		 else continue;
            	}
        else continue;

    }
    if(max_count_best > max_count_best_threshold*(ball_info.r))	return true;
    else	 return false;
}







void print(IplImage* img, int x, int y, const char* text){
	    CvFont font;
	    double hscale = 2.0;
	    double vscale = 2.0;
	    int linewidth = 2;
	    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,hscale,vscale,0,linewidth);
	    CvScalar textColor =cvScalar(0,255,255);
	    CvPoint textPos =cvPoint(x, y);
	    cvPutText(img, text, textPos, &font,textColor);
}

























/*


#include <opencv2/opencv.hpp>
#include<stdio.h>
#include <math.h>
#include<cxmisc.h>
#include <sys/time.h>
#include <unistd.h>
#include "tcp.h"
using namespace std;
//const double PI = 2*asin(1);

//#define GONG_KONG
#define method1



#define hough_cmp_gt(l1,l2) (aux[l1] > aux[l2])
static CV_IMPLEMENT_QSORT_EX( icvHoughSortDescent32s, int, hough_cmp_gt, const int* );
unsigned char H,S,V;
unsigned char HSV[64][64][64]={0};
CvCapture* capture;
IplImage *Image,*ball_Image,* pImage,*white_Image,*white_cvtImage,*HSVImage,*HSVImagegray; 						//HSVImagegray和white_cvtImage分别是匹配完绿色和白色的灰度图
//whiteImage用来起匹配场地中的白色,全局变量直接在Color_Table()中赋值
//但是whiteImage要在大循环中至零,不然匹配的白点会累积
//cv::Ptr<CvMat> edges;										//第一种方法将轮廓画入这张图,用于第二种方法处理
CvMat* edges;
CvMemStorage* stor = cvCreateMemStorage(0);				//这个用于存储第一种方法中的轮廓
CvSeq *cont;
CvMat temp;
CvMat* mat;									//用于拷贝的矩阵,传入函数icvHoughCirclesGradient

typedef struct{
	int x;
	int y;
	float r;
	bool ball_find;
}Ball_info;

Ball_info ball_info;



int white_points_low_2 = 50;
int white_points_up_2 = 350;
int green_points_low_2 = 5;
int green_points_up_2= 200;
int green_circle_low_2 = 5;
int green_circle_up_2= 170;
int max_count_best_2 = 160;
int white_points_low_1 = 5;
int white_points_up_1 = 250;
int green_points_low_1 = 5;
int green_points_up_1= 150;
double t1,t2;

int Image_Init();
void Image_Run();
void Image_Show();
void Image_Clear();
IplImage* Color_Table(IplImage *src );
int Read_ColorTable();
bool get_ball_pos(CvMat *src);
bool get_ball_pos_1(CvMat *src);
//输入为单通道图像,我们用的时二值化图像
void print(IplImage* img, int x, int y, const char* text);
bool get_ball_pos_2( CvMat* img, float dp,
                         int min_radius, int max_radius,
                          float acc_threshold,float max_count_best_threshold);



double get_time()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	double t;
	t = tv.tv_sec + tv.tv_usec * pow(10, -6);
	return t;
}





//发送数据， 成功则返回未发送完的字节数， 失败则返回-1
int write_sock(int sock_fd, char *buf, int length)
{
	int byte_send;
	if((byte_send = write(sock_fd, buf, length)) == -1){
		std::cout<<"error in write sock."<<std::endl;
		return -1;
	}
	else{
		return length - byte_send;
	}
}

//发送一幅图像
bool image_send_one_frame(IplImage *img)
{
	int length = img->imageSize;
	int bytes_remain;
	char *img_data = img->imageData;
	while((bytes_remain = write_sock(client_sock, img_data, length)) != 0)
	{
		if(bytes_remain == -1)
			return false;
		img_data += length - bytes_remain;
		length = bytes_remain;
	}
	return true;
}

bool image_update()
{
	image_send_one_frame(ball_Image);
//	cvShowImage("simple-demo", show_Image);
	char ch = cvWaitKey(1);
	if(ch == ' '){
		return false;
	}
	return true;
}




int main ()
{
	int frames = 0;
	int detected = 0;
	if(Image_Init() == 0)
	{
		return 0;
	}
	cvWaitKey();
	t1 = get_time();
//	server_sock_init(&client_sock, 8888, "192.168.1.190");
    	while(1){
    	Image_Run();
    	frames++;
    	get_ball_pos(mat);

#ifdef GONG_KONG
    	image_send_one_frame(ball_Image);
#endif
	    Image_Show();
	char c=cvWaitKey(20);
//	if (ball_info.ball_find == true)
//		c = cvWaitKey();
	if (ball_info.ball_find == true)
		detected ++;
	if(get_time( ) - t1 >= 10.0)
		break;
	if(c=='s'){
				cvSaveImage("/home/monkeyd/picture/28.bmp",ball_Image);
				cvSaveImage("/home/monkeyd/picture/29.bmp",HSVImage);
				cvSaveImage("/home/monkeyd/picture/30.bmp",white_cvtImage);
				cvSaveImage("/home/monkeyd/picture/31.bmp",edges);
			}
	else if(c == 'c') continue;
	else if(c== 'b')
		break;
    	}
		cout<<"frames = "<<frames<<endl;
		cout<<"detected = "<<detected<<endl;
    	Image_Clear();
}







int Image_Init(){
	if(Read_ColorTable() == 0)
		{
			return 0;
		}
#ifdef GONG_KONG
	capture = cvCreateCameraCapture(0);
#else
		capture = cvCreateCameraCapture(1);
#endif
		Image = cvQueryFrame(capture);
#ifndef GONG_KONG
		cvNamedWindow("Image",1);
		cvNamedWindow("HSVImage",1);
#endif
		CvSize sz;																						//这里对图像改变大小
		sz.height = Image->height*0.5;
		sz.width = Image->width*0.5;
		pImage = cvCreateImage( sz, Image->depth, 3); //构造目标图象
		cvResize(Image,pImage,CV_INTER_LINEAR);
		ball_Image = cvCreateImage(cvGetSize(pImage),8,3);
		HSVImage=cvCreateImage(cvGetSize(pImage),8,3);
		white_Image=cvCreateImage(cvGetSize(pImage),pImage->depth,pImage->nChannels);
		white_cvtImage=cvCreateImage(cvGetSize(pImage),pImage->depth,1);
		HSVImagegray = cvCreateImage(cvGetSize(HSVImage),HSVImage->depth,1);
		cvCvtColor(pImage, pImage, CV_RGB2HSV);											//这些操作在while(1)中也有,这是对第一帧的操作,因为edges需要用mat来初始化,而mat需要HSVImagegray
		HSVImage = Color_Table(pImage);
		cvCvtColor(HSVImage,HSVImagegray,CV_RGB2GRAY);
		cvCvtColor(white_Image,white_cvtImage,CV_RGB2GRAY);
#ifndef GONG_KONG
		cvCreateTrackbar("white_points_low","Image",&white_points_low_2,300,NULL);
		cvCreateTrackbar("white_points_up","Image",&white_points_up_2,300,NULL);
		cvCreateTrackbar("green_points_low","Image",&green_points_low_2,300,NULL);
		cvCreateTrackbar("green_points_up","Image",&green_points_up_2,300,NULL);
		cvCreateTrackbar("green_circle_low","Image",&green_circle_low_2,300,NULL);
		cvCreateTrackbar("green_circle_up","Image",&green_circle_up_2,300,NULL);
		cvCreateTrackbar("max_count_best","Image",&max_count_best_2,300,NULL);
		cvCreateTrackbar("white_points_low","HSVImage",&white_points_low_1,300,NULL);
		cvCreateTrackbar("white_points_up","HSVImage",&white_points_up_1,300,NULL);
		cvCreateTrackbar("green_points_low","HSVImage",&green_points_low_1,300,NULL);
		cvCreateTrackbar("green_points_up","HSVImage",&green_points_up_1,300,NULL);
#endif
		mat = cvGetMat(HSVImagegray, &temp);  //深拷贝
		//canny算子求单像素二值化边缘，保存在edges变量中
		edges = cvCreateMat( mat->rows, mat->cols, CV_8UC1 );
		return 1;
}


void Image_Run(){
	Image = cvQueryFrame(capture);
	cvResize(Image,ball_Image,CV_INTER_LINEAR);
	cvResize(Image,pImage,CV_INTER_LINEAR);
	cvCvtColor(pImage, pImage, CV_RGB2HSV);
	HSVImage = Color_Table(pImage);
	cvCvtColor(HSVImage,HSVImagegray,CV_RGB2GRAY);
	cvCvtColor(white_Image,white_cvtImage,CV_RGB2GRAY);
	mat = cvGetMat(HSVImagegray, &temp);  //深拷贝
}

void Image_Show(){
//	if(ball_info.ball_find == true)cvCircle(ball_Image,cvPoint(ball_info.x,ball_info.y),ball_info.r,cvScalar(0,0,255),2,8,0);
#ifndef GONG_KONG
	cvShowImage("edges",edges);
	cvShowImage("Image",ball_Image);
	cvShowImage("HSVImage",HSVImage);
	cvShowImage("HSV",HSVImagegray);
	cvShowImage("white_cvtimage",white_cvtImage);
#endif
	cvSetZero(white_Image);
	cvSetZero(edges);
}

void Image_Clear(){
#ifndef GONG_KONG
	cvDestroyWindow("Image");
	cvDestroyWindow("HSVImage");
#endif
	cvClearMemStorage(stor);
	cvReleaseImage(&ball_Image);
	cvReleaseImage(&HSVImage);
	cvReleaseImage(&white_cvtImage);
	cvReleaseCapture(&capture);
}

IplImage* Color_Table(IplImage *src )
{
	IplImage *HSVImage = cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
	CvSize  size=cvGetSize(src);
//	long i;
	CvScalar s;
	int x,y;
	int nstep=src->widthStep;
	int nchannel=src->nChannels;
    for(y=0;y<size.height;y++)
    {
    	for(x=0;x<size.width;x++)
	    {
	    	//得到HS值
		    H=src->imageData[y*nstep+x*nchannel+1];
		    S=src->imageData[y*nstep+x*nchannel+2];
		    V=src->imageData[y*nstep+x*nchannel+0];
            H=H/4; S=S/4;V=V/4;
		  // 查表

		   switch (HSV[H][S][V])
		   {   case  0:														//代表其他颜色
			   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
			   s.val[0]=0;
				s.val[1]=0;
				s.val[2]=0;
				cvSet2D(white_Image,y,x,s);//set the (i,j) pixel value
			   break;
   	   case  1:																//代表绿色
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
			   s.val[0]=0;
				   s.val[1]=0;
				   s.val[2]=0;
				   cvSet2D(white_Image,y,x,s);//set the (i,j) pixel value
		       break;
	   	   case  2:															//代表白色
	   	   	   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
				   s.val[0]=255;
				   s.val[1]=255;
				   s.val[2]=255;
				   cvSet2D(white_Image,y,x,s);//set the (i,j) pixel value
			       break;
		   	   default:
//		   		     cout<<"                                default     colour="<<HSV[H][S][V]+0<<endl;
		   		      break;
		   }
	    }
    }
    cvSmooth(HSVImage,HSVImage,CV_MEDIAN,3);
   // cvSmooth(white_Image,white_Image,CV_MEDIAN,3);
    return HSVImage;
}


int Read_ColorTable()
{
	long x,y,z;
	long fp_position;
		FILE  *fp;
		  if((fp = fopen("ColourTable_HSV.txt", "r+")) == NULL){
			  cout<<"can't open the file\n"<<endl;
		  	  return 0;
		  }
		        	 for( x=0;x<64;x++)
		        		  		{
		        				for( y=0;y<64;y++)
		        					{
		        					for( z=0;z<64;z++)
		        					{
		                            rewind(fp);
		        					fp_position=ftell(fp);
//		        					cout<<"重置指针位置"<<fp_position<<endl;
		                            fseek(fp,4*long(x*64*64+y*64+z),0);
		        						fread(&HSV[x][y][z],sizeof(int), 1,fp) ;
//		        						cout<<"colour="<<HSV[x][y][z]+0<<endl;
		        					}
		        					}
		        		  		}

				fclose(fp);
				return 1;
}


bool get_ball_pos(CvMat *src){
//#ifdef	method1
			if(get_ball_pos_1(src) == true){
					ball_info.ball_find = true;
	//		    	cout<<"t1 = "<<get_time()-t1<<endl;
	//				cout<<"ball.r = "<<ball_info.r<<endl;
					 cvCircle(ball_Image,cvPoint(ball_info.x,ball_info.y),ball_info.r,cvScalar(0,0,255),2,8,0);

					 return true;
				}
//else
				if(get_ball_pos_2(src,1.2,(ball_Image->width/40),ball_Image->width/4,1.0,((float)max_count_best_2/100)) == true){
					ball_info.ball_find = true;
	//		    	cout<<"t2 = "<<get_time()-t2<<endl;
	//				cout<<"ball.r = "<<ball_info.r<<endl;
					 cvCircle(ball_Image,cvPoint(ball_info.x,ball_info.y),ball_info.r,cvScalar(0,255,255),2,8,0);
					 return true;
				}
//#endif
		    else{
		    	    	 ball_info.ball_find = false;
		   // 	    	 cout<<"can't find the ball"<<endl;
		    	    	 return false;
		    	    }


}

bool get_ball_pos_1(CvMat *src){
	const double area_ratio_limit = 0.4;	//可调
	CvPoint2D32f center;
	float real_radius=0;
	double cont_area;
	double circle_area;
	double area_ratio = 0;
	double max_area_ratio = 0;
	float max_radius = 0;
	int x, y, r;
	//t1 = get_time();
	cvFindContours(src, stor, &cont, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
	for (; cont; cont = cont->h_next)
	{

		if(cvMinEnclosingCircle(cont, &center, &real_radius) <= 0){
			continue;
		}
		if(cont->total < 10 || cont->total > 300){
			continue;
		}
//		cout<<"cont_total="<<cont->total<<endl;
		if(real_radius < 8 || real_radius > 150){
			continue;
		}
		cvDrawContours(edges,cont,cvScalar(255,255,255),cvScalar(255,255,255),0);
		cont_area = cvContourArea(cont);
			circle_area = CV_PI * real_radius * real_radius;
			area_ratio = cont_area/circle_area;
			if(area_ratio < area_ratio_limit){
				continue;
			}
			if(area_ratio < max_area_ratio || real_radius < max_radius){
				continue;
			}
			max_area_ratio = area_ratio;
						max_radius = real_radius;
						   int white_points_in_circle=0;
							int green_points_in_circle = 0;
							int nchannel = white_cvtImage->nChannels;
							int nstep = white_cvtImage->widthStep;
						        	for(int l = -real_radius;l  <= real_radius ;l++){
						        		int d = sqrt(real_radius*real_radius - l*l);
						        		for(int d_tmp = -d;d_tmp < d ; d_tmp++){
						        			 int cy_l = center.y+l;
						        			 int cx_d = center.x+d_tmp;
						        			 if(cy_l<1 || cy_l>white_cvtImage->height-1) continue;
						        			if(cx_d<1 || cx_d>white_cvtImage->width-1) continue;
						        			if(white_cvtImage->imageData[cy_l*nstep+cx_d*nchannel]!=0){
						        				white_points_in_circle++;
			//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
						        			}
						        			if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0){
						        				green_points_in_circle++;
			//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
						        			}
						        		}
						        	}
						        	if((white_points_in_circle>real_radius*real_radius*((float)white_points_low_1/100))&&(white_points_in_circle<real_radius*real_radius*((float)white_points_up_1/100))&&
						        		(green_points_in_circle>real_radius*real_radius*((float)green_points_low_1/100))&&(green_points_in_circle<real_radius*real_radius*((float)green_points_up_1/100)))
						        								        	{
						        												ball_info.x = (center.x + 0.5);
						        												ball_info.y =  (center.y + 0.5);
						        												ball_info.r = (real_radius+0.5);
//						        												cout<<"r1="<<ball_info.r<<endl;
						        												return true;
						        								        	}
						        			}

						        				return false;




}

bool get_ball_pos_2( CvMat* img, float dp,
                         int min_radius, int max_radius,
                          float acc_threshold,float max_count_best_threshold)
{
	//参数：
	//img: 输入图像
	//dp: 识别精度,1.0表示按照原图精度
	//min_dist: 圆心点位置识别精度
	//min_radius: 所需要找的圆的最小半径
	//max_radius：所需要找的圆的最大半径
	//canny_threshold：canny算子的高阀值
	//acc_threshold：累加器阀值，计数大于改阀值的点即被认为是可能的圆心
	//circles: 保存找到的符合条件的所有圆
	//circles_max: 最多需要的找到的圆的个数
    const int SHIFT = 10, ONE = 1 << SHIFT;
    cv::Ptr<CvMat> dx, dy;
 //   cv::Ptr<CvMat> edges, edge_initial,accum, dist_buf;
    cv::Ptr<CvMat> accum, dist_buf;
    std::vector<int> sort_buf;
    cv::Ptr<CvMemStorage> storage;

    int x, y, i, j, k, center_count, nz_count;
    float min_radius2 = (float)min_radius*min_radius;
    float max_radius2 = (float)max_radius*max_radius;
    int rows, cols, arows, acols;
    int astep, *adata;
    float* ddata;
    CvSeq *nz, *centers;
    float idp, dr;
    CvSeqReader reader;



  //  t2 = get_time();
    cvFindContours(img, stor, &cont, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

    for (; cont; cont = cont->h_next)
    {
    	if(cont->total<50){
    		continue;
    	}
    	cvDrawContours(edges,cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0);
    }

    //sobel算子求水平和垂直方向的边缘，用于计算边缘点的法线方向
    dx = cvCreateMat( img->rows, img->cols, CV_16SC1 );
    dy = cvCreateMat( img->rows, img->cols, CV_16SC1 );
    cvSobel( img, dx, 1, 0, 3 );
    cvSobel( img, dy, 0, 1, 3 );

    //dp表示识别精度
    if( dp < 1.f )
        dp = 1.f;
    idp = 1.f/dp;
    //accum用作累加器，包含图像中每一个点的计数。图像中每一个点都有一个计数，点的计数表示每一个canny边缘点法线方向上，
    //到该点距离为R的边缘点的个数，初始化为0
    accum = cvCreateMat( cvCeil(img->rows*idp)+2, cvCeil(img->cols*idp)+2, CV_32SC1 );
    cvZero(accum);

    storage = cvCreateMemStorage();
    nz = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
    //centers用于保存可能的圆心点
    centers = cvCreateSeq( CV_32SC1, sizeof(CvSeq), sizeof(int), storage );

    rows = img->rows;
    cols = img->cols;
    arows = accum->rows - 2;
    acols = accum->cols - 2;
    adata = accum->data.i;
    astep = accum->step/sizeof(adata[0]);
    // Accumulate circle evidence for each edge pixel
    //以下这个循环用于获取所有可能的圆边缘点，存储在nz中，同时设置
    //累加器中的值
    for( y = 0; y < rows; y++ )
    {
        const uchar* edges_row = edges->data.ptr + y*edges->step;
        const short* dx_row = (const short*)(dx->data.ptr + y*dx->step);
        const short* dy_row = (const short*)(dy->data.ptr + y*dy->step);

        for( x = 0; x < cols; x++ )
        {
            float vx, vy;
            int sx, sy, x0, y0, x1, y1, r;
            CvPoint pt;
     //vx,vy均为梯度
            vx = dx_row[x];
            vy = dy_row[x];

            if( !edges_row[x] || (vx == 0 && vy == 0) )
                continue;

            float mag = sqrt(vx*vx+vy*vy);
            assert( mag >= 1 );
            //sx表示cos, sy表示sin
            sx = cvRound((vx*idp)*ONE/mag);
            sy = cvRound((vy*idp)*ONE/mag);

            x0 = cvRound((x*idp)*ONE);
            y0 = cvRound((y*idp)*ONE);
            // Step from min_radius to max_radius in both directions of the gradient
            //循环两次表示需要计算两个方向，法线方向和法线的反方向
            for(int k1 = 0; k1 < 2; k1++ )
            {
                //半径方向的水平增量和垂直增量
                x1 = x0 + min_radius * sx;
                y1 = y0 + min_radius * sy;
                //在法线方向和反方向上，距离边缘点的距离为输入的最大半径和最小半径范围内找点
                //每找到一个点，该点的累加器计数就加1
                for( r = min_radius; r <= max_radius; x1 += sx, y1 += sy, r++ )
                {
                    int x2 = x1 >> SHIFT, y2 = y1 >> SHIFT;
                    if( (unsigned)x2 >= (unsigned)acols || (unsigned)y2 >= (unsigned)arows)
                        break;
                    adata[y2*astep + x2]++;
                }
                //反方向
                sx = -sx; sy = -sy;
            }
            //保存可能的圆边缘点
            pt.x = x; pt.y = y;
            cvSeqPush( nz, &pt );
        }
    }

    nz_count = nz->total;
//    cout<<"边缘点个数"<<nz_count<<endl;
    if( !nz_count )
    {
    	cout<<"problem 1"<<endl;
        return false;
    }
        //Find possible circle centers
    //累加器中，计数大于阀值的点，被认为可能的圆心点。因为计算各点计数过程中，距离有误差，所以
   //在与阀值比较时，该点计数先要与4邻域内的各个点的计数比较，最大者才能和阀值比较。可能的圆心
   //点保存在centers中
    for( y = 1; y < arows - 1; y++ )
    {
        for( x = 1; x < acols - 1; x++ )
        {
            int base = y*(acols+2) + x;
            if( adata[base] > acc_threshold &&
                adata[base] > adata[base-1] && adata[base] > adata[base+1] &&
                adata[base] > adata[base-acols-2] && adata[base] > adata[base+acols+2])
//				&&adata[base] > adata[base-acols]&&adata[base] > adata[base-acols-2]&&
//				adata[base] > adata[base+acols+1]&&adata[base] > adata[base+acols])									//扫四临域,不扫八临域
              {
//			    cout<<"满足条件的Center的累加器"<<adata[base]<<endl;
            	if (adata[base]> 10)
                cvSeqPush(centers, &base);
              }
        }
    }
    center_count = centers->total;
//    cout<< "满足条件的center共有"<<center_count<<endl;
    if( !center_count||center_count>2000 )
    {
    	cout<<"problem 2"<<endl;
        return false;
    }
    sort_buf.resize( MAX(center_count,nz_count) );
    //链表结构的certers转化成连续存储结构sort_buf
    cvCvtSeqToArray( centers, &sort_buf[0] );
    //经过icvHoughSortDescent32s函数后，以sort_buf中元素作为adata数组下标,
   //adata中的元素降序排列, 即adata[sort_buf[0]]是adata所有元素中最大的,
   //adata[sort_buf[center_count-1]]是所有元素中最小的
    icvHoughSortDescent32s( &sort_buf[0], center_count, adata );
    cvClearSeq( centers );
    //经过排序后的元素，重新以链表形式存储到centers中
    cvSeqPushMulti( centers, &sort_buf[0], center_count );

    dist_buf = cvCreateMat( 1, nz_count, CV_32FC1 );
    ddata = dist_buf->data.fl;

    dr = dp;
    // For each found possible center
    // Estimate radius and check support
    //对于每一个可能的圆心点，计算所有边缘点到该圆心点的距离。由于centers中的
   //元素已经经过前面排序，所以累加器计数最大的可能圆心点最先进行下面的操作




//    if(center_count>50){												//扫的圆心点数不能太少,特别是距离球比较远时
  //  	center_count = 50;
 //  }

    int max_count_best = 0;
    for(i=0;i< center_count;i++)

    {
        if(center_count>500) i=i+1;														//如果圆心点数太多,运算量就会很大,就让它隔几个点找一次.
        if(center_count>1000) i=i+1;
        int ofs = *(int*)cvGetSeqElem( centers, i );

        y = ofs/(acols+2);
        x = ofs - (y)*(acols+2);

        //Calculate circle's center in pixels
        float cx = (float)((x + 0.5f)*dp), cy = (float)(( y + 0.5f )*dp);

        float start_dist, dist_sum;
        float r_best = 0;
        int max_count = 0;
        // Check distance with previously detected circles
        //如果该可能的圆心点和已经确认的圆心点的距离小于阀值，则表示
       //这个圆心点和已经确认的圆心点是同一个点


        // Estimate best radius
        cvStartReadSeq( nz, &reader );

        //求所有边缘点到当前圆心点的距离，符合条件的距离值保存在ddata中
        for( j = k = 0; j < nz_count; j++ )
        {
            CvPoint pt;
            float _dx, _dy, _r2;
            CV_READ_SEQ_ELEM( pt, reader );
            _dx = cx - pt.x; _dy = cy - pt.y;
            _r2 = _dx*_dx + _dy*_dy;
            if(min_radius2 <= _r2 && _r2 <= max_radius2 )
            {
                ddata[k] = _r2;
                sort_buf[k] = k;
                k++;
            }
        }

        int nz_count1 = k, start_idx = nz_count1 - 1;
        if( nz_count1 == 0 )
            continue;
        dist_buf->cols = nz_count1;
        cvPow( dist_buf, dist_buf, 0.5 );
        //经过如下处理后，以sort_buf中元素作为ddata数组下标,ddata中的元素降序排列,
       //即ddata[sort_buf[0]]是ddata所有元素中最大的, ddata[sort_buf[nz_count1-1]]
       //是所有元素中最小的
        icvHoughSortDescent32s( &sort_buf[0], nz_count1, (int*)ddata );
        //对所有的距离值做处理，求出最可能圆半径值，max_count为到圆心的距离为最可能半径值的点的个数
        dist_sum = start_dist = ddata[sort_buf[nz_count1-1]];

        for( j = nz_count1 - 2; j >= 0; j-- )
        {
            float d = ddata[sort_buf[j]];

            if( d > max_radius )
                break;

            if( d - start_dist > dr )
            {
                float r_cur = ddata[sort_buf[(j + start_idx)/2]];
                if( (start_idx - j)*r_best >= max_count*r_cur ||
                    (r_best < FLT_EPSILON && start_idx - j >= max_count) )
                {
                    r_best = r_cur;
                    max_count = start_idx - j;
                }
                start_dist = d;
                start_idx = j;
                dist_sum = 0;
            }
            dist_sum += d;
        }
        // Check if the circle has enough support
        //max_count大于阀值，表示这几个边缘点构成一个圆
        int white_points_in_circle=0;
        int green_points_in_circle = 0;
        int green_circle_in_circle = 0;
    	int nchannel = white_cvtImage->nChannels;
    	int nstep = white_cvtImage->widthStep;

        if( max_count > int(acc_threshold*r_best) )
        {
//        	CvScalar s;
        	for(int l = -r_best;l  <= r_best ;l++){
        		int d = sqrt(r_best*r_best - l*l);
        		for(int d_tmp = -d;d_tmp <= d ; d_tmp++){
 //       			s.val[2] = 255;
//					s.val[1] = 0;
//					s.val[0] = 0;
					 int cy_l = cy+l;
					 int cx_d = cx+d_tmp;
					 if(cy_l<3 || cy_l>white_cvtImage->height-3) continue;
					 if(cx_d<3 || cx_d>white_cvtImage->width-3) continue;
        			if((d_tmp == -d)||(d_tmp == d)){
//        				if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0)	green_circle_in_circle++;
        				if(HSVImagegray->imageData[cy_l*nstep+(cx_d-2)*nchannel] ==0) green_circle_in_circle++;
        				if(HSVImagegray->imageData[cy_l*nstep+(cx_d+2)*nchannel] ==0) green_circle_in_circle++;
        			}

        			if(cy_l<1 || cy_l>white_cvtImage->height-1) continue;
        			if(cx_d<1 || cx_d>white_cvtImage->width-1) continue;
        			if(white_cvtImage->imageData[cy_l*nstep+cx_d*nchannel]!=0){
        				white_points_in_circle++;
 //       				cvSet2D(ball_Image,cy_l,cx_d,s);
        			}
//        			s.val[2] = 0;
 //       			s.val[1] = 255;
//					s.val[0] = 0;
        			if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0){
        				green_points_in_circle++;
//        				cvSet2D(ball_Image,cy_l,cx_d,s);
        			}
        		}
        	}
	 		 if((white_points_in_circle>r_best*r_best*((float)white_points_low_2/100))&&(white_points_in_circle<r_best*r_best*((float)white_points_up_2/100))&&
	 				 (green_points_in_circle>r_best*r_best*((float)green_points_low_2/100))&&(green_points_in_circle<r_best*r_best*((float)green_points_up_2/100))&&
					 (green_circle_in_circle > r_best*CV_PI*2*((float)green_circle_low_2/100))&&(green_circle_in_circle < r_best*CV_PI*2*((float)green_circle_up_2/100)))
	 		 {
//	 			cout<<"max_count/r_best="<<(float)max_count/r_best<<endl;
	 			if(max_count > max_count_best){
	 								  ball_info.x = cx;
	 								  ball_info.y = cy;
	 								  ball_info.r = r_best;
	 								  max_count_best = max_count;
//					cout<<"i="<<i<<endl;
//				  cout<<"r_best="<<r_best<<endl;
//					cout<<"max_count_best/r_best="<<(float)max_count_best/r_best<<endl;
//				  cout<<"white_points_in_circle_ratio="<<((float)white_points_in_circle)/(r_best*r_best)<<endl;
//				  cout<<"green_points_in_circle_ratio="<<((float)green_points_in_circle)/(r_best*r_best)<<endl;
//				  cout<<"green_circle_in_circle="<<((float)green_circle_in_circle)/(r_best*CV_PI*2)<<endl;

				  }
	 		 }
	 		 else continue;
            	}
        else continue;

    }
    if(max_count_best > max_count_best_threshold*(ball_info.r))	return true;
    else	 return false;
}







void print(IplImage* img, int x, int y, const char* text){
	    CvFont font;
	    double hscale = 2.0;
	    double vscale = 2.0;
	    int linewidth = 2;
	    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,hscale,vscale,0,linewidth);
	    CvScalar textColor =cvScalar(0,255,255);
	    CvPoint textPos =cvPoint(x, y);
	    cvPutText(img, text, textPos, &font,textColor);
}

*/







































/*

#include <opencv2/opencv.hpp>																		//颜色表
#include<fcntl.h>
#include<unistd.h>
#include<stdio.h>
using namespace std;

unsigned char  H,S,V;
unsigned char HSV[64][64][64]={0};
CvPoint s_cvPrePoint;
CvPoint cvCurrPoint;
//IplImage *pSrcImage=0;
IplImage  *dst=0;
//IplImage *SrcImage;
CvRect rect;
int others=0,green=1,white=2,black=3;


//鼠标点选感兴趣区域
void on_mouse(int event,int x, int y, int flags, void* pSrcImage)
{
    switch (event)
    {
    case CV_EVENT_LBUTTONDOWN:
        s_cvPrePoint = cvPoint(x, y);

        break;

    case  CV_EVENT_RBUTTONDOWN:

    		rect.x=0;
        	rect.y=0;
        	rect.width=0;
        	rect.height=0;
        	cvCurrPoint = cvPoint(x, y);
//        cvRectangle((IplImage*)pSrcImage, s_cvPrePoint, cvCurrPoint,CV_RGB(100, 50, 100),1,8,0);

        	cvShowImage("src", (IplImage*)pSrcImage);
        	rect.x=s_cvPrePoint.x;
            rect.y=s_cvPrePoint.y;
            rect.width=cvCurrPoint.x-s_cvPrePoint.x;
            rect.height=cvCurrPoint.y-s_cvPrePoint.y;
            if(rect.width<0){
            	rect.x += rect.width;
            	rect.width *= -1;
            }
            if(rect.height<0){
            	rect.y += rect.height;
            	rect.height *= -1;
            }

            cvSetImageROI((IplImage*)pSrcImage,rect);

			dst=cvCreateImage(cvSize(rect.width,rect.height),8,3);
            cvCopy((IplImage*)pSrcImage,dst);
//			cout<<"run here"<<endl;
			cvResetImageROI((IplImage*)pSrcImage );
			cvShowImage("src", pSrcImage);
        	cvShowImage("dst", dst);
//        	cvWaitKey();
        break;

      default :

        break;
    }
}
//将图像中的ＨＳＶ值写入颜色表
void save_HSVColourTable()
{  long fp_position;
   unsigned char H,S,V;
	int x,y;
	FILE  *fp;
    int nstep=dst->widthStep;
    int nchannel=dst->nChannels;
//	cout<<"run here"<<endl;
	if((fp = fopen("ColourTable_HSV.txt", "r+")) == NULL)
	   {cout<<"can't open the file"<<endl;}
	  for(y=0;y<rect.height;y++)
	  			{
					for( x=0;x<rect.width;x++)
						{
							H=dst->imageData[y*nstep+x*nchannel+1];
							S=dst->imageData[y*nstep+x*nchannel+2];
							V=dst->imageData[y*nstep+x*nchannel+0];
							H=H/4; S=S/4;V=V/4;
                           rewind(fp);
//							fp_position=ftell(fp);
//							cout<<"重置指针位置"<<fp_position<<endl;
                           fseek(fp,4*long(H*64*64+S*64+V),0);
							fp_position=ftell(fp);
							cout<<"调整至写入位置"<<fp_position<<endl;
							fwrite(&white, sizeof(int), 1,fp) ;
							cout<<white<<endl;
							cout<<H+0<<"    "<<S+0<<endl;
//							fp_position=ftell(fp);
//							cout<<"指针写入后位置"<<fp_position<<endl;
						}
	  			}
	fclose(fp);
}


IplImage* Color_Table(IplImage *src )
{

	IplImage *HSVImage = cvCreateImage(cvGetSize(src),src->depth,src->nChannels);
	CvSize  size=cvGetSize(src);
//	long i;
	CvScalar s;
	int x,y;
	int nstep=src->widthStep;
	int nchannel=src->nChannels;

    for(y=0;y<size.height;y++)
    {
    	for(x=0;x<size.width;x++)
	    {
	    	//得到HS值
		    H=src->imageData[y*nstep+x*nchannel+1];
		    S=src->imageData[y*nstep+x*nchannel+2];
		    V=src->imageData[y*nstep+x*nchannel+0];
            H=H/4; S=S/4;V=V/4;

		  // 查表
//			   colour=read_HSVColourTable(H,S);
//           cout<<++i<<endl;
//		    cout<<H+0<<"   "<<S+0<<"    "<<V+0<<endl;


		   switch (HSV[H][S][V])
		   {   case  0:
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
			   break;
   	   case  1:
			   s.val[0]=0;
			   s.val[1]=255;
			   s.val[2]=0;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
		       break;
	   	   case  2:
	   	   	   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
			       break;
	   	   case 3:
				s.val[0]=0;
				s.val[1]=0;
				s.val[2]=255;
				cvSet2D(HSVImage,y,x,s);//set the (i,j) pixel value
				   break;
		   	   default:
		   		     cout<<"                                default     colour="<<HSV[H][S][V]+0<<endl;
		   		      break;
		   }
	    }
    }
    cvSmooth(HSVImage,HSVImage,CV_MEDIAN,3);
    return HSVImage;
}


int Read_ColorTable()
{
	long x,y,z;
	long fp_position;
		FILE  *fp;
		  if((fp = fopen("ColourTable_HSV.txt", "r+")) == NULL){
			  cout<<"can't open the file\n"<<endl;
		  	  return 0;
		  }
		        	 for( x=0;x<64;x++)
		        		  		{
		        				for( y=0;y<64;y++)
		        					{
		        					for( z=0;z<64;z++)
		        					{
		                            rewind(fp);
		        					fp_position=ftell(fp);
//		        					cout<<"重置指针位置"<<fp_position<<endl;
		                            fseek(fp,4*long(x*64*64+y*64+z),0);
		        						fread(&HSV[x][y][z],sizeof(int), 1,fp) ;
//		        						cout<<"colour="<<HSV[x][y][z]+0<<endl;
		        					}
		        					}
		        		  		}

				fclose(fp);
				return 1;
}

int main()
{
	Read_ColorTable();
	CvCapture *capture = cvCreateCameraCapture(1);
	IplImage *pSrcImage = cvQueryFrame(capture);
    //cvCvtColor(SrcImage, pSrcImage, CV_RGB2HSV);
    cvNamedWindow("src", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("src_HSV", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("colorlut_img",CV_WINDOW_AUTOSIZE);
    cvNamedWindow("dst",CV_WINDOW_AUTOSIZE);
    //cvShowImage("src",SrcImage);
    //cvShowImage("src_HSV", pSrcImage);
	cvSetMouseCallback("src_HSV", on_mouse, (IplImage*)pSrcImage);
	IplImage *colorlut_img = cvCreateImage(cvGetSize(pSrcImage),pSrcImage->depth,3);
	IplImage *SrcImage = cvCreateImage(cvGetSize(pSrcImage),pSrcImage->depth,3);
	colorlut_img = Color_Table(pSrcImage);
	cvShowImage("colorlut_img",colorlut_img);














































//颜色表初始化
    long fp_position;
	FILE  *fp;
	if((fp = fopen("ColourTable_HSV.txt", "w+")) == NULL)
	   {printf("can't open the file\n");}
	for( H=0;H<=64;H++)
  			{
				for(S=0;S<=64;S++)
					{
                   		for(V=0;V<=64;V++)
					{
                          rewind(fp);
                           fseek(fp,4*long(H*64*64+S*64+V),0);
							cout<<"指针位置"<<fp_position<<endl;
						fwrite(&others, sizeof(int), 1,fp) ;
						cout<<H+0<<"    "<<S+0<<endl;
						fp_position=ftell(fp);
						cout<<"指针位置"<<fp_position<<endl;
                 }
					}
	  			}
	fclose(fp);
















































	while(1){

	char c = cvWaitKey(33);
	if(c == 'c'){
		pSrcImage = cvQueryFrame(capture);
		cvCopy(pSrcImage,SrcImage);
		cvCvtColor(pSrcImage, pSrcImage, CV_RGB2HSV);
		cvShowImage("src", SrcImage);
		cvShowImage("src_HSV", pSrcImage);
		colorlut_img = Color_Table(pSrcImage);
		cvShowImage("colorlut_img",colorlut_img);
	}
	if(c == 's')	save_HSVColourTable();
	if(c == 'f')	cvSaveImage("/home/monkeyd/picture/5.bmp",SrcImage);
	if(c == 'g')   cvSaveImage("/home/monkeyd/picture/6.bmp",pSrcImage);
	if(c == 'h')   cvSaveImage("/home/monkeyd/picture/7.bmp",colorlut_img);
	if(c == 'r')	Read_ColorTable();
	if(c == 'b')break;
	}
//	 write_HSV();
//	 read_HSV();

//    cvCvtColor(dst, dst, CV_RGB2HSV);
    cvDestroyWindow("src");
    cvDestroyWindow("dst");
    cvReleaseImage(&pSrcImage);
//    cvReleaseImage(&dst);
    return 0;
}






*/












