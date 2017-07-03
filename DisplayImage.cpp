
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>   
#include <unistd.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> 
#include <termios.h> 
#include <errno.h> 
#include <string.h>
#include <getopt.h>
#include <string>

using namespace cv;

#define debug
#define r 35
#define FALSE -1
#define TRUE 0

string window_name = "Capture";
int lowThreshold = 30;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
Mat src_origin,src,src_gray;
Mat detected_edges;
Mat src_roi;
Mat hsv_roi;

vector<vector<Point> > contours;
vector<Moments> mu;
vector<Point2f> mc;
vector<char> filter;//1 big enough
vector<Mat> hsv_split_roi;
int x_count , y_count ,no_count=0;

short center_x,center_y;
//vector<double> length; don not why
double	closest_length = 160000;
int closest_id;
int length_tmp;
int closest_x,closest_y;

float cmp_rect, cmp_circle;
Point2f center;
float radius;

float area_unkown;
float area_rect;
float area_circle;

int r_count , g_count , b_count ;
char h, s, v;
short roi_up,roi_left;

	char *c7="AT+CIPSEND=2,2\r\n\r";
	char *rs="rs\r\n\r";
	char *gs="gs\r\n\r";
	char *bs="bs\r\n\r";
	char *rc="rc\r\n\r";
	char *gc="gc\r\n\r";
	char *bc="bc\r\n\r";
	char *no="no\r\n\r";
	char *n="\r";

int speed_arr[] = {B921600,B460800,B230400,B115200,B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {921600,460800,230400,115200,57600, 38400, 19200, 9600, 4800, 2400, 1200, 300, 38400,
19200, 9600, 4800, 2400, 1200, 300, };

int fd;
char write_buffer[512];

/*
int send_char(char *send_string)
{
	int i;
//	sleep(3);
	i=write(fd,c7,strlen(c7));
    if(i > 0)
    {
    	printf("write success start send\n");

    }
//	sleep(1);
	usleep(300000);
	i=write(fd,send_string,strlen(send_string));
    if(i > 0)
    {
		printf("write success_send_string\n");
    }
}
*/
int send_char(char *send_string)
{
	int i;
	i=write(fd,send_string,strlen(send_string));
//    i=write(fd,send_string,12);
    if(i > 0)
    {
		printf("write success_send_string\n");
    }
}

void set_speed(int fd, int speed)
{
	int i;
	int status;
	struct termios Opt;
	tcgetattr(fd, &Opt);
	for ( i= 0; i < sizeof(speed_arr) / sizeof(int); i++) 
	{
		if (speed == name_arr[i])
		{
			tcflush(fd, TCIOFLUSH);
			cfsetispeed(&Opt, speed_arr[i]);
			cfsetospeed(&Opt, speed_arr[i]);
			status = tcsetattr(fd, TCSANOW, &Opt);
			if (status != 0) 
			{
				perror("tcsetattr fd1");
				return;
			}
			tcflush(fd,TCIOFLUSH);
		}
	}
}

int set_Parity(int fd,int databits,int stopbits,int parity,int flowctrl)
{
	struct termios options;
	if ( tcgetattr( fd,&options) != 0)
	{
		perror("SetupSerial 1");
		return(FALSE);
	}
	options.c_cflag &= ~CSIZE;
	switch (databits) 
	{
		case 7:
		options.c_cflag |= CS7;
		break;
		case 8:
		options.c_cflag |= CS8;
		break;
		default:
		fprintf(stderr,"Unsupported data sizen"); return (FALSE);
	}
	switch (parity)
	{
	case 'n':
	case 'N':
	options.c_cflag &= ~PARENB; /* Clear parity enable */
	options.c_iflag &= ~INPCK; /* Enable parity checking */
	break;
	case 'o':
	case 'O':
	options.c_cflag |= (PARODD | PARENB); 
	options.c_iflag |= INPCK; /* Disnable parity checking */
	break;
	case 'e':
	case 'E':
	options.c_cflag |= PARENB; /* Enable parity */
	options.c_cflag &= ~PARODD; 
	options.c_iflag |= INPCK; /* Disnable parity checking */
	break;
	case 'S':
	case 's': /*as no parity*/
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;break;
	default:
	fprintf(stderr,"Unsupported parityn");
	return (FALSE);
	}


	switch (stopbits)
	{
	case 1:
	options.c_cflag &= ~CSTOPB;
	break;
	case 2:
	options.c_cflag |= CSTOPB;
	break;
	default:
	fprintf(stderr,"Unsupported stop bitsn");
	return (FALSE);
	}
	if(flowctrl)
		options.c_cflag |= CRTSCTS;
	else
		options.c_cflag &= ~CRTSCTS;
/* Set input parity option */
	if (parity != 'n')
	options.c_iflag |= INPCK;
	tcflush(fd,TCIFLUSH);
	options.c_cc[VTIME] = 150; 
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */

	if (tcsetattr(fd,TCSANOW,&options) != 0)
	{
		perror("SetupSerial 3");
		return (FALSE);
	}
	return (TRUE);
}


/*********************************************************************/
int OpenDev(char *Dev)
{
	int fd = open( Dev, O_RDWR );
//| O_NOCTTY | O_NDELAY
	if (-1 == fd)
	{
		perror("Can't Open Serial Port");
		return -1;
	}
	else
	return fd;
}

int main()
{

	int nread,i;
	char buff[512];
	char read_buffer[512];
	int flowctrl = 0;
	pid_t pid;
	
	char *dev = "/dev/ttyUSB0"; 
	char *xmit = "com test by guo jia wei\r\n";
	char *c1="AT+RST\r\n\r";
	char *c2="AT+CWMODE=1\r\n\r";
	char *c3="AT+CWJAP=\"TEST\",\"123456123456\"\r\n\r";
	char *c4="AT+CIFSR\r\n\r";
	char *c5="AT+CIPMUX=1\r\n\r";
	char *c6="AT+CIPSTART=2,\"TCP\",\"192.168.4.1\",8080\r\n\r";
	

	char c;
	CvCapture* capture;
	Mat frame;
	capture = cvCaptureFromCAM( -1 );
	for (char i = 1; i <= 3; i++)
	{
		src_origin = cvQueryFrame(capture);
	}
	src_origin = cvQueryFrame(capture);
/*
	short x_left = src_origin.cols / 4;//120
	short y_up = src_origin.rows / 6;//80
	short length = src_origin.cols / 2;
	short hight = src_origin.rows / 2 - 50 ;
*/
	short x_left = 0;//120
	short y_up = 0;//80
	short length = src_origin.cols ;
	short hight = src_origin.rows / 2 + 40 ;
	int contours_size;

    int delat_x,delat_y;
    char *left_right_flag;
	char *up_down_flag;
	float angle;
	sleep(1);
	fd = OpenDev(dev);//open what you use to send
	memset(read_buffer, 0, sizeof(read_buffer));
	if(fd > 0 )
	{
		printf("open success\n");
		set_speed(fd,115200);
	}
	else
	{
		fprintf(stderr,"error opening %s\n",dev,strerror(errno));
		exit(1);
	}
	if (set_Parity(fd,8,1,'N',flowctrl) == FALSE)
	{
		fprintf(stderr,"Set Parity Errorn");
		close(fd);
		exit (1);
	}
	printf("send: %s\n",xmit);

	bool stop(false);
	while( !stop )
	{
		src_origin = cvQueryFrame( capture );
#ifdef debug
		if( src_origin.empty() )
		{ printf(" --(!) No captured frame -- Break!\n"); break; }
		else
		{
			printf("get frame!\n");
			imshow( window_name, src_origin );
		}
#endif
//set ROI
		if( !src_origin.empty() )
		src_origin(Rect(x_left, y_up, length, hight)).copyTo(src);
//set center
		if( !src.empty() )
		{
			center_x = src.cols / 2;
			center_y = src.rows / 5 * 3;
		}
//color to gray
		if( !src.empty() )
		cvtColor(src, src_gray, CV_BGR2GRAY);
#ifdef debug
			namedWindow("src_gray");
			imshow("src_gray", src_gray);
			namedWindow("src");
			imshow("src", src);//正常显示
#endif
//canny
		if( !src_gray.empty() )
		blur(src_gray, detected_edges, Size(3, 3));
		Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);
//findContours		
		findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		contours_size=contours.size();
//		if(contours_size==0) continue;
//find mc
		mu.resize(contours_size);//
		for (unsigned int i = 0; i < contours_size; i++)
		{
			mu[i] = moments(contours[i], false);
		}
		mc.resize(contours_size);
		for (unsigned int i = 0; i < contours_size; i++)
		{
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}
//x_count:what is found y_count:the number that is big enough
		filter.resize(contours_size);
		x_count = contours_size;
		y_count = 0;
		if (x_count > 0)
		{
			printf("%d is here:   ", x_count);
			for (int i = 0; i != contours_size; i++)
			{
//				printf("%f\n",contourArea(contours[i]));
				if (contourArea(contours[i])>6000)
				{
					filter[i] = 1;
					y_count++;
				}
				else
				{
					filter[i] = 0;
				}
			}

			if (y_count > 0)
			{
				printf("%d is big enough   ", y_count);
			}
			else
			{
				printf("no one is big enough\n");
//					continue;

			}
		}
		else
		{
			printf("no one!\n");
//				continue;
		}
//check 5 times if there is no what we need ,send no
		if(x_count==0 || y_count==0)
		{
			no_count++;
			if(no_count>=5)
			{
				no_count=0;
				send_char("no\r\n\r");
			}
		}
		else
		{
			no_count=0;
		}

//calculate every distance and find the clostset one
		vector<double> length(contours_size);

		closest_id = 0;

			for (int i = 0; i != contours_size; i++)
			{
				if (filter[i] == 1)
				{

/*

					length[i] = (center_x - (short)(mc[i].x))*(center_x - (short)(mc[i].x)) + (center_y - (short)(mc[i].y))*(center_y - (short)(mc[i].y));
*/
				length_tmp=(center_x - (short)(mc[i].x))*(center_x - (short)(mc[i].x)) + (center_y - (short)(mc[i].y))*(center_y - (short)(mc[i].y));
				length.push_back(length_tmp);
//if find the closest one 				
					if (length[i] <= closest_length)
					{
						closest_length = length[i];
						closest_id = i;
					}
						printf("the closest is at %d\n", closest_id);
						closest_x=(int)mc[closest_id].x;
						closest_y=(int)mc[closest_id].y;
						printf("the closest is at %d,%d\n", closest_x, closest_y);
//circle or s
						area_unkown = contourArea(contours[closest_id]);
						area_rect = minAreaRect(contours[closest_id]).size.area();
						angle = minAreaRect(contours[closest_id]).angle;
					
						minEnclosingCircle(contours[closest_id], center, radius);
						area_circle = CV_PI * radius*radius;
						printf("area is %f,rect is %f,circle is %f   ", area_unkown, area_rect, area_circle);
						cmp_rect = area_unkown / area_rect;
						cmp_circle = area_unkown / area_circle;


						if (cmp_rect > cmp_circle)
						{
							printf("rect!\n");

						}
						else
						{
							printf("circle!\n");
						}
						printf("      %f\n",angle);
						if(angle<=0){angle = -angle;}
//what color and set another ROI
						roi_up=closest_x- r;
						roi_left=closest_y- r;
						if(roi_up<0) roi_up=0;if(roi_up>190) roi_up=190;
						if(roi_left<0) roi_left=0;if(roi_left>320) roi_left=320;
						if(!src.empty())
						src(Rect(roi_up, roi_left, 2 * r, 2 * r)).copyTo(src_roi);
						
#ifdef debug
			cv::namedWindow("src_roi");
			cv::imshow("src_roi", src_roi);
#endif
/*
						if(!src_roi.empty())						
						cvtColor(src_roi, hsv_roi, COLOR_BGR2HSV); 
						split(hsv_roi, hsv_split_roi);
						equalizeHist(hsv_split_roi[2], hsv_split_roi[2]);
						merge(hsv_split_roi, hsv_roi);	
						h=0;s=0; v=0;
						r_count = 0, g_count = 0,b_count=0; 					
						for( int i = 0; i != src_roi.rows; i++)
						{
							for (int j = 0; j != src_roi.cols; j++)
							{
								h = hsv_roi.at<cv::Vec3b>(i, j)[0];
								s = hsv_roi.at<cv::Vec3b>(i, j)[1];
								v = hsv_roi.at<cv::Vec3b>(i, j)[2];

								if (h >= 0 && h <= 15 && s >= 5 && s <= 255) r_count++;
								if (h >= 66 && h <= 123 && s >= 0 && s <= 117) g_count++;
								if (h >= 96 && h <= 179) b_count++;

							}
						}
*/
						int b_num=0,g_num=0, r_num=0;
						r_count = 0, g_count = 0,b_count=0; 					
						for( int i = 0; i != src_roi.rows; i++)
						{
							for (int j = 0; j != src_roi.cols; j++)
							{
								b_num = src_roi.at<cv::Vec3b>(i, j)[0];
								g_num = src_roi.at<cv::Vec3b>(i, j)[1];
								r_num = src_roi.at<cv::Vec3b>(i, j)[2];

								if (b_num >= g_num && b_num >= r_num ) b_count++;
								if (g_num >= b_num && g_num >= r_num ) g_count++;
								if (r_num >= b_num && r_num >= g_num ) r_count++;

							}
						}

					delat_x=320-30 - closest_x;delat_y=240-50 - closest_y;
					if(delat_x>=0)
					{
						left_right_flag="l";
//						delat_x=delat_x/2;
					}
					else
					{
						left_right_flag="y";
						delat_x=(-delat_x);
					}
					if(delat_y>=0)
					{
						up_down_flag="u";
//						delat_x=delat_x/2;
					}
					else
					{
						up_down_flag="d";
						delat_y=(-delat_y);
					}
					printf("delat is %s %d %s %d\n", left_right_flag,delat_x,up_down_flag,delat_y);
					int j;
					j = sprintf(write_buffer,"%s",left_right_flag);
					j += sprintf(write_buffer+j,"%d",delat_x);
					j += sprintf(write_buffer+j,"%s",up_down_flag);
					j += sprintf(write_buffer+j,"%d",delat_y);
										
//send message
					if(no_count==0)
					{
						if (r_count >= g_count)
						{
							if (r_count >= b_count)
							{
								printf("red is %d\n", r_count);

								if (cmp_rect > cmp_circle)
								{
									j += sprintf(write_buffer+j,"%s","rs");

									//send_char("rs\r\n\r");
								}
								else
								{
									j += sprintf(write_buffer+j,"%s","rc");

									//send_char("rc\r\n\r");
								}
						
							}

							else
							{
								printf("blue is %d\n", b_count);

								if (cmp_rect > cmp_circle)
								{
									j += sprintf(write_buffer+j,"%s","bs");
									//send_char("bs\r\n\r");
								}
								else
								{
									j += sprintf(write_buffer+j,"%s","bc");
									//send_char("bc\r\n\r");
								}
								
							}
						}
						else
						{
							if (g_count >= b_count)
							{
								printf("green is %d\n", g_count);

								if (cmp_rect > cmp_circle)
								{
									j += sprintf(write_buffer+j,"%s","gs");
//									send_char("gs\r\n\r");
								}
								else
								{
									j += sprintf(write_buffer+j,"%s","gc");
//									send_char("gc\r\n\r");
								}

							}
							else
							{
								printf("blue is %d\n", b_count);

								if (cmp_rect > cmp_circle)
								{
									j += sprintf(write_buffer+j,"%s","bs");
									
								}
								else
								{
									j += sprintf(write_buffer+j,"%s","bc");
								}

							}

						}
						
					}
//start send message

                    j += sprintf(write_buffer+j,"%d",(int)(angle + 0.5));
					j += sprintf(write_buffer+j,"%s","e");
					send_char(write_buffer);
					printf("    what we will send is %s and the number is %d\n",write_buffer,j);

	

				}
			}
// can not visit mc[closest_id].x mc[closest_id].y do not know why , put upside ok ,why
//		closest_x=(int)mc[closest_id].x;
//		closest_y=(int)mc[closest_id].y;
//		printf("the closest is at %d,%d\n", closest_x, closest_y);

       		char c=waitKey(10);
       		if( c == 27 ) { close(fd); break; }
         }
	
	return 0;
}
