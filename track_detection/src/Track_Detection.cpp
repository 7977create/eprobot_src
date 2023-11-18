#include "Track_Detection.h"

// height: 120
// width: 160

Track_Detection::Track_Detection()
{
    ros::NodeHandle nh_private("~");
    nh_private.param<int>("GyraThreshold", this->GyraThreshold, 132); 
    nh_private.param<bool>("StartMove", this->StartMove, false); 
    nh_private.param<double>("MixKP", this->MixKP, 0.025f);
    nh_private.param<double>("MaxKP", this->MaxKP, 0.04f);
    nh_private.param<double>("HiBotSpeed", this->HiBotSpeed, 0.2);

    image_transport::ImageTransport it(n);
	this->astra_Image_sub = it.subscribe("/camera/rgb/image_raw", 1, &Track_Detection::imageCallback,this);
    this->Image_pub = it.advertise("camera/image", 1);
    this->RgbImage_pub = it.advertise("camera/rgb", 1);
    this->cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);

    memset(MiddleArray, 0, sizeof(MiddleArray));

    // capture.open(0);
    // if(!capture.isOpened())
    // {
        // ROS_INFO("Open video0 is error!");
        // ros::shutdown();
    // }

    // capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    // capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    // capture.set(CV_CAP_PROP_FPS, 30.0);
    // if (!capture.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G')))
    // {
        // ROS_INFO("set format failed \n");
    // }

    // frame = boost::make_shared< cv_bridge::CvImage >();
    // frame->encoding = sensor_msgs::image_encodings::BGR8;
}


void Track_Detection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	
	// Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
	  
	cv_ptr->header.stamp = ros::Time::now();
    this->RgbImage_pub.publish(cv_ptr->toImageMsg());

    this->SrcImageRead = cv_ptr->image;
}


Track_Detection::~Track_Detection()
{
    capture.release();
}

void Track_Detection::Open_Imread()
{
    capture >> frame->image; //流的转换   

    if(frame->image.empty())
    {
        ROS_INFO( "Failed to capture frame!" );
        ros::shutdown();
    }

    frame->header.stamp = ros::Time::now();
    this->RgbImage_pub.publish(frame->toImageMsg());

    this->SrcImageRead = frame->image;
}

void Track_Detection::Image_Filter_Process()
{
    Mat GaussianImage, autoImage;
	//Rect rect(40,180,280,60);
	Rect rect(40,180,280,60);
	if(this->SrcImageRead.empty())
	{
		ROS_INFO( "no image!" );
		return;
	}
    resize(this->SrcImageRead, this->SrcImageRead, Size(), 0.5, 0.5);
	//ROS_INFO("%d, %d,",this->SrcImageRead.rows, this->SrcImageRead.cols);
	if(this->SrcImageRead.rows>=240 && this->SrcImageRead.cols>=320)		
		this->SrcImageRead = this->SrcImageRead(rect);
	
	// resize(this->SrcImageRead, this->SrcImageRead, Size(this->SrcImageRead.cols/2,this->SrcImageRead.rows/2), 0, 0);
    cvtColor(this->SrcImageRead, this->GyraImage, COLOR_RGB2GRAY);

    ros::param::get("/Track_Detection_node/GyraThreshold", GyraThreshold);
  
    //adaptiveThreshold(GyraImage, DestImageOut, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY_INV, 45, 3);  
    threshold(this->GyraImage, this->DestImageOut, 0, 255, THRESH_OTSU | THRESH_BINARY);
}

void Track_Detection::Show_Image(String WindowsName)
{
    imshow(WindowsName, DestImageOut);
}

void Track_Detection::ImageMiddleDetection()
{
    unsigned char *PtrRows = NULL;
	
	if(this->SrcImageRead.empty())
	{
		ROS_INFO( "no image!" );
		return;
	}
	

    memset(MiddleArray, 0, sizeof(MiddleArray));
    //ROS_INFO("%d, %d,",DestImageOut.rows, DestImageOut.cols);

    for (int i = DestImageOut.rows-1; i > 0; i--)
    {
        PtrRows = DestImageOut.ptr<uchar>(i);       //获取每行首地址

        for (int j = MIDDLE_VALUE; j > 0; j--)       // left line found
        { 
            if((PtrRows[j] - PtrRows[j-1]) != 0)    
            {
                this->MiddleArray[i][0] = j;
                break;
            }
            this->MiddleArray[i][0] = 0;
        }

        for (int j = MIDDLE_VALUE; j < DestImageOut.cols; j++)       // right line found
        {
            if((PtrRows[j] - PtrRows[j-1]) != 0)
            {
                this->MiddleArray[i][1] = j;
                break;
            }
            this->MiddleArray[i][1] = DestImageOut.cols;
        }
        this->MiddleArray[i][2] = (this->MiddleArray[i][0] + this->MiddleArray[i][1]) / 2;  //middle value
        DestImageOut.at<uchar>(i, this->MiddleArray[i][2]) = 0;
     }

    this->HiBotMove_Control();
	 
	// cv::line(this->DestImageOut, Point(0,180), Point(320,180), CV_RGB(0,0,0),1);
	// cv::line(this->DestImageOut, Point(40,0), Point(40,240), CV_RGB(0,0,0),1);
	// cv::line(this->DestImageOut, Point(170,0), Point(170,240), CV_RGB(0,0,0),1);
	cv::line(this->DestImageOut, Point(140,0), Point(140,60), CV_RGB(0,0,0),1);

    // sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", DestImageOut).toImageMsg(); //mono8
	sensor_msgs::ImagePtr imageMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", this->DestImageOut).toImageMsg(); //mono8
	this->Image_pub.publish(imageMsg);
    // this->Image_pub.publish(imageMsg);
}

void Track_Detection::HiBotMove_Control()       // pid
{
    unsigned char EffectiveValue = 0;
    float MiddleAverageVal = 0;
    float OutPutVth = 0;

    for (int i = 1; i < PROSPECT_VALUE; i++)
    {
        if(fabs(MiddleArray[DestImageOut.rows-i][2] - MiddleArray[DestImageOut.rows-i-1][2]) > 10)  //寻找有效的前瞻值
        {
            EffectiveValue = i;
            break;
        }
        if(i == PROSPECT_VALUE-1)
        {
            EffectiveValue = PROSPECT_VALUE-1;
        }
    }

    float SlopeValue = (this->MiddleArray[DestImageOut.rows-1][2] - this->MiddleArray[DestImageOut.rows - EffectiveValue][2]) / ((EffectiveValue) *1.0f);

    for (int i = 0; i < EffectiveValue; i++)        //中值高斯滤波，去除不可信的中值
    {
        MiddleAverageVal += this->MiddleArray[DestImageOut.rows-i-1][2];
    }
    MiddleAverageVal = MiddleAverageVal/EffectiveValue;

    float Error = MIDDLE_VALUE - MiddleAverageVal;      //P

    ros::param::get("/Track_Detection_node/MixKP", this->MixKP);
    ros::param::get("/Track_Detection_node/MaxKP", this->MaxKP);

    if((EffectiveValue < 20) && (fabs(SlopeValue) > SLOPE_VALUE))   //斜率大于定值或者前瞻小于定值时，使用较大的PID参数
    {
         OutPutVth = Error * this->MaxKP;
        ROS_INFO("Max = %d  %d  %f  %d  %f",    this->MiddleArray[DestImageOut.rows-1][2],\
                                            this->MiddleArray[DestImageOut.rows - EffectiveValue][2]\
                                            , SlopeValue, EffectiveValue, OutPutVth);
    }
    else
    {
        OutPutVth = Error * this->MixKP;
        ROS_INFO("Mix = %d  %d  %f  %d  %f",    this->MiddleArray[DestImageOut.rows-1][2],\
                                            this->MiddleArray[DestImageOut.rows - EffectiveValue][2]\
                                            , SlopeValue, EffectiveValue, OutPutVth);
    }

    ros::param::get("/Track_Detection_node/StartMove", this->StartMove);
    ros::param::get("/Track_Detection_node/HiBotSpeed", this->HiBotSpeed);
    (this->StartMove)?(this->HiBotCmd_Vel(this->HiBotSpeed, OutPutVth)):(this->HiBotCmd_Vel(0.0, 0.0));
    
}

void Track_Detection::HiBotCmd_Vel(float vx, float vz)
{
    geometry_msgs::Twist  twistMsg;

    twistMsg.linear.x = vx;
    twistMsg.angular.z = vz;

    this->cmd_pub.publish(twistMsg);
}


int main(int argc, char** argv)
{
 
    ros::init(argc, argv, "EPRoBot_track_detection");
    ROS_INFO("[ZHAIWEIFENG] Track_Detection_node start!");

    Track_Detection HiBotTrackDetection;

    ros::Rate loop_rate(20);
	ros::Duration(1).sleep();
    while(ros::ok())
    {	
		// HiBotTrackDetection.Open_Imread();
        HiBotTrackDetection.Image_Filter_Process();
        HiBotTrackDetection.ImageMiddleDetection();
		
		ros::spinOnce();
        loop_rate.sleep();
        
    }

}