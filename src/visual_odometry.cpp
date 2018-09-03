#include "visual_odometry.h"


const int kMinNumFeature = 300;


Visual_Odometry::Visual_Odometry(PinholeCamera* cam):cam_(cam)
{

this->focal_ = this->cam_->fx();
this->pp_  = cv::Point2d(this->cam_->cx(), this->cam_->cy());
this->frame_stage_ = STAGE_FIRST_FRAME;

}

Visual_Odometry::~Visual_Odometry(){}



void Visual_Odometry::addImage(const cv::Mat& img, int frame_id)
{

	//对添加的图像进行判断
	if (img.empty() || img.type() != CV_8UC1 || img.cols != this->cam_->width() || img.rows != this->cam_->height())
		throw std::runtime_error("Frame: provided image has not the same size as the camera model or image is not grayscale");

    // 添加新帧
    this->new_frame_ = img;
    bool result = true;
    if ( this->frame_stage_ == STAGE_DEFAULT_FRAME )
        result = processFrame( frame_id );
    else if ( this->frame_stage_ == STAGE_FIRST_FRAME)
        result = processFirstFrame();
    else 
        result = processSecondFrame();

    std::cout<<this->px_ref_.size()<<std::endl;

    // 处理结束之后将当前帧设为上一处理帧
    this->last_frame_ = this->new_frame_;

}

/*
protected
*/
bool Visual_Odometry::processFirstFrame()
{
    // 对当前帧进行特征检测
    featureDetection(this->new_frame_, this->px_ref_);
    // 修改状态，表明第一帧已经处理完成
    this->frame_stage_ = STAGE_SECOND_FRAME;
    return true;
}

bool Visual_Odometry::processSecondFrame()
{
    //通过光流跟踪确定第二帧中的相关特征
    featureTracking(this->last_frame_, this->new_frame_, this->px_ref_, this->px_cur_, this->disparities_);
    // 计算初始位置
    cv::Mat R,t, E, mask;
    E = cv::findEssentialMat(this->px_cur_, this->px_ref_, this->focal_, this->pp_, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, this->px_cur_, this->px_ref_, R, t, this->focal_, this->pp_, mask);
	this->cur_R_ = R.clone();
	this->cur_t_ = t.clone();
    this->frame_stage_ = STAGE_DEFAULT_FRAME;
    this->px_ref_ = this->px_cur_;
    return true;
}

bool Visual_Odometry::processFrame(int frame_id)
{
	double scale = 1.00;//初始尺度为1
    //通过光流跟踪确定第二帧中的相关特征
	featureTracking(this->last_frame_, this->new_frame_, this->px_ref_, this->px_cur_, this->disparities_); 
    cv::Mat R,t, E, mask;
    E = cv::findEssentialMat(this->px_cur_, this->px_ref_, this->focal_, this->pp_, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, this->px_cur_, this->px_ref_, R, t, this->focal_, this->pp_, mask);

	this->cur_R_ = R.clone();
	this->cur_t_ = t.clone();

	// 如果跟踪特征点数小于给定阈值，进行重新特征检测
   
	if (this->px_ref_.size() < kMinNumFeature)
	{
		featureDetection(this->new_frame_, this->px_ref_);
		featureTracking(this->last_frame_, this->new_frame_, this->px_ref_, this->px_cur_, this->disparities_);
	}
	this->px_ref_ = this->px_cur_;
	return true;
}


void Visual_Odometry::featureDetection(cv::Mat image, std::vector<cv::Point2f> &px_vec)
{
    std::vector<cv::KeyPoint> key_points;
    int fast_threshold = 20;
    bool non_max_suppression = true;
    cv::FAST(image, key_points, fast_threshold, non_max_suppression);
    cv::KeyPoint::convert(key_points, px_vec);
}

void Visual_Odometry::featureTracking(cv::Mat image_ref, cv::Mat image_cur,
		std::vector<cv::Point2f>& px_ref, std::vector<cv::Point2f>& px_cur, std::vector<double>& disparities)
{

    std::vector<unsigned char> status;
    std::vector<float> error; 
    cv::calcOpticalFlowPyrLK( image_ref, image_cur, px_ref, px_cur, status, error );

    // 把跟丢的点删掉
    std::vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
	std::vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
	disparities.clear(); disparities.reserve(px_cur.size());

    for (int i = 0; px_ref_it != px_ref.end(); ++i)
	{
		if (!status[i])
		{
			px_ref_it = px_ref.erase(px_ref_it);    //myvector.erase (myvector.begin()+5);    if want to erase 6th
			px_cur_it = px_cur.erase(px_cur_it);
			continue;
		}
        disparities.push_back(norm(cv::Point2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y)));
		++px_ref_it;
		++px_cur_it;
    }
}


