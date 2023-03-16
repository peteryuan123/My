//
// Created by mpl on 22-7-14.
//

#ifndef MYSLAM_STEREOFRAME_H
#define MYSLAM_STEREOFRAME_H

#include <utility>

#include "Camera/StereoCamera.h"
#include "Frame/FrameBase.h"

class StereoFrame: public FrameBase
{
public:
    StereoFrame(const cv::Mat& leftImage, const cv::Mat& rightImage, StereoCamera::Ptr stereoCamera, double timestamp);

    inline cv::Mat& leftImage()
    {
        return m_leftImage;
    }

    inline cv::Mat& rightImage()
    {
        return m_rightImage;
    }

    inline cv::Mat& leftImageRectified()
    {
        return m_leftImageRectified;
    }

    inline cv::Mat& rightImageRectified()
    {
        return m_rightImageRectified;
    }

    void showImage();

    void showImageRectified();


protected:
    cv::Mat m_leftImage, m_rightImage;
    cv::Mat m_leftImageRectified, m_rightImageRectified;


    StereoCamera::Ptr m_stereoCamera;
};



#endif //MYSLAM_STEREOFRAME_H
