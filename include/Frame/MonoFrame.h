//
// Created by mpl on 22-7-20.
//

#ifndef MYSLAM_MONOFRAME_H
#define MYSLAM_MONOFRAME_H

#include <opencv2/opencv.hpp>
#include "Camera/PinholeCamera.h"
#include "Frame/FrameBase.h"

class MonoFrame: public FrameBase
{
public:
    MonoFrame(const cv::Mat& image, PinholeCamera::Ptr pinholeCamera, double timestamp);

    inline cv::Mat& Image()
    {
        return m_image;
    }

    inline cv::Mat& imageRectified()
    {
        return m_imageRectified;
    }

    void showImage();

    void showImageRectified();


protected:
    cv::Mat m_image, m_imageRectified;
    PinholeCamera::Ptr m_pinholeCamera;
};



#endif //MYSLAM_MONOFRAME_H
