//
// Created by mpl on 22-7-14.
//
#include "Frame/StereoFrame.h"
#include <opencv2/core/eigen.hpp>


StereoFrame::StereoFrame(const cv::Mat& leftImage, const cv::Mat& rightImage,
            StereoCamera::Ptr stereoCamera, double timestamp):
        m_leftImage(leftImage), m_rightImage(rightImage), m_stereoCamera(std::move(stereoCamera)), FrameBase(timestamp)
{
    if (leftImage.empty() || rightImage.empty())
        std::cout << "image empty, left empty:" + std::to_string(leftImage.empty()) +
                     ",right empty:" + std::to_string(rightImage.empty()) << std::endl;
    m_stereoCamera->undistortRectify(m_leftImage, m_rightImage, m_leftImageRectified, m_rightImageRectified);
}

void StereoFrame::showImage()
{
    cv::Mat dest;
    cv::hconcat(m_leftImage, m_rightImage, dest);
    cv::imshow("StereoImageOriginal", dest);
    cv::waitKey(0);
}

void StereoFrame::showImageRectified()
{
    cv::Mat dest;
    cv::hconcat(m_leftImageRectified, m_rightImageRectified, dest);
    cv::line(dest, cv::Point(0, 270), cv::Point(1439, 270), cv::Scalar(255, 0, 0), 1);
    cv::imshow("StereoImageRectified", dest);
    cv::waitKey(0);
//    cv::imwrite("/home/mpl/left.png", m_leftImageRectified);
//    cv::imwrite("/home/mpl/right.png", m_rightImageRectified);
}