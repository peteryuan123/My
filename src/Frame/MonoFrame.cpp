//
// Created by mpl on 22-7-20.
//

#include "Frame/MonoFrame.h"

MonoFrame::MonoFrame(const cv::Mat &image, PinholeCamera::Ptr pinholeCamera, double timestamp)
:m_image(image), m_pinholeCamera(std::move(pinholeCamera)), FrameBase(timestamp)
{
    if (image.empty())
        std::cout << "image empty" << std::endl;
    if (m_pinholeCamera->isPreUndistort())
        m_pinholeCamera->undistortImage(m_image, m_imageRectified);
}

void  MonoFrame::showImage()
{
    cv::imshow("MonoImageOriginal", m_image);
    cv::waitKey(0);
}

void  MonoFrame::showImageRectified()
{
    cv::imshow("MonoImageRectified", m_imageRectified);
    cv::waitKey(0);
}