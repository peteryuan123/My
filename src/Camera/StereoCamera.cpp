
#include "Camera/StereoCamera.h"
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>


StereoCamera::StereoCamera(const cv::FileNode &sensorNode):
CameraBase(sensorNode), m_K(Eigen::Matrix3d::Zero()), m_invK(Eigen::Matrix3d::Zero())
{
    cv::FileNode data;
    std::string distortion_type;
    std::vector<double> temp;


    data = sensorNode["size"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_size(dataItr - data.begin()) = *dataItr;
    }

    data = sensorNode["distortion_type"];
    distortion_type = data.string();
    if (data.string() == "equidistant")
        m_parameter.resize(16); // (fx, fy, cx, cy, k1, k2, k3, k4)*2
    else if (data.string() == "pinhole")
        m_parameter.resize(18); // (fx, fy, cx, cy, k1 k2, p1, p2, k3)*2
    else
        throw std::invalid_argument("not support type:" + data.string());

    //TODO: check invalid input

    // Intrinsic
    // Left camera
    int counter = 0;
    data = sensorNode["focal_length_l"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    data = sensorNode["principal_point_l"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    data = sensorNode["distortion_l"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    // Left camera
    data = sensorNode["focal_length_r"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    data = sensorNode["principal_point_r"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    data = sensorNode["distortion_r"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        m_parameter[counter] = *dataItr;
        counter++;
    }
    // Intrinsic

    // Extrinsic between two camera
    data = sensorNode["Qrl"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        temp.emplace_back(*dataItr);
    }
    Eigen::Quaterniond Qrl(temp.data());
    temp.clear();
    data = sensorNode["trl"];
    for (cv::FileNodeIterator dataItr = data.begin(); dataItr != data.end(); dataItr++){
        temp.emplace_back(*dataItr);
    }
    m_trl = Eigen::Vector3d(temp.data());


//
//    Eigen::Matrix3d R_rl_ = (Qsb_r*Qsb_l.conjugate()).toRotationMatrix();
//    t_rl_ = Qsb_r*Qsb_l.conjugate() * -tsb_l + tsb_r;
//    std::cout << "R_rl_:" << R_rl_ << std::endl;
//    std::cout << "t_rl_:" << t_rl_.transpose() << std::endl;

    if (distortion_type == "equidistant")
    {
        cv::Size cvSize(m_size[0], m_size[1]);
        cv::Mat1f Kl = (cv::Mat1f(3, 3) << m_parameter[0], 0, m_parameter[2],
                                           0, m_parameter[1], m_parameter[3],
                                           0, 0, 1);
        cv::Mat1f Dl = (cv::Mat1f(1, 4) << m_parameter[4], m_parameter[5], m_parameter[6], m_parameter[7]);
        cv::Mat1f Kr = (cv::Mat1f(3, 3) << m_parameter[8], 0, m_parameter[10],
                                            0, m_parameter[9], m_parameter[11],
                                            0, 0, 1);
        cv::Mat1f Dr = (cv::Mat1f(1, 4) << m_parameter[12], m_parameter[13], m_parameter[14], m_parameter[15]);
        cv::Mat cv_Rrl, cv_trl;
        cv::eigen2cv(Qrl.toRotationMatrix(), cv_Rrl);
        cv::eigen2cv(m_trl, cv_trl);

        cv::Mat Rab_l, Rab_r, P1, P2, Q; // rectification transformation, before rectification to after
        cv::fisheye::stereoRectify(Kl, Dl, Kr, Dr, cvSize, cv_Rrl, cv_trl, Rab_l, Rab_r, P1, P2, Q, cv::CALIB_ZERO_DISPARITY);
        cv::fisheye::initUndistortRectifyMap(Kl, Dl, Rab_l, P1, cvSize, CV_32F, m_M1l, m_M2l);
        cv::fisheye::initUndistortRectifyMap(Kr, Dr, Rab_r, P2, cvSize, CV_32F, m_M1r, m_M2r);

        cv::cv2eigen(Q, m_Q);
        Eigen::Matrix<double, 3, 4> Pr;
        cv::cv2eigen(P2, Pr);
        m_K = Pr.block<3, 3>(0, 0);
        m_invK = m_K.inverse();
        m_stereo_shift = Pr(0, 3);

        // the rectification fix transformation, old fisheye coordinate to new pinhole coordinate,
        // after rectification there is a fix rotation applied to both camera to let the two rectified plane be coplanar
        Eigen::Matrix3d Rab_l_eigen;
        cv::cv2eigen(Rab_l, Rab_l_eigen);
        m_Qsb = Eigen::Quaterniond(Rab_l_eigen) * m_Qsb;
    }
    else
        throw std::invalid_argument("not support type:" + data.string());


    std::cout << "************Stereo Camera**********\n";
    std::cout << "size_:" << m_size.transpose()<< std::endl;
    std::cout << "left original parameter (fx, fy, cx, cy, distortion):" << m_parameter.segment(0, 8).transpose()<< std::endl;
    std::cout << "right original parameter (fx, fy, cx, cy, distortion):" << m_parameter.segment(8, 8).transpose()<< std::endl;
    std::cout << "Undistort_K:" << m_K << std::endl;
    std::cout << "stereo shift:" << m_stereo_shift << std::endl;
    std::cout << "Qsb:" << m_Qsb.coeffs().transpose() << std::endl;
    std::cout << "tsb:" << m_tsb.transpose() << std::endl;
    std::cout << "trl:" << m_trl.transpose() << std::endl;

}


void StereoCamera::undistortRectify(const cv::Mat &src_l, const cv::Mat &src_r, cv::Mat &dest_l, cv::Mat &dest_r)
{
    cv::remap(src_l, dest_l, m_M1l, m_M2l, cv::INTER_AREA);
    cv::remap(src_r, dest_r, m_M1r, m_M2r, cv::INTER_AREA);
}


