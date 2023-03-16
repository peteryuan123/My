//
// Created by mpl on 22-7-5.
//

#include "Camera/StereoCamera.h"
#include "Frame/StereoFrame.h"
#include "Frame/MonoFrame.h"
#include <experimental/filesystem>
using namespace std::experimental;

void loadImageNames(std::string img_dir, std::vector<std::string>& img_names, std::vector<double>& time_stamps)
{

    // read all the img names and sort to ensure time sequencing
    filesystem::path img_path(img_dir);
    if (filesystem::is_directory(img_path))
    {
        for (const auto &p: filesystem::directory_iterator(img_path))
        {
            std::string name = p.path().string();
            int start = name.find_last_of('/');
            int len = name.find_last_of('.') - start - 1;
            double time = std::stod(name.substr(start+1, len)) / 1000000000;
//            double time = std::stod(name.substr(start+1, len)) ;

            time_stamps.emplace_back(time);
            img_names.push_back(p.path().string());
        }
    }
    else
    {
        std::cout << "given image path is not a directory...\n";
        return;
    }

    std::sort(img_names.begin(), img_names.end());
    std::sort(time_stamps.begin(), time_stamps.end());

//    std::cout << img_names[0] << " " << time_stamps[0] << std::endl;
    // read images
    //    for (const std::string& img_name: img_names)
    //    {
    //        cv::Mat cur_img = cv::imread(img_name, cv::IMREAD_GRAYSCALE);
    //        imgs.push_back(cur_img);
    //    }

}

int main(int argc, char ** argv)
{
    StereoCamera::Ptr stereo0;
    PinholeCamera::Ptr pinhole0;
    PinholeCamera::Ptr pinhole1;

    std::vector<std::string> pinhole0_image_names;
    std::vector<std::string> pinhole1_image_names;
    std::vector<double> pinhole0_image_stamps;
    std::vector<double> pinhole1_image_stamps;

    std::vector<std::string> stereo0_image_names_l;
    std::vector<std::string> stereo0_image_names_r;
    std::vector<double> stereo0_image_stamps_l;
    std::vector<double> stereo0_image_stamps_r;

    cv::FileStorage fsSettings("/home/mpl/My/config/test_camera.yaml", cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << std::string(argv[1]) << std::endl;
        exit(-1);
    }
    cv::FileNode data;
    cv::FileNode camerasNode = fsSettings["cameras"];
    for (cv::FileNodeIterator cameraNodeItr = camerasNode.begin();
         cameraNodeItr != camerasNode.end();
         ++cameraNodeItr)
    {
        const auto &cameraNode = *cameraNodeItr;
        std::cout << "****************************Camera****************************" << std::endl;\

        // read intrinsic
        std::string model = cameraNode["model"];
        std::cout << "model: " << model << std::endl;
        if (model == "StereoCamera")
        {
            stereo0 = std::make_shared<StereoCamera>(cameraNode);
            loadImageNames(cameraNode["path_l"], stereo0_image_names_l, stereo0_image_stamps_l);
            loadImageNames(cameraNode["path_r"], stereo0_image_names_r, stereo0_image_stamps_r);
        }

        if (model == "PinholeCamera")
        {
            if (cameraNode.name() == "c0")
            {
                pinhole0 = std::make_shared<PinholeCamera>(cameraNode);
                loadImageNames(cameraNode["path"], pinhole0_image_names, pinhole0_image_stamps);
            }

            if (cameraNode.name() == "c1")
            {
                pinhole1 = std::make_shared<PinholeCamera>(cameraNode);
                loadImageNames(cameraNode["path"], pinhole1_image_names, pinhole1_image_stamps);
            }

        }

    }
    std::cout << "---------------------Camera read end--------------------\n";

// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
    // ****************PreUndistort test****************
    // project triangulated point to other camera
//    cv::Mat img_stereo_l = cv::imread(stereo0_image_names_l[0], cv::IMREAD_GRAYSCALE);
//    cv::Mat img_stereo_r = cv::imread(stereo0_image_names_r[0], cv::IMREAD_GRAYSCALE);
//    cv::Mat img_mono_0 = cv::imread(pinhole0_image_names[0], cv::IMREAD_GRAYSCALE);
//    cv::Mat img_mono_1 = cv::imread(pinhole1_image_names[0], cv::IMREAD_GRAYSCALE);
//
//    StereoFrame stereo_frame_test(img_stereo_l, img_stereo_r, stereo0, 0);
//    MonoFrame mono_frame_test0(img_mono_0, pinhole0, 0);
//    MonoFrame mono_frame_test1(img_mono_1, pinhole1, 0);
//
//    cv::imwrite("stereo_l.png", stereo_frame_test.leftImageRectified());
//    cv::imwrite("stereo_r.png", stereo_frame_test.rightImageRectified());
//    cv::imwrite("mono0.png", mono_frame_test0.imageRectified());
//    cv::imwrite("mono1.png", mono_frame_test1.imageRectified());
//
//    Eigen::Vector3d Xc1 = stereo0->triangulate(Eigen::Vector2d(583, 268), Eigen::Vector2d(568, 268));
//    Eigen::Vector3d Xc2 = stereo0->triangulate(Eigen::Vector2d(92, 419), Eigen::Vector2d(80, 419));
//
//    Eigen::Vector3d Xw1 = stereo0->Qsb().conjugate() * (Xc1 - stereo0->tsb());
//    Eigen::Vector3d Xw2 = stereo0->Qsb().conjugate() * (Xc2 - stereo0->tsb());
//
//    Eigen::Vector3d Xc1_mono0 = pinhole0->Qsb() * Xw1 + pinhole0->tsb();
//    Eigen::Vector3d Xc2_mono1 = pinhole1->Qsb() * Xw2 + pinhole1->tsb();
//
//    std::cout << pinhole0->project(Xc1_mono0).transpose() << std::endl; // 100, 241, TODO::error too large
//    std::cout << pinhole1->project(Xc2_mono1).transpose() << std::endl; // 507, 381
    // ****************PreUndistort test****************
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
    //    frame_test.showImage();
//    frame_test.showImageRectified();
//
//    Eigen::Vector2d test_triangulation_uv_l(392, 353);
//    Eigen::Vector2d test_triangulation_uv_r(387, 353);
//    Eigen::Vector3d test_triangulation_Xc = stereo_test->triangulate(test_triangulation_uv_l, test_triangulation_uv_r);
//    std::cout << test_triangulation_Xc.transpose() << std::endl;
//    std::cout << stereo_test->project(test_triangulation_Xc).transpose() << std::endl;
//    std::cout << stereo_test->project_right(test_triangulation_Xc).transpose() << std::endl;
//
//    std::cout << "--------------\n" << std::endl;
//    std::cout << test_triangulation_Xc.normalized().transpose() << std::endl;
// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------
    // ****************Mono Distortion test (set the preundistort to 0)*************
    cv::Mat distorted = cv::Mat(1, 1, CV_32FC2);
    distorted.at<cv::Vec2f>(0, 0) = cv::Vec2f(360, 230);
    cv::Mat undistorted;
    pinhole0->undistortPoints(distorted, undistorted);
    std::cout << distorted << std::endl;
    std::cout << undistorted << std::endl;


}