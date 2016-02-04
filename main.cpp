#include <iostream>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>
#include <opencv2/core/core.hpp>
#include <opencv2/cudev/common.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include "time.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>

using namespace std;

#define M_PI           3.14159265358979323846  /* pi */

enum RefImage {LeftRefImage, RightRefImage};

struct CostVolumeParams {

    uint8_t min_disp;
    uint8_t max_disp;
    uint8_t num_disp_layers;
    uint8_t method; // 0 for AD, 1 for ZNCC
    uint8_t win_r;
    RefImage ref_img;

};

struct PrimalDualParams {

    uint32_t num_itr;

    float alpha;
    float beta;
    float epsilon;
    float lambda;
    float aux_theta;
    float aux_theta_gamma;

    /* With preconditoining, we don't need these. */
    float sigma;
    float tau;
    float theta;

};

extern "C"
void runCudaPart(cv::cuda::PtrStepSz<float> data, cv::cuda::PtrStepSz<float> result,int rows, int cols);
void runCudaSet(float* data, float* result, int rows, int cols);

cv::Mat stereoCalcu(int _m, int _n, float* _left_img, float* _right_img, CostVolumeParams _cv_params, PrimalDualParams _pd_params);

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (30.0);

//    Eigen::Matrix3f m;
//    m << 1, 2, 3,
//         4, 5, 6,
//         7, 8, 9;
 
// Eigen::Affine3f transform;
    // Eigen::Vector3f t(0.0 , 0.0, 2.0);
    // transform.translation() = t;

    // Eigen::Matrix3f r;
    // r = Eigen::AngleAxisf((90.0/180.0)*M_PI, Eigen::Vector3f::UnitX())*Eigen::AngleAxisf((90.0/180.0)*M_PI, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf((90.0/180.0)*M_PI, Eigen::Vector3f::UnitZ());
    // transform.linear() = r;

    // cout << r << endl;

    // viewer->addCoordinateSystem (1.5, transform);
 
    viewer->initCameraParameters ();
    return (viewer);
}

int main()
{
    cv::Mat Img_rgb_left;
    cv::Mat Img_rgb_right;
    cv::Mat Img_rgb_left1;
    cv::Mat Img_rgb_right1;

    cv::VideoCapture VCLeft;
    VCLeft.open("/home/roy/Data/vivo2/left.avi");
    // VCLeft.open("/home/roy/Data/vivo3/left.avi");
    // VCLeft.open("/home/roy/Data/vivo4/left.avi");
    VCLeft.read(Img_rgb_left);
    for(int i = 0; i < 20; i++)
        VCLeft.read(Img_rgb_left);
    VCLeft.release();

    cv::VideoCapture VCRight;
    VCRight.open("/home/roy/Data/vivo2/right.avi");
    // VCRight.open("/home/roy/Data/vivo3/right.avi");
    //VCRight.open("/home/roy/Data/vivo4/right.avi");
    VCRight.read(Img_rgb_right);
    for(int i = 0; i < 20; i++)
        VCRight.read(Img_rgb_right);
    VCRight.release();

    //    cv::Mat Img_rgb_left = cv::imread("/home/roy/Data/vivo2/unrec_left.jpg");
    //    cv::Mat Img_rgb_right = cv::imread("/home/roy/Data/vivo2/unrec_right.jpg");
    cv::Mat Img_gray_left;
    cv::Mat Img_gray_right;
    cvtColor(Img_rgb_left, Img_gray_left, CV_RGB2GRAY);
    cvtColor(Img_rgb_right, Img_gray_right, CV_RGB2GRAY);

    int rows = Img_rgb_left.rows;
    int cols = Img_rgb_left.cols;

    cv::Size Img_size(cols, rows);

    // vivo2 true!!!
    cv::Mat K_right = (cv::Mat_<float>(3,3) << 751.706129, 0.000000, 338.665467,
                   0.000000, 766.315580, 257.986032,
                   0.000000, 0.000000, 1.000000);
    cv::Mat Distortion_right = (cv::Mat_<float>(1,4) << -0.328424, 0.856059, 0.003430, 0.000248);

    cv::Mat K_left = (cv::Mat_<float>(3,3) << 752.737796, 0.000000, 263.657845,
                  0.000000, 765.331797, 245.883296,
                  0.000000, 0.000000, 1.000000);
    cv::Mat Distortion_left = (cv::Mat_<float>(1,4) << -0.332222, 0.708196, 0.003904, 0.008043);

    cv::Mat R_StereoCamera = (cv::Mat_<double>(3,3) << 0.9999, 0.0069, -0.0121,
                          -0.0068, 1.0000, 0.0055,
                          0.0121, -0.0054, 0.9999);
    R_StereoCamera = R_StereoCamera.inv();

    cv::Mat T_StereoCamera = (cv::Mat_<double>(3,1) << -5.3754, -0.0716, -0.1877);

    // vivo3
//    cv::Mat K_left = (cv::Mat_<float>(3,3) << 391.656525, 0.000000, 165.964371,
//                      0.000000, 426.835144, 154.498138,
//                      0.000000, 0.000000, 1.000000);
//    cv::Mat Distortion_left = (cv::Mat_<float>(1,4) << -0.196312, 0.129540, 0.004356, 0.006236);

//    cv::Mat K_right = (cv::Mat_<float>(3,3) << 390.376862, 0.000000, 190.896454,
//                       0.000000, 426.228882, 145.071411,
//                       0.000000, 0.000000, 1.000000);
//    cv::Mat Distortion_right = (cv::Mat_<float>(1,4) << -0.205824, 0.186125, 0.015374, 0.003660);

//    cv::Mat R_StereoCamera = (cv::Mat_<double>(3,3) << 0.999999, -0.001045, -0.000000,
//                              0.001045, 0.999999, -0.000000,
//                              0.000000, 0.000000, 1.000000);
//    cv::Mat T_StereoCamera = (cv::Mat_<double>(3,1) << -5.520739, -0.031516, -0.051285);

    //cv::Mat Img_rgb_left_undistor;
    //cv::Mat K_left_undistor;
    //undistort(Img_rgb_left, Img_rgb_left_undistor, K_left, Distortion_left, K_left_undistor);

    cv::Mat R1, R2, P1, P2, Q;
    cv::Rect roi1, roi2;

    cv::stereoRectify(K_left, Distortion_left, K_right, Distortion_right,  Img_size, R_StereoCamera, T_StereoCamera, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, Img_size, &roi1, &roi2);
    //cout << Q;
    //cout << P1 << endl;
    //cout << P2 << endl;

    cv::Mat map11, map12, map21, map22;
    cv::initUndistortRectifyMap(K_left, Distortion_left, R1, P1, Img_size, CV_16SC2, map11, map12);
    cv::initUndistortRectifyMap(K_right, Distortion_right, R2, P2, Img_size, CV_16SC2, map21, map22);

    cv::Mat img_rec_left, img_rec_right;
    cv::remap(Img_gray_left, img_rec_left, map11, map12, cv::INTER_LINEAR);
    cv::remap(Img_gray_right, img_rec_right, map21, map22, cv::INTER_LINEAR);
    img_rec_left.convertTo(img_rec_left, CV_32F);
    img_rec_right.convertTo(img_rec_right, CV_32F);

    //cv::resize(img_left_gray, img_left_gray, Size(img_left_gray.cols/2, img_left_gray.rows/2));
    //cv::resize(img_right_gray, img_right_gray, Size(img_right_gray.cols/2, img_right_gray.rows/2));

    CostVolumeParams cv_params;
    cv_params.min_disp = 0;
    cv_params.max_disp = 64;
    cv_params.method = 1;
    cv_params.win_r = 11;
    cv_params.ref_img = LeftRefImage;

    PrimalDualParams pd_params;
    pd_params.num_itr = 150; // 500
    pd_params.alpha = 0.1; // 10.0 0.01
    pd_params.beta = 1.0; // 1.0
    pd_params.epsilon = 0.1; // 0.1
    pd_params.lambda = 1e-2; // 1e-3
    pd_params.aux_theta = 10; // 10
    pd_params.aux_theta_gamma = 1e-6; // 1e-6


    cv::Mat result = stereoCalcu(img_rec_left.rows, img_rec_left.cols, (float*)img_rec_left.data, (float*)img_rec_right.data, cv_params, pd_params);
    // convert for [0,1] to [min_d, max_d]
    result.convertTo(result, CV_32F, cv_params.max_disp);

    //----------- triangulate
    cv::Mat image3D;
    cv::reprojectImageTo3D(result, image3D, Q);
    cout << "K:" << Q;

    // save the depth image
    cv::Mat XYZcor[3];
    cv::split(image3D, XYZcor);

    cv::Mat Zcor(image3D.rows, image3D.cols, CV_16U);

    for(int x = 0; x < image3D.cols; x++)
    {
        for(int y = 0; y < image3D.rows; y++)
        {
            if(cvIsInf(XYZcor[2].at<float>(y,x)))
                Zcor.at<ushort>(y,x) = 0;
            else
            {
                float temp = 50.0*XYZcor[2].at<float>(y,x);
                Zcor.at<ushort>(y,x) = (ushort)(temp);
            }
        }
    }

    cout << Zcor.row(100);
    cv::imwrite("./depth.png", Zcor);

    //cout << image3D.cols << " " << image3D.rows << " " << image3D.channels() << endl ;
    //cout << image3D.at<cv::Vec3f>(1,1);

//    cv::FileStorage fs("/home/roy/3Dpoints.yml", cv::FileStorage::WRITE);
//    fs << "image3D" << image3D;
//    fs.release();

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------

    //---------rectify the rgb imag
    cv::remap(Img_rgb_left, Img_rgb_left, map11, map12, cv::INTER_LINEAR);
    cv::imwrite("./left.png", Img_rgb_left);

    // XYZRGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int x = 0; x < image3D.cols; x++)
    {
        for(int y = 0; y < image3D.rows; y++)
        {
            cv::Vec3f point3D = image3D.at<cv::Vec3f>(y,x);
            cv::Vec3b pointRGB = Img_rgb_left.at<cv::Vec3b>(y,x);
            pcl::PointXYZRGB basic_point;
            basic_point.x = point3D.val[0];
            basic_point.y = point3D.val[1];
            basic_point.z = point3D.val[2];
            basic_point.b = pointRGB.val[0];
            basic_point.g = pointRGB.val[1];
            basic_point.r = pointRGB.val[2];
            if(cvIsInf(point3D.val[0]) || cvIsInf(point3D.val[1]) || cvIsInf(point3D.val[2]))
                ;//
            else
            {
 //               cout << point3D.val[0] << " " << point3D.val[1] << " " << point3D.val[2] << endl ;
                point_cloud_ptr->points.push_back(basic_point);

            }
        }
    }
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);

    //--------------------
    // -----Main loop viewing-----
    //--------------------
    bool changed = true; 
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (500);
        
//        if(changed)
//        {
//            viewer->removeCoordinateSystem();

//            Eigen::Affine3f transform;
//            Eigen::Vector3f t(0.0 , 0.0, 2.0);
//            transform.translation() = t;

//            Eigen::Matrix3f r;
//            r = Eigen::AngleAxisf((90.0/180.0)*M_PI, Eigen::Vector3f::UnitX())*Eigen::AngleAxisf((90.0/180.0)*M_PI, Eigen::Vector3f::UnitY())*Eigen::AngleAxisf((90.0/180.0)*M_PI, Eigen::Vector3f::UnitZ());
//            //    r = Eigen::AngleAxisf(M_PI*(90.0/360.0), Eigen::Vector3f::UnitZ());
//            transform.linear() = r;

//            viewer->addCoordinateSystem (1.5, transform);
//            changed = !changed;

//        }
//        else
//        {

//            viewer->removeCoordinateSystem();
//            viewer->addCoordinateSystem (1.0);
//            changed = !changed;
//        }

        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    //  cout << result.rows << " " << result.cols << endl;
    //  cout << result.row(100);
    //  result = result.reshape(0,1);
    //  float minVal, maxVal;
    //  cv::min(result, minVal);
    //  cv::max(result, maxVal);
    //  result.reshape(0, img_right_gray.rows);
    //  result.convertTo(result, CV_8U, 1/(maxVal-minVal), -minVal/(maxVal-minVal));

    cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);
    result.convertTo(result, CV_8U);

    //cv::imshow("left", img_rec_left);
    cv::imshow("disparity", result);
    cv::waitKey(0);

    return 0;

}
