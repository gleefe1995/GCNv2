#include <iostream>
// #include "open3d/Open3D.h"
#include "string"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace pc2d{

void read_objectpoint(std::vector<cv::Point3f> &object_points,std::string &filename){
    // std::vector<pcl::PointXYZ> pt_cloud;
    

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::io::loadPLYFile<pcl::PointXYZ> (filename, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud);    

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PLYReader Reader;
        // Reader.read(filename, *cloud);
        std::cout<<cloud->points.size()<<std::endl;
        // open3d::geometry::PointCloud test_ply;
        // auto test_ply = std::make_shared<open3d::geometry::PointCloud>();
        // open3d::io::ReadPointCloud(filename, *test_ply);

        

    // open3d::visualization::DrawGeometries({test_ply}, "window");
        // pt_cloud = cloud->points;

    
    // std::cout<<pt_cloud.size()<<std::endl;
    std::vector<cv::Point3f> object_points_;
    // object_points_.reserve(cloud->points.size());
    pcl::PointXYZ point;
    for (int i=0;i<cloud->points.size();i++){
        point = cloud->points[i];
        cv::Point3f point_3f = cv::Point3f(float(point.x),float(point.y),float(point.z));
        object_points_.push_back(point_3f);
    }

    object_points = object_points_;
    
}


// void filter(cv::Mat img, cv::Mat& dst, cv::Mat mask)		// ȸ�� ���� �Լ�
// {
// 	dst = cv::Mat(img.size(), CV_32F, cv::Scalar(0));			// ȸ�� ��� ���� ���
// 	cv::Point h_m = mask.size() / 2;							// ����ũ �߽� ��ǥ	

// 	for (int i = h_m.y; i < img.rows - h_m.y; i++) {		// �Է� ��� �ݺ� ��ȸ
// 		for (int j = h_m.x; j < img.cols - h_m.x; j++)
// 		{
// 			float sum = 0;
// 			for (int u = 0; u < mask.rows; u++) {	// ����ũ ���� ��ȸ
// 				for (int v = 0; v < mask.cols; v++)
// 				{
// 					int y = i + u - h_m.y;
// 					int x = j + v - h_m.x;
// 					sum += mask.at<float>(u, v) * img.at<uchar>(y, x);	// ȸ�� ����
// 				}
// 			}
// 			dst.at<float>(i, j) = sum;				// ȸ�� ������ ���ȭ�� ����
// 		}
// 	}
// }











cv::Mat depth_image(std::vector<cv::Point3f> &object_points,cv::Mat &Tcw, cv::Mat &K){
    // std::cout<<"Hello open3D"<<std::endl;
    
    // std::string filename = "/home/gleefe/D3Feat.pytorch/test_ply.ply";

    // auto test_ply = std::make_shared<open3d::geometry::PointCloud>();
    
    // // std::cout<<test_ply->points_.size()<<std::endl;
    // // Eigen::Vector3d pt_cloud;
    // std::vector<Eigen::Vector3d> pt_cloud;
    
    
    // // open3d::geometry::PointCloud test_ply;
    
    // open3d::io::ReadPointCloud(filename, *test_ply);
    // pt_cloud = test_ply->points_;
    // std::cout<<pt_cloud.size()<<std::endl;
    // open3d::visualization::DrawGeometries({test_ply}, "window");

    cv::Mat rot_mat = cv::Mat::eye(3, 3, CV_64FC1);

    // std::cout<<Tcw.size()<<std::endl;

    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            rot_mat.at<double>(i,j) = double(Tcw.at<float>(i,j));
        }
    }

    cv::Mat rvec;
    cv::Rodrigues(rot_mat,rvec);
    cv::Mat tvec = cv::Mat::zeros(3,1,CV_64FC1);
    for (int i=0;i<3;i++){
        tvec.at<double>(i,0) = double(Tcw.at<float>(i,3));
    }
    
    // Eigen::Matrix3d rvec; 
    // rvec<<1,0,0,
    //       0,1,0,
    //       0,0,1;
    // Eigen::Matrix<float,3,1> tvec;
    // tvec<<0,0,0;

    // double fx= 535.4;
    // double fy= 539.2;
    // double cx= 320.1;
    // double cy= 247.6;

    // cv::Mat K = (cv::Mat_<float>(3,3)<< fx, 0, cx,
    //                                     0, fy, cy,
    //                                     0,  0,   1);

    // Eigen::Matrix3d K;
    // K<< fx,0,cx,
    //     0,fy,cy,
    //     0,0,1;

    // std::vector<cv::Point3f> object_points;
    // for (int i=0;i<pt_cloud.size();i++){
    //     cv::Point3f point = cv::Point3f(float(pt_cloud[i][0]),float(pt_cloud[i][1]),float(pt_cloud[i][2]));
    //     object_points.push_back(point);
    // }

    std::vector<cv::Point2f> depth_map;
    cv::projectPoints(object_points,rvec,tvec,K,cv::noArray(),depth_map);

    // std::cout<<depth_map.size()<<std::endl;
    
    cv::Mat image(480,640,CV_8UC3); 
    image = cv::Mat::zeros(480,640,CV_8UC3);
    
    cv::Mat depth_value_mat = cv::Mat::zeros(480,640,CV_64FC1);

    cv::cvtColor(image,image,cv::COLOR_BGR2HSV);

    cv::Mat R_solve_inv;
    cv::Mat t_solve_f;

    R_solve_inv = rot_mat.t();
    t_solve_f = -R_solve_inv*tvec;

    for (int i=0;i<depth_map.size();i++){
        double dist;
        double x_diff = double(object_points[i].x) - t_solve_f.at<double>(0);
        double y_diff = double(object_points[i].y) - t_solve_f.at<double>(1);
        double z_diff = double(object_points[i].z) - t_solve_f.at<double>(2);
        dist = sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);

        cv::Point2i xy_pt = cv::Point2i(int(depth_map[i].x),int(depth_map[i].y));
        
        if ( (xy_pt.x<640)&&(xy_pt.y<480)&&(xy_pt.x>0)&&(xy_pt.y>0) ){

        if( (dist<depth_value_mat.at<double>(xy_pt.y,xy_pt.x)) || (depth_value_mat.at<double>(xy_pt.y,xy_pt.x)==0)){
            // std::cout<<xy_pt.x<<" "<<xy_pt.y<<std::endl;
            depth_value_mat.at<double>(xy_pt.y,xy_pt.x) = dist;

            // if (dist<4){
            //     int hsv_color = int(( dist *30)); // 120 / min_dist
            //     // std::cout<<i<<std::endl;
            //     cv::circle(image,xy_pt,1,cv::Scalar(hsv_color,255,255),1);
            // }

        }
        }

        
        
    }

    // cv::medianBlur(depth_value_mat,depth_value_mat,25);
    
    for (int i=0;i<depth_value_mat.rows;i++){
        for (int j=0;j<depth_value_mat.cols;j++){
            if ((depth_value_mat.at<double>(i,j)<4)&&(depth_value_mat.at<double>(i,j)!=0)){
                int hsv_color = int( (depth_value_mat.at<double>(i,j)*30));
                cv::circle(image,cv::Point2i(j,i),1,cv::Scalar(hsv_color,255,255),1);
            }
        }
    }
    // cv::resize(image,image,cv::Size(320,240),CV_INTER_CUBIC);
    cv::medianBlur(image,image,25);
    cv::cvtColor(image,image,cv::COLOR_HSV2BGR);
    
    
	


    
    // cv::GaussianBlur(image, image, cv::Size(7, 7), 0);
    // cv::imshow("vis", image);
    // cv::waitKey();

    return image;
}
    

}