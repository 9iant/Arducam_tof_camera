#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ctime>
#include "ArducamTOFCamera.hpp"

#define MAX_DISTANCE 1
using namespace Arducam;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addCoordinateSystem(0.01);
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->initCameraParameters();
    return (viewer);
}

void getPreview(uint8_t *preview_ptr, float *depth_image_ptr, float *amplitude_image_ptr)
{
    auto len = 240 * 180;
    for (int i = 0; i < len; i++)
    {
        uint8_t mask = *(amplitude_image_ptr + i) > 30 ? 254 : 0;
        float depth = ((1 - (*(depth_image_ptr + i) / MAX_DISTANCE)) * 255);
        uint8_t pixel = depth > 255 ? 255 : depth;
        *(preview_ptr + i) = pixel & mask;
    }
}

int main()
{
    ArducamTOFCamera tof;
    ArducamFrameBuffer *frame;
    char buff[60];
    float *depth_ptr;
    float *amplitude_ptr;
    uint8_t *preview_ptr = new uint8_t[43200];
    if (tof.init(Connection::CSI))
    {
        std::cerr << "initialization failed" << std::endl;
        exit(-1);
    }
    if (tof.start())
    {
        std::cerr << "Failed to start camera" << std::endl;
        exit(-1);
    }
    tof.setControl(ControlID::RANGE, MAX_DISTANCE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_ptr->points.push_back(pcl::PointXYZ(10, 10, 4));
    cloud_ptr->width = cloud_ptr->size();
    cloud_ptr->height = 1;
    cloud_ptr->is_dense = true;
    vtkObject::GlobalWarningDisplayOff();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);

    const float fx = 240 / (2 * tan(0.5 * M_PI * 64.3 / 180));
    const float fy = 180 / (2 * tan(0.5 * M_PI * 50.4 / 180));
    for (;;)
    {
        frame = tof.requestFrame(200);

        if (frame != nullptr)
        {
            depth_ptr = (float *)frame->getData(FrameType::DEPTH_FRAME);
            amplitude_ptr = (float *)frame->getData(FrameType::AMPLITUDE_FRAME);
            getPreview(preview_ptr, depth_ptr, amplitude_ptr);
            cv::Mat result_frame(180, 240, CV_8U, preview_ptr);
            cv::Mat amplitude_frame(180, 240, CV_32F, amplitude_ptr);

           
            cv::Mat depth_frame(180, 240, CV_32F, depth_ptr);

            pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            current_cloud->height = 180;
            current_cloud->width = 240;
            current_cloud->points.resize(current_cloud->height * current_cloud->width);

            for (int v = 0; v < depth_frame.rows; ++v)
            {
                for (int u = 0; u < depth_frame.cols; ++u)
                {
                    float z = depth_frame.at<float>(v, u);
                    float x = (u - depth_frame.cols / 2.0) * z / fx;
                    float y = (v - depth_frame.rows / 2.0) * z / fy;
                    current_cloud->points[v * depth_frame.cols + u].x = x;
                    current_cloud->points[v * depth_frame.cols + u].y = y;
                    current_cloud->points[v * depth_frame.cols + u].z = z;
                }
            }

            viewer->updatePointCloud<pcl::PointXYZ>(current_cloud, "sample cloud");

            viewer->spinOnce(1);
            pcl::io::savePCDFileASCII("test_pcd.pcd", *current_cloud);

            cv::imshow("Preview", result_frame);
            cv::imshow("Amplitude", amplitude_frame);
            cv::waitKey(1);

            tof.releaseFrame(frame);
        }
    }

    tof.stop();
    delete[] preview_ptr;

    return 0;
}
