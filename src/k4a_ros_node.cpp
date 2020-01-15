//Kinect 4 Azure
#include <k4a/k4a.h>//C API
#include <k4a/k4a.hpp>//C++ API, not used here

//C++
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//Customed
#include "Pixel.h"
#include "DepthPixelColorizer.h"
#include "StaticImageProperties.h"
#include "turbojpeg.h"

using namespace sen;
using namespace std;

std::string pkg_loc = ros::package::getPath("k4a-ros");
static float ZoomFacter = 1.0f;// 1.0 to be millimeters

struct bgraPixel
{
	uint8_t Blue;
	uint8_t Green;
	uint8_t Red;
	uint8_t Alpha;
};

struct bgrPixel
{
	uint8_t Blue;
	uint8_t Green;
	uint8_t Red;
};

/**
 * \get OpenCV CV16UC1 Mat data from k4a_image_t
 *
 * \remarks
 *  Input should be: K4A_IMAGE_FORMAT_DEPTH16 or K4A_IMAGE_FORMAT_IR16 format
 */
static bool Get_CV_Mat_CV16UC1(const k4a_image_t src, cv::Mat &dst)
{
	if( (k4a_image_get_format(src)!=K4A_IMAGE_FORMAT_DEPTH16) && ((k4a_image_get_format(src)!=K4A_IMAGE_FORMAT_IR16)))
	{
		printf("source image is not K4A_IMAGE_FORMAT_DEPTH16 or K4A_IMAGE_FORMAT_IR16(CV_16UC1) format!\n");
		return false;
	}
	uint16_t *buffer = reinterpret_cast<uint16_t *>(k4a_image_get_buffer(src));
	cv::Mat RES_CV_MAT;
	RES_CV_MAT = cv::Mat(k4a_image_get_height_pixels(src), k4a_image_get_width_pixels(src), CV_16UC1, buffer);
	RES_CV_MAT.copyTo(dst);
	return true;
}

/**
 * \get OpenCV CV8UC1 Mat data from k4a_image_t
 *
 * \just for Visualization
 *
 * \remarks
 *  Input should be: K4A_IMAGE_FORMAT_DEPTH16 or K4A_IMAGE_FORMAT_IR16 format
 */
static bool Get_CV_Mat_CV8UC1(const k4a_image_t src, cv::Mat &dst)
{
	if( (k4a_image_get_format(src)!=K4A_IMAGE_FORMAT_DEPTH16) && ((k4a_image_get_format(src)!=K4A_IMAGE_FORMAT_IR16)))
	{
		printf("source image is not K4A_IMAGE_FORMAT_DEPTH16 or K4A_IMAGE_FORMAT_IR16(CV_16UC1) format!\n");
		return false;
	}
	uint16_t *buffer = reinterpret_cast<uint16_t *>(k4a_image_get_buffer(src));
	cv::Mat RES_CV_MAT;
	RES_CV_MAT = cv::Mat(k4a_image_get_height_pixels(src), k4a_image_get_width_pixels(src), CV_16UC1, buffer);
	RES_CV_MAT.convertTo(dst,CV_8UC1,1/255.0);
	return true;
}

/**
 * \get OpenCV CV8UC3 Mat data from k4a_image_t
 *
 * \remarks
 *  Input should be: K4A_IMAGE_FORMAT_COLOR_BGRA32 format
 *  Convert to BGR8 in OpenCV
 */
static bool Get_CV_Mat_CV8UC3(const k4a_image_t src, cv::Mat &dst)
{
	if( k4a_image_get_format(src) != K4A_IMAGE_FORMAT_COLOR_BGRA32 )
	{
		printf("source image is not K4A_IMAGE_FORMAT_COLOR_BGRA32(CV8UC4) format!\n");
		return false;
	}
	uint8_t *buffer = k4a_image_get_buffer(src);
	std::vector<bgrPixel> v_bgrpixels;
	bgrPixel bgrpixel;
	for (int i = 0; i < 4*k4a_image_get_height_pixels(src)*k4a_image_get_width_pixels(src); ++i)
	{
		if( (i+1)%4 == 1 )
			bgrpixel.Blue = buffer[i];
		if( (i+1)%4 == 2 )
			bgrpixel.Green = buffer[i];
		if( (i+1)%4 == 3 )
			bgrpixel.Red = buffer[i];
		if( (i+1)%4 == 0 )
			v_bgrpixels.push_back(bgrpixel);
	}
	cv::Mat RES_CV_MAT;
	RES_CV_MAT = cv::Mat(k4a_image_get_height_pixels(src), k4a_image_get_width_pixels(src), CV_8UC3, v_bgrpixels.data());
	RES_CV_MAT.copyTo(dst);
	return true;
}

/**
 * \get OpenCV CV8UC4 Mat data from k4a_image_t
 *
 * \just for Visualization Convenience while using imshow
 *
 * \remarks
 *  Input could be: K4A_IMAGE_FORMAT_COLOR_BGRA32
 *
 *  Convert to BGRA8 in OpenCV
 */
static bool Get_CV_Mat_CV8UC4(const k4a_image_t src, cv::Mat &dst)
{
	if(k4a_image_get_format(src) != K4A_IMAGE_FORMAT_COLOR_BGRA32)
	{
		printf("source image is not K4A_IMAGE_FORMAT_COLOR_BGRA32(CV8UC4) format!\n");
		return false;
	}
	uint8_t *buffer = k4a_image_get_buffer(src);
	cv::Mat RES_CV_MAT;
	RES_CV_MAT = cv::Mat(k4a_image_get_height_pixels(src), k4a_image_get_width_pixels(src), CV_8UC4, buffer);
	RES_CV_MAT.copyTo(dst);
	return true;
}

/**
 * \get xy_table to help transform depth map to PCL pointcloud
 *
 */
static void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
	k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

	int width = calibration->depth_camera_calibration.resolution_width;
	int height = calibration->depth_camera_calibration.resolution_height;

	k4a_float2_t p;
	k4a_float3_t ray;
	int valid;

	for (int y = 0, idx = 0; y < height; y++)
	{
		p.xy.y = (float)y;
		for (int x = 0; x < width; x++, idx++)
		{
			p.xy.x = (float)x;

			k4a_calibration_2d_to_3d(
				calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

			if (valid)
			{
				table_data[idx].xy.x = ray.xyz.x;
				table_data[idx].xy.y = ray.xyz.y;
			}
			else
			{
				table_data[idx].xy.x = nanf("");
				table_data[idx].xy.y = nanf("");
			}
		}
	}
}

/**
 * \get PCL PointCloud of PointXYZ from k4a_image_t by xy_table
 *
 * point cloud get from this function has more poinsts than bool Get_PCL_point_cloud_PointXYZRGB()
 *
 *  Convert to PointCloud<PointXYZ> in PCL
 */
static bool Get_PCL_point_cloud_PointXYZ(
	const k4a_image_t depth_image,
	const k4a_image_t xy_table,
	pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	int height = k4a_image_get_height_pixels(depth_image);
	int width = k4a_image_get_width_pixels(depth_image);

	cloud.height = height;
	cloud.width = width;
	cloud.resize(height*width);

	uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
	k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

	for (int i = 0; i < width * height; i++)
	{
		if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
		{
			cloud.points.at(i).x = ZoomFacter * xy_table_data[i].xy.x * (float)depth_data[i];
			cloud.points.at(i).y = ZoomFacter * xy_table_data[i].xy.y * (float)depth_data[i];
			cloud.points.at(i).z = ZoomFacter * (float)depth_data[i];
		}
		else
		{
			cloud.points.at(i).x = nanf("");
			cloud.points.at(i).y = nanf("");
			cloud.points.at(i).z = nanf("");
		}
	}
	return true;
}

/**
 * \get PCL PointCloud of PointXYZRGB from k4a_image_t
 *
 * point cloud get from this function has less poinsts than bool Get_PCL_point_cloud_PointXYZ()
 * because only matched color and points can be selected to PointXYZRGB
 *
 *  Convert to PointCloud<PointXYZRGB> in PCL
 */
bool Get_PCL_point_cloud_PointXYZRGB(
	const k4a_image_t point_cloud_image,
	const k4a_image_t color_image,
	pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	int pc_height = k4a_image_get_height_pixels(point_cloud_image);
	int pc_width = k4a_image_get_width_pixels(point_cloud_image);
	int color_height = k4a_image_get_height_pixels(color_image);
	int color_width = k4a_image_get_width_pixels(color_image);
	if ((pc_height != color_height) || (pc_width != color_width))
	{
		printf("point_cloud_image has different size to color_image!\n");
		return false;
	}

	cloud.height = pc_height;
	cloud.width = pc_width;
	cloud.resize(pc_height*pc_width);

	int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	for (int i = 0; i < pc_width * pc_height; i++)
	{
		if (point_cloud_image_data[3 * i + 2] != 0
			&& color_image_data[4 * i + 0] != 0
			&& color_image_data[4 * i + 1] != 0
			&& color_image_data[4 * i + 2] != 0
			&& color_image_data[4 * i + 3] != 0)
		{
			cloud.points.at(i).x = ZoomFacter * (float)point_cloud_image_data[3 * i + 0];
			cloud.points.at(i).y = ZoomFacter * (float)point_cloud_image_data[3 * i + 1];
			cloud.points.at(i).z = ZoomFacter * (float)point_cloud_image_data[3 * i + 2];
			// image data is BGR
			cloud.points.at(i).b = (float)color_image_data[4 * i + 0];
			cloud.points.at(i).g = (float)color_image_data[4 * i + 1];
			cloud.points.at(i).r = (float)color_image_data[4 * i + 2];
		}
		else
		{
			cloud.points.at(i).x = nanf("");
			cloud.points.at(i).y = nanf("");
			cloud.points.at(i).z = nanf("");
		}
	}
	return true;
}


int main(int argc, char **argv)
{
	//ROS
	ros::init(argc, argv, "k4a_ros_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Publisher pub_color 			= n.advertise<sensor_msgs::Image>("k4a/color/color", 10);
	ros::Publisher pub_color_to_depth 	= n.advertise<sensor_msgs::Image>("k4a/color/color_to_depth", 10);

	ros::Publisher pub_depth 			= n.advertise<sensor_msgs::Image>("k4a/depth/depth", 10);
	ros::Publisher pub_depth_to_color 	= n.advertise<sensor_msgs::Image>("k4a/depth/depth_to_color", 10);

	ros::Publisher pub_depth_colorized			= n.advertise<sensor_msgs::Image>("k4a/depth/depth_colorized", 10);
	ros::Publisher pub_depth_to_color_colorized = n.advertise<sensor_msgs::Image>("k4a/depth/depth_to_color_colorized", 10);

	ros::Publisher pub_point_cloud_depth_to_color = n.advertise<sensor_msgs::PointCloud2>("k4a/point_cloud/point_cloud_depth_to_color", 10);
	ros::Publisher pub_point_cloud_depth_to_depth = n.advertise<sensor_msgs::PointCloud2>("k4a/point_cloud/point_cloud_depth_to_depth", 10);
	ros::Publisher pub_point_cloud_color_to_depth = n.advertise<sensor_msgs::PointCloud2>("k4a/point_cloud/point_cloud_color_to_depth", 10);

	ros::Publisher pub_ir_image = n.advertise<sensor_msgs::Image>("k4a/ir_image", 10);

	//Kinect 4 Azure related initial parameters
	int returnCode = 1;
	k4a_device_t device = NULL;
	const int32_t TIMEOUT_IN_MS = 1000;
	k4a_transformation_t transformation = NULL;
	k4a_transformation_t transformation_color_downscaled = NULL;
	k4a_capture_t capture = NULL;
	uint32_t device_count = 0;
	k4a_image_t depth_image = NULL;
	k4a_image_t color_image = NULL;
	k4a_image_t ir_image = NULL;
	k4a_image_t xy_table = NULL;
	k4a_image_t color_image_downscaled = NULL;

	int point_count = 0;
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	config.synchronized_images_only = true;
	//Kinect 4 Azure related initial parameters

	// START CONNECT TO "Kinect 4 Azure"
	device_count = k4a_device_get_installed_count();

	if (device_count == 0)
	{
		printf("No K4A devices found\n");
		return 0;
	}

	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
	{
		printf("Failed to open device\n");
		goto Exit;
	}

	k4a_calibration_t calibration;
	if (K4A_RESULT_SUCCEEDED !=	k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
	{
		printf("Failed to get calibration\n");
		goto Exit;
	}

	transformation = k4a_transformation_create(&calibration);

	k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
		calibration.depth_camera_calibration.resolution_width,
		calibration.depth_camera_calibration.resolution_height,
		calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
		&xy_table);
	create_xy_table(&calibration, xy_table);

	if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
	{
		printf("Failed to start cameras\n");
		goto Exit;
	}

	// START RECEIVE DATA AND PROCESS DATA FRAMES
	while(ros::ok())
	{
		// Get a capture
		switch (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS))
		{
			case K4A_WAIT_RESULT_SUCCEEDED:
			break;
			case K4A_WAIT_RESULT_TIMEOUT:
			printf("Timed out waiting for a capture\n");
			goto Exit;
			case K4A_WAIT_RESULT_FAILED:
			printf("Failed to read a capture\n");
			goto Exit;
		}



		// all msgs transformed from color image and depth map should share the same header timestamp,
		// because they are synchronized by the device as well.
		std_msgs::Header header;
		header.stamp = ros::Time::now();



		// Publish 16UC1 encoding depth map under depth sensor frame
		// Get a depth image pointer
		depth_image = k4a_capture_get_depth_image(capture);
		if (depth_image == 0)
		{
			printf("Failed to get depth image from capture\n");
			continue;
		}
		cv::Mat cv_depth_image_CV16UC1;
		if( Get_CV_Mat_CV16UC1(depth_image, cv_depth_image_CV16UC1) )
		{
			imshow("cv_depth_image CV_16UC1", cv_depth_image_CV16UC1);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/depth_sensor";
			msg = cv_bridge::CvImage(header, "16UC1", cv_depth_image_CV16UC1).toImageMsg();
			pub_depth.publish(msg);
		}

		cv::Mat cv_depth_image_CV8UC1;
		if( Get_CV_Mat_CV8UC1(depth_image, cv_depth_image_CV8UC1) )
		{
			imshow("cv_depth_image CV_8UC1", cv_depth_image_CV8UC1);
		}

		// Publish BGRA8 encoding Colorize Depth Image under depth sensor frame
		std::vector<Pixel> ColorizedDepthBuffer;
		if( ColorizeDepthImage(depth_image, DepthPixelColorizer::ColorizeBlueToRed,
			GetDepthModeRange(config.depth_mode), &ColorizedDepthBuffer) )
		{
			cv::Mat cv_ColorizedDepth_CV8UC4;
			cv_ColorizedDepth_CV8UC4 = cv::Mat(k4a_image_get_height_pixels(depth_image),
				k4a_image_get_width_pixels(depth_image), CV_8UC4, ColorizedDepthBuffer.data());
			cv::imshow("cv_ColorizedDepth CV_8UC4", cv_ColorizedDepth_CV8UC4);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/depth_sensor";
			msg = cv_bridge::CvImage(header, "bgra8", cv_ColorizedDepth_CV8UC4).toImageMsg();
			pub_depth_colorized.publish(msg);
		}

		pcl::PointCloud<pcl::PointXYZ> cloud_depth_to_depth;
		if (Get_PCL_point_cloud_PointXYZ(depth_image, xy_table, cloud_depth_to_depth) )
		{
			sensor_msgs::PointCloud2 cloud_msg;
			toROSMsg(cloud_depth_to_depth, cloud_msg);
			cloud_msg.header = header;
			cloud_msg.header.frame_id = "/k4a/depth_sensor";
			pub_point_cloud_depth_to_depth.publish(cloud_msg);
		}



		// Publish BGR8 encoding color image under color sensor frame
		// Get a color image pointer
		color_image = k4a_capture_get_color_image(capture);
		if (color_image == 0)
		{
			printf("Failed to get color image from capture\n");
			continue;
		}
		cv::Mat cv_color_image_CV8UC4;
		if( Get_CV_Mat_CV8UC4(color_image, cv_color_image_CV8UC4) )
			imshow("cv_color_image CV8UC4", cv_color_image_CV8UC4);

		cv::Mat cv_color_image_CV8UC3;
		if( Get_CV_Mat_CV8UC3(color_image, cv_color_image_CV8UC3) )
		{
			imshow("cv_color_image CV8UC3", cv_color_image_CV8UC3);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/color_sensor";
			msg = cv_bridge::CvImage(header, "bgr8", cv_color_image_CV8UC3).toImageMsg();
			pub_color.publish(msg);
		}



		// point cloud and color image under depth sensor frame
		int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
		int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);

		k4a_image_t transformed_color_image = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
			depth_image_width_pixels,
			depth_image_height_pixels,
			depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
			&transformed_color_image))
		{
			printf("Failed to create transformed color image\n");
			return false;
		}

		k4a_image_t point_cloud_image_color_to_depth = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			depth_image_width_pixels,
			depth_image_height_pixels,
			depth_image_width_pixels * 3 * (int)sizeof(int16_t),
			&point_cloud_image_color_to_depth))
		{
			printf("Failed to create point cloud image\n");
			return false;
		}

		if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation,
			depth_image,
			color_image,
			transformed_color_image))
		{
			printf("Failed to compute transformed color image\n");
			return false;
		}

		// Publish BGR8 encoding color image under depth sensor frame
		cv::Mat cv_transformed_color_image_CV8UC3;
		if( Get_CV_Mat_CV8UC3(transformed_color_image, cv_transformed_color_image_CV8UC3) )
		{
			imshow("cv_transformed_color_image CV8UC3", cv_transformed_color_image_CV8UC3);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/depth_sensor";
			msg = cv_bridge::CvImage(header, "bgr8", cv_transformed_color_image_CV8UC3).toImageMsg();
			pub_color_to_depth.publish(msg);
		}

		// Publish Point cloud XYZRGB data, project  color to depth, under depth sensor frame
		if (K4A_RESULT_SUCCEEDED == k4a_transformation_depth_image_to_point_cloud(transformation,
			depth_image,
			K4A_CALIBRATION_TYPE_DEPTH,
			point_cloud_image_color_to_depth))
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud_color_to_depth;
			if (Get_PCL_point_cloud_PointXYZRGB(point_cloud_image_color_to_depth, transformed_color_image, cloud_color_to_depth) )
			{
				sensor_msgs::PointCloud2 cloud_msg;
				toROSMsg(cloud_color_to_depth, cloud_msg);
				cloud_msg.header = header;
				cloud_msg.header.frame_id = "/k4a/depth_sensor";
				pub_point_cloud_color_to_depth.publish(cloud_msg);
			}
		}
		else
		{
			printf("Failed to compute point cloud\n");
		}

		k4a_image_release(transformed_color_image);
		k4a_image_release(point_cloud_image_color_to_depth);



		// point cloud and depth map under color sensor frame
		int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
		int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
		k4a_image_t transformed_depth_image = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * (int)sizeof(uint16_t),
			&transformed_depth_image))
		{
			printf("Failed to create transformed depth image\n");
			return false;
		}

		k4a_image_t point_cloud_image_depth_to_color = NULL;
		if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
			color_image_width_pixels,
			color_image_height_pixels,
			color_image_width_pixels * 3 * (int)sizeof(int16_t),
			&point_cloud_image_depth_to_color))
		{
			printf("Failed to create point cloud image\n");
			return false;
		}

		if (K4A_RESULT_SUCCEEDED !=
			k4a_transformation_depth_image_to_color_camera(transformation, depth_image, transformed_depth_image))
		{
			printf("Failed to compute transformed depth image\n");
			return false;
		}

		// Publish 16UC1 encoding depth map under color sensor frame
		cv::Mat cv_transformed_depth_image_CV16UC1;
		if( Get_CV_Mat_CV16UC1(transformed_depth_image, cv_transformed_depth_image_CV16UC1) )
		{
			imshow("cv_transformed_depth_image CV16UC1", cv_transformed_depth_image_CV16UC1);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/color_sensor";
			msg = cv_bridge::CvImage(header, "16UC1", cv_transformed_depth_image_CV16UC1).toImageMsg();
			pub_depth_to_color.publish(msg);
		}

		// Publish BGRA8 encoding Colorize Depth Image under color sensor frame
		std::vector<Pixel> ColorizedDepthToColorBuffer;
		if( ColorizeDepthImage(transformed_depth_image, DepthPixelColorizer::ColorizeBlueToRed,
			GetDepthModeRange(config.depth_mode), &ColorizedDepthToColorBuffer) )
		{
			cv::Mat cv_ColorizedDepthToColor_CV8UC4;
			cv_ColorizedDepthToColor_CV8UC4 = cv::Mat(k4a_image_get_height_pixels(transformed_depth_image),
				k4a_image_get_width_pixels(transformed_depth_image), CV_8UC4, ColorizedDepthToColorBuffer.data());
			cv::imshow("cv_ColorizedDepthToColor CV_8UC4", cv_ColorizedDepthToColor_CV8UC4);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/color_sensor";
			msg = cv_bridge::CvImage(header, "bgra8", cv_ColorizedDepthToColor_CV8UC4).toImageMsg();
			pub_depth_to_color_colorized.publish(msg);
		}

		// Publish Point cloud under color sensor frame
		if (K4A_RESULT_SUCCEEDED == k4a_transformation_depth_image_to_point_cloud(transformation,
			transformed_depth_image,
			K4A_CALIBRATION_TYPE_COLOR,
			point_cloud_image_depth_to_color))
		{
			pcl::PointCloud<pcl::PointXYZRGB> cloud_depth_to_color;
			if (Get_PCL_point_cloud_PointXYZRGB(point_cloud_image_depth_to_color, color_image, cloud_depth_to_color) )
			{
				sensor_msgs::PointCloud2 cloud_msg;
				toROSMsg(cloud_depth_to_color, cloud_msg);
				cloud_msg.header = header;
				cloud_msg.header.frame_id = "/k4a/color_sensor";
				pub_point_cloud_depth_to_color.publish(cloud_msg);
			}
			else{
				printf("Failed to compute point cloud\n");
				return false;
			}
		}

		k4a_image_release(transformed_depth_image);
		k4a_image_release(point_cloud_image_depth_to_color);



		// Publish 16UC1 encoding ir image under depth sensor frame
		// Get an ir image pointer
		ir_image = k4a_capture_get_ir_image(capture);
		if (ir_image == 0)
		{
			printf("Failed to get ir image from capture\n");
			continue;
		}
		cv::Mat cv_ir_image_CV16UC1;
		if( Get_CV_Mat_CV16UC1(ir_image, cv_ir_image_CV16UC1) )
		{
			imshow("cv_ir_image CV_16UC1", cv_ir_image_CV16UC1);
			sensor_msgs::ImagePtr msg;
			header.frame_id = "/k4a/depth_sensor";
			msg = cv_bridge::CvImage(header, "16UC1", cv_ir_image_CV16UC1).toImageMsg();
			pub_ir_image.publish(msg);
		}

		cv::waitKey(10);
		// ************************************************ Release Resources ************************************************ //
		// release images
		if (depth_image != NULL)
			k4a_image_release(depth_image);
		depth_image = NULL;

		if (color_image != NULL)
			k4a_image_release(color_image);
		color_image = NULL;

		if (ir_image != NULL)
			k4a_image_release(ir_image);
		ir_image = NULL;

		if (color_image_downscaled != NULL)
			k4a_image_release(color_image_downscaled);
		color_image_downscaled = NULL;

		// release capture
		if (capture != NULL)
			k4a_capture_release(capture);
		capture = NULL;

		// ************************************************ END Release Resources ******************************************** //

		loop_rate.sleep();
	}

	if (xy_table != NULL)
		k4a_image_release(xy_table);
	xy_table = NULL;

	if (transformation != NULL)
		k4a_transformation_destroy( transformation);
	transformation = NULL;

	if (transformation_color_downscaled != NULL)
		k4a_transformation_destroy( transformation_color_downscaled);
	transformation_color_downscaled = NULL;

	returnCode = 0;


	Exit:
	printf("\nExit: cleaning up...");
	if (depth_image != NULL)
	{
		k4a_image_release(depth_image);
	}
	if (color_image != NULL)
	{
		k4a_image_release(color_image);
	}
	if (color_image_downscaled != NULL)
	{
		k4a_image_release(color_image_downscaled);
	}
	if (xy_table != NULL)
	{
		k4a_image_release(xy_table);
	}
	if (capture != NULL)
	{
		k4a_capture_release(capture);
	}
	if (transformation != NULL)
	{
		k4a_transformation_destroy(transformation);
	}
	if (transformation_color_downscaled != NULL)
	{
		k4a_transformation_destroy(transformation_color_downscaled);
	}
	if (device != NULL)
	{
		k4a_device_close(device);
	}
	printf("\nExit: all shut down!\n");
	return returnCode;
}
