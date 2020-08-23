#ifndef __REALSENSE__
#define __REALSENSE__

//#include "example.hpp"
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include "imgui.h"
#include "imgui_impl_glfw.h"


class RealSense
{
private:
	// RealSense
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile;
	rs2::frameset aligned_frameset;

	// Color Buffer *NEW*
	rs2::frame color_frame;
	cv::Mat color_mat;
	uint32_t color_width = 848;
	uint32_t color_height = 480;
	uint32_t color_fps = 30;

	// Depth Buffer
	rs2::frame depth_frame;
	cv::Mat depth_mat;
	
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;

	uint32_t depth_width = 848;
	uint32_t depth_height = 480;
	uint32_t depth_fps = 30;

public:
	// Constructor
	RealSense();

	// Destructor
	~RealSense();

	// Processing
	void run();


private:
	// Initialize
	void initialize();

	// Initialize Sensor
	inline void initializeSensor();

	// Enable Advanced Mode
	inline void enableAdvancedMode();

	// Finalize
	void finalize();

	// Disable Advanced Mode
	inline void disableAdvancedMode();

	// Update Data
	void update();

	// Update Frame
	inline void updateFrame();

	void updateColor();

	// Update Depth
	inline void updateDepth();

	// Draw Data
	void draw();

	void drawColor();

	// Draw Depth
	inline void drawDepth();

	// Show Data
	void show();

	void showColor();

	// Show Depth
	inline void showDepth();

	// Draw Circle
	void MyFilledCircle(cv::Mat img, cv::Point center);
};

#endif // __REALSENSE__