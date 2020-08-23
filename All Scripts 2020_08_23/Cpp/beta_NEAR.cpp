#include <Windows.h>
#include "pch.h"
#include "beta.h"
#include "tracker.h"
#include <cmath>
// Third-party dependencies
//#include "realsense.h"
// []
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include <opencv2/opencv.hpp>


#define w 400
#ifndef PI
const double PI = 3.14159265358979323846;
#endif

// Global output array!
float OUT_ARRAY[16];//[6];

// Realsense and Openpose variables #!
bool lost_tracking = false;
float FINGER_D_VALUE = 0.0f;
float OLD_FINGER_D_VALUE = 0.0f;
float TUM_D_VALUE = 0.0f;
float OLD_TUM_D_VALUE = 0.0f;
//
float MID_D_VALUE = 0.0f; // -new
float OLD_MID_D_VALUE = 0.0f;// -new

// RealSense wrapper
rs2::pipeline *p;
rs2::pipeline_profile *pp(new rs2::pipeline_profile);
rs2::frameset ALIGNED_FRAMESET;

// Color Buffer *NEW*
rs2::frame color_frame;
cv::Mat color_mat;
uint32_t color_width = 848;
uint32_t color_height = 480;
uint32_t color_fps = 60;

// Depth Buffer
rs2::frame depth_frame;
cv::Mat depth_mat;
uint32_t depth_width = 848;
uint32_t depth_height = 480;
uint32_t depth_fps = 60;

// RealSense Config variables!
int MIN_CLAMP_VAL = 50;
int MAX_CLAMP_VAL = 1200;
float COLOR_EXPOSURE_VAL = 156.0f;
float DEPTH_EXPOSURE_VAL = 4100.0f;

// Hand key-point variables!
cv::Point KEY_POINT_LOC_PEK;
cv::Point OLD_KEY_POINT_LOC_PEK;
cv::Point KEY_POINT_LOC_TUM;
cv::Point OLD_KEY_POINT_LOC_TUM;
cv::Point KEY_POINT_LOC_MID;// -new
cv::Point OLD_KEY_POINT_LOC_MID;// -new

Vector3 NEAR_FINGER_LOC;
// DLL variables. #!
int GoTime;
Vector3 posVec;

// Nearest point variables!	
double minVal;
double maxVal;
cv::Point minLoc;
cv::Point maxLoc;
// Nearest point variables!	
double minVal2;
double maxVal2;
cv::Point minLoc2;
cv::Point maxLoc2;

struct float3 {
	float x, y, z;
	float3 operator*(float t)
	{
		return { x * t, y * t, z * t };
	}

	float3 operator-(float t)
	{
		return { x - t, y - t, z - t };
	}

	void operator*=(float t)
	{
		x = x * t;
		y = y * t;
		z = z * t;
	}

	void operator=(float3 other)
	{
		x = other.x;
		y = other.y;
		z = other.z;
	}

	void add(float t1, float t2, float t3)
	{
		x += t1;
		y += t2;
		z += t3;
	}
};

float3 the_theta;

// Tracker Object!
tracker t_obj(color_width, color_height);

float average(float a[], int n)
{	// Function that return average of an array. 
	// Find sum of array element 
	int sum = 0;
	for (int i = 0; i < n; i++)
		sum += a[i];

	return sum / n;
}
//												**
//************************************************





//********* RealSense Converter functions ********
//												**
Vector3 convert_depth_pixel_to_metric_coordinate(float pixel_x, float pixel_y, float depth)
{

	auto const camera_intrinsics = p->get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
	/*
	float metric_x = (pixel_x - camera_intrinsics.ppx) / camera_intrinsics.fx *depth;
	float metric_y = (pixel_y - camera_intrinsics.ppy) / camera_intrinsics.fy *depth;

	Vector3 metric_vec = { metric_x, metric_y, depth };
	return metric_vec;*/

	// % TEST %%%%%%%%%%%
	assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
	assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
	//assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

	float x = (pixel_x - camera_intrinsics.ppx) / camera_intrinsics.fx;
	float y = (pixel_y - camera_intrinsics.ppy) / camera_intrinsics.fy;
	if (camera_intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
	{
		float r2 = x * x + y * y;
		float f = 1 + camera_intrinsics.coeffs[0] * r2 + camera_intrinsics.coeffs[1] * r2*r2 + camera_intrinsics.coeffs[4] * r2*r2*r2;
		float ux = x * f + 2 * camera_intrinsics.coeffs[2] * x*y + camera_intrinsics.coeffs[3] * (r2 + 2 * x*x);
		float uy = y * f + 2 * camera_intrinsics.coeffs[3] * x*y + camera_intrinsics.coeffs[2] * (r2 + 2 * y*y);
		x = ux;
		y = uy;
	}

	Vector3 metric_vec = { depth*x, depth*y, depth };
	return metric_vec;
}
//												**
//************************************************

bool init_RS_OP(rs2::pipeline_profile *pp)
{
	// ----------oO THIS BABY DOES 3 THINGS: Oo------------------------------
	// 1) with openpose api configs and starts the wrappers.
	// 2) with realsense api configs and starts a color and depth stream.
	// 3) with realsense api apply advanced mode -> clamping dist.
	//----------------------------------------------------------------------- obs check returns!


	// REALSENSE STUFF
	//wrap = new rs_wrapper();
	p = new rs2::pipeline();
	rs2::pipeline_profile pipeline_profile;
	pipeline_profile = *pp;

	//################## TURN ON CAMERA STREAMS ##################
	//															##
	cv::setUseOptimized(true);
	// Set Device Config
	rs2::config config;
	config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps); // Depth Stream
	// Start Pipeline
	pipeline_profile = p->start(config);
	// Retrieve Device
	rs2::device device = pipeline_profile.get_device();

	// %%%%%%%%%%%% SET EXPOSER %%%%%%%%%%%%%%%%
	rs2::sensor myDepthSensor = device.first<rs2::depth_sensor>();
	// Set
	try {

		myDepthSensor.set_option(RS2_OPTION_EXPOSURE, DEPTH_EXPOSURE_VAL);
	}
	catch (const rs2::invalid_value_error& e) {

		std::cout << "\nGOOD BYE" << e.what() << std::endl;
	}
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	// Advanced Mode
	if (device.is<rs400::advanced_mode>())
	{
		// Enable Advanced Mode
		rs400::advanced_mode advanced_mode = device.as<rs400::advanced_mode>();
		if (!advanced_mode.is_enabled()) {
			advanced_mode.toggle_advanced_mode(true);
		}

		// e.g. Control Depth Table (Clamp Depth to Specified Range)
		STDepthTableControl depth_table_control = advanced_mode.get_depth_table();
		depth_table_control.depthClampMin = MIN_CLAMP_VAL; // Min Depth 1.0m
		depth_table_control.depthClampMax = MAX_CLAMP_VAL; // Max Depth 2.0m
		depth_table_control.disparityShift = 0;	// Disparity Mode
		advanced_mode.set_depth_table(depth_table_control);
		std::cout << "\nHELLO line 83";

		// e.g. Apply Visual Preset (https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets)

	}
	else
	{
		std::cout << "\nFAIL line 70 IF-sats";
		//Kick back to global pipes
		*pp = pipeline_profile;
		return false;
	}
	//Kick back to global pipes
	*pp = pipeline_profile;
	//															##
	//################## ---------------------- ##################

	return true;
}

int main()
{
	// This will activate the camera streams.
	if (!init_RS_OP(pp))
		return -1;

	return 0;

}

void init()
{	// Does main() work or not? GoTime will know.
	GoTime = (int)main();

}

void newImageUpdate(float* fp)
{
	
	// Update Frame
	rs2::frameset frameset = p->wait_for_frames();
	//*NEW*
	// Retrieve Aligned Frame
	//-rs2::align align(rs2_stream::RS2_STREAM_COLOR);
	//-ALIGNED_FRAMESET = align.process(frameset);
	// Retrieve Color Frame
	//-color_frame = ALIGNED_FRAMESET.get_color_frame();

	// Retrive Frame Information
	//-color_width = color_frame.as<rs2::video_frame>().get_width();
	//-color_height = color_frame.as<rs2::video_frame>().get_height();

	// Retrieve Depth Frame
	//-depth_frame = ALIGNED_FRAMESET.get_depth_frame();
	depth_frame = frameset.get_depth_frame();
	// Retrive Frame Size
	depth_width = depth_frame.as<rs2::video_frame>().get_width();
	depth_height = depth_frame.as<rs2::video_frame>().get_height();

	// LAST THING TO DO TO DEPTH MAT
	// Create cv::Mat form Depth Frame
	depth_mat = cv::Mat(depth_height, depth_width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));
	//-rs2::depth_frame fancy_dframe = ALIGNED_FRAMESET.get_depth_frame();

	// #### NEAREST POINT STUFF ####
	// Turn all zeros to max clamp value.
	depth_mat.setTo(MAX_CLAMP_VAL, depth_mat == 0.0f);


	//std::cout << "\nFinger Tip D-val: [" << FINGER_D_VALUE << "] @: [" << KEY_POINT_LOC.x << ", " << KEY_POINT_LOC.y << "]";
	cv::Mat scale_mat;
	depth_mat.convertTo(scale_mat, CV_8U, -255.0 / MAX_CLAMP_VAL, 255.0); // 0-10000 -> 255(white)-0(black)
	cv::applyColorMap(scale_mat, scale_mat, cv::COLORMAP_RAINBOW);
	// Show the drawn image.
	cv::imshow("Depth", scale_mat);




	//########## LAST PART ########### 
	// Transfer both output vector values to global (float) output array.
	for (int i = 0; i < 3; i++) {
		*fp++ = 0.0f;//Mat_out.at<float>(0, i);
	}
	for (int i = 0; i < 3; i++) {
		*fp++ = 0.0f;//Mat_out2.at<float>(0, i);
	}
	
	for (int i = 0; i < 3; i++) {
		*fp++ = 0.0f;
	}

	*fp++ = 0.0f;
	*fp++ = 0.0f;
	*fp++ = 0.0f;
	*fp++ = 0.0f;
	*fp++ = 0.0f;
	*fp++ = 0.0f;

	*fp++ = 0.0f;





}

void close_all()
{
	// realsense
	p->stop();
	delete p;
	cv::destroyAllWindows();
}

extern "C" {


	// Dummy function to bypass the unity realsense issue.
	int BETA_API dummy_function(int handle)
	{/*

		*/
		return 0;
	}
	// 
	int BETA_API Init(int handle)
	{
		// init() will give GoTime a value 0||-1
		init();

		return GoTime;
	}
	// The so-called "main" function, supposedly called each frame in unity.
	float BETA_API MonoUpdate(int handle)
	{
		return float(GoTime);
	}

	// This one returns the x,y and depth of the detected finger tip.
	float BETA_API *ImageUpdate(int handle)
	{
		newImageUpdate(OUT_ARRAY);
		return OUT_ARRAY;
	}

	void BETA_API Shutdown(int handle)
	{
		close_all();
		std::cout << "\nI close all. Shutdown completed!";
	}
}/*<--- extern "C" */


/// NEAREST POINT BETA !
