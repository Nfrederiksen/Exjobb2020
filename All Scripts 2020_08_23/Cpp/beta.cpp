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


// Command-line user interface
#include <openpose/flags.hpp>
// OpenPose dependencies
#include <openpose/headers.hpp>
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

op::Wrapper opWrapper{ op::ThreadManagerMode::Asynchronous };


// RealSense wrapper
rs2::pipeline *p;
rs2::pipeline_profile *pp(new rs2::pipeline_profile);
rs2::frameset ALIGNED_FRAMESET;

// Color Buffer *NEW*
rs2::frame color_frame;
cv::Mat color_mat;
uint32_t color_width = 848;
uint32_t color_height = 480;
uint32_t color_fps = 30;

// Depth Buffer
rs2::frame depth_frame;
cv::Mat depth_mat;
uint32_t depth_width = 848;
uint32_t depth_height = 480;
uint32_t depth_fps = 30;

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

//********* OPENPOSE functions *******************
//												**
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, tracker t)
{
	try
	{
		// User's displaying/saving/other processing here
			// datum.cvOutputData: rendered frame with pose or heatmaps
			// datum.poseKeypoints: Array<float> with the estimated pose
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			// Display image
			const cv::Mat cvMat = OP_OP2CVCONSTMAT(datumsPtr->at(0)->cvOutputData);
			cv::Point p1 = { t.bbox[0], t.bbox[1] };
			cv::Point p2 = { t.bbox[0] + t.bbox[2], t.bbox[1] + t.bbox[3] };
			cv::rectangle(cvMat, p1, p2, cv::Scalar(0, 255, 0), 1);

			cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - Tutorial C++ API", cvMat);
			//cv::waitKey(0);
		}
		else
			op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

void configureWrapper(op::Wrapper& opWrapper)
{
	try
	{
		// Configuring OpenPose

		// logging_level
		op::checkBool(
			0 <= FLAGS_logging_level && FLAGS_logging_level <= 255, "Wrong logging_level value.",
			__LINE__, __FUNCTION__, __FILE__);
		op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
		op::Profiler::setDefaultX(FLAGS_profile_speed);


		// Applying user defined configuration - GFlags to program variables
		// producerType
		op::ProducerType producerType;
		op::String producerString;
		std::tie(producerType, producerString) = op::flagsToProducer(
			op::String(FLAGS_image_dir), op::String(FLAGS_video), op::String(FLAGS_ip_camera), FLAGS_camera,
			FLAGS_flir_camera, FLAGS_flir_camera_index);
		// cameraSize
		const auto cameraSize = op::flagsToPoint(op::String(FLAGS_camera_resolution), "-1x-1");
		// outputSize
		const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
		// netInputSize
		const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
		// faceNetInputSize
		const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
		// handNetInputSize
		const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
		// poseMode
		const auto poseMode = op::flagsToPoseMode(FLAGS_body);
		// poseModel
		const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
		// JSON saving
		if (!FLAGS_write_keypoint.empty())
			op::opLog(
				"Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json`"
				" instead.", op::Priority::Max);
		// keypointScaleMode
		const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);
		// heatmaps to add
		const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts, FLAGS_heatmaps_add_bkg,
			FLAGS_heatmaps_add_PAFs);
		const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);
		// >1 camera view?
		const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1);
		// Face and hand detectors
		const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
		const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);
		// Enabling Google Logging
		const bool enableGoogleLogging = true;

		// Pose configuration (use WrapperStructPose{} for default and recommended configuration)
		const op::WrapperStructPose wrapperStructPose{
			poseMode, netInputSize, outputSize, keypointScaleMode, FLAGS_num_gpu, FLAGS_num_gpu_start,
			FLAGS_scale_number, (float)FLAGS_scale_gap, op::flagsToRenderMode(FLAGS_render_pose, multipleView),
			poseModel, !FLAGS_disable_blending, (float)FLAGS_alpha_pose, (float)FLAGS_alpha_heatmap,
			FLAGS_part_to_show, op::String(FLAGS_model_folder), heatMapTypes, heatMapScaleMode, FLAGS_part_candidates,
			(float)FLAGS_render_threshold, FLAGS_number_people_max, FLAGS_maximize_positives, FLAGS_fps_max,
			op::String(FLAGS_prototxt_path), op::String(FLAGS_caffemodel_path),
			(float)FLAGS_upsampling_ratio, enableGoogleLogging };
		opWrapper.configure(wrapperStructPose);
		// Face configuration (use op::WrapperStructFace{} to disable it)
		const op::WrapperStructFace wrapperStructFace{
			FLAGS_face, faceDetector, faceNetInputSize,
			op::flagsToRenderMode(FLAGS_face_render, multipleView, FLAGS_render_pose),
			(float)FLAGS_face_alpha_pose, (float)FLAGS_face_alpha_heatmap, (float)FLAGS_face_render_threshold };
		opWrapper.configure(wrapperStructFace);
		// Hand configuration (use op::WrapperStructHand{} to disable it)
		const op::WrapperStructHand wrapperStructHand{
			FLAGS_hand, handDetector, handNetInputSize, FLAGS_hand_scale_number, (float)FLAGS_hand_scale_range,
			op::flagsToRenderMode(FLAGS_hand_render, multipleView, FLAGS_render_pose), (float)FLAGS_hand_alpha_pose,
			(float)FLAGS_hand_alpha_heatmap, (float)FLAGS_hand_render_threshold };
		opWrapper.configure(wrapperStructHand);
		// Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
		const op::WrapperStructExtra wrapperStructExtra{
			FLAGS_3d, FLAGS_3d_min_views, FLAGS_identification, FLAGS_tracking, FLAGS_ik_threads };
		opWrapper.configure(wrapperStructExtra);
		// Output (comment or use default argument to disable any output)
		const op::WrapperStructOutput wrapperStructOutput{
			FLAGS_cli_verbose, op::String(FLAGS_write_keypoint), op::stringToDataFormat(FLAGS_write_keypoint_format),
			op::String(FLAGS_write_json), op::String(FLAGS_write_coco_json), FLAGS_write_coco_json_variants,
			FLAGS_write_coco_json_variant, op::String(FLAGS_write_images), op::String(FLAGS_write_images_format),
			op::String(FLAGS_write_video), FLAGS_write_video_fps, FLAGS_write_video_with_audio,
			op::String(FLAGS_write_heatmaps), op::String(FLAGS_write_heatmaps_format), op::String(FLAGS_write_video_3d),
			op::String(FLAGS_write_video_adam), op::String(FLAGS_write_bvh), op::String(FLAGS_udp_host),
			op::String(FLAGS_udp_port) };
		opWrapper.configure(wrapperStructOutput);
		// No GUI. Equivalent to: opWrapper.configure(op::WrapperStructGui{});
		// Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
		if (FLAGS_disable_multi_thread)
			opWrapper.disableMultiThreading();
	}
	catch (const std::exception& e)
	{
		op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
	}
}

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



	// OPENPOSE STUFF ##### !
	// Requiredflags to enable heatmaps
	FLAGS_body = 0;
	FLAGS_hand = true;
	FLAGS_hand_detector = 2;

	configureWrapper(opWrapper);
	// Starting OpenPose
	op::opLog("Starting thread(s)...", op::Priority::High);
	opWrapper.start();


	//################## TURN ON CAMERA STREAMS ##################
	//															##
	cv::setUseOptimized(true);
	// Set Device Config
	rs2::config config;
	config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps); // *NEW* Color stream
	config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps); // Depth Stream
	// Start Pipeline
	pipeline_profile = p->start(config);
	// Retrieve Device
	rs2::device device = pipeline_profile.get_device();

	// %%%%%%%%%%%% SET EXPOSER %%%%%%%%%%%%%%%%
	rs2::sensor myColorSensor = device.first<rs2::color_sensor>();
	rs2::sensor myDepthSensor = device.first<rs2::depth_sensor>();
	// Set
	try {
		myColorSensor.set_option(RS2_OPTION_EXPOSURE, COLOR_EXPOSURE_VAL);
		//myDepthSensor.set_option(RS2_OPTION_EXPOSURE, DEPTH_EXPOSURE_VAL);
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

		std::ifstream ifs("../VisualPresets/HighResHighDensityPreset.json");
		if (ifs.is_open()) {
			std::ostringstream oss;
			oss << ifs.rdbuf();
			advanced_mode.load_json(oss.str());
		}
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
	float fBool;

	// Update Frame
	rs2::frameset frameset = p->wait_for_frames();
	//*NEW*
	// Retrieve Aligned Frame
	rs2::align align(rs2_stream::RS2_STREAM_COLOR);
	ALIGNED_FRAMESET = align.process(frameset);
	// Retrieve Color Frame
	color_frame = ALIGNED_FRAMESET.get_color_frame();

	// Retrive Frame Information
	color_width = color_frame.as<rs2::video_frame>().get_width();
	color_height = color_frame.as<rs2::video_frame>().get_height();

	// Retrieve Depth Frame
	depth_frame = ALIGNED_FRAMESET.get_depth_frame();

	// Retrive Frame Size
	depth_width = depth_frame.as<rs2::video_frame>().get_width();
	depth_height = depth_frame.as<rs2::video_frame>().get_height();

	// Create cv::Mat form Color Frame
	color_mat = cv::Mat(color_height, color_width, CV_8UC3, const_cast<void*>(color_frame.get_data()));


	// OPENPOSE STUFF
	// Read image and hand rectangle locations
	const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(color_mat);
	const std::vector<std::array<op::Rectangle<float>, 2>> handRectangles
	{
		// Left/Right hands of person 0
		std::array<op::Rectangle<float>, 2>{
			op::Rectangle<float>{0.0f, 0.0f, 0.0f, 0.0f}, // Left hand
			op::Rectangle<float>{(float)t_obj.bbox[0], (float)t_obj.bbox[1], (float)t_obj.bbox[2], (float)t_obj.bbox[3]}  // Right hand

	},
	};
	// Create new datum
	auto datumsPtr = std::make_shared<std::vector<std::shared_ptr<op::Datum>>>();
	datumsPtr->emplace_back();
	auto& datumPtr = datumsPtr->at(0);
	datumPtr = std::make_shared<op::Datum>();
	// Fill datum with image and handRectangles
	datumPtr->cvInputData = imageToProcess;
	datumPtr->handRectangles = handRectangles;

	// Process and display image
	opWrapper.emplaceAndPop(datumsPtr);

	//----|
	if (datumsPtr != nullptr && !datumsPtr->empty())
	{
		//op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
						//op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);

		const auto numberHandParts = datumsPtr->at(0)->handKeypoints[1].getSize(1);

		// Slightly more efficient version
		// ### FIND FINGER LOCATION ###
		const auto baseIndex = datumsPtr->at(0)->handKeypoints[1].getSize(2)*(0 * numberHandParts + 7);
		const auto xR = datumsPtr->at(0)->handKeypoints[1][baseIndex];
		const auto yR = datumsPtr->at(0)->handKeypoints[1][baseIndex + 1];
		KEY_POINT_LOC_PEK = { (int)xR, (int)yR };
		// ### FIND THUMB LOCATION ###
		const auto baseIndex2 = datumsPtr->at(0)->handKeypoints[1].getSize(2)*(0 * numberHandParts + 3);
		const auto xR2 = datumsPtr->at(0)->handKeypoints[1][baseIndex2];
		const auto yR2 = datumsPtr->at(0)->handKeypoints[1][baseIndex2 + 1];
		KEY_POINT_LOC_TUM = { (int)xR2, (int)yR2 };
		// ### FIND FINGER3 LOCATION ### //-new
		const auto baseIndex3 = datumsPtr->at(0)->handKeypoints[1].getSize(2)*(0 * numberHandParts + 2);
		const auto xR3 = datumsPtr->at(0)->handKeypoints[1][baseIndex3];
		const auto yR3 = datumsPtr->at(0)->handKeypoints[1][baseIndex3 + 1];
		KEY_POINT_LOC_MID = { (int)xR3, (int)yR3 };
		// HELLO
		float array_x[21], array_y[21];
		float *px, *py;
		auto total_cs = 0.0f;
		for (int i = 0; i < 21; i++)
		{
			auto bi = datumsPtr->at(0)->handKeypoints[1].getSize(2)*(0 * numberHandParts + i);
			array_x[i] = datumsPtr->at(0)->handKeypoints[1][bi];
			array_y[i] = datumsPtr->at(0)->handKeypoints[1][bi + 1];
			total_cs += datumsPtr->at(0)->handKeypoints[1][bi + 2];
		}

		px = array_x;
		py = array_y;
		// Calculate the MEAN Confidence Score!
		total_cs /= 21;

		// %%%%%%%%%%%%%%%% REINITIALIZE OR NOT? %%%%%%%%%%%%%%%%
		if (total_cs <= 0.08f)
		{
			KEY_POINT_LOC_PEK = OLD_KEY_POINT_LOC_PEK;
			KEY_POINT_LOC_TUM = OLD_KEY_POINT_LOC_TUM;
			KEY_POINT_LOC_MID = OLD_KEY_POINT_LOC_MID;//-new
			lost_tracking = true;
			t_obj.default_bbox_creator();
			fBool = 1.0f;
		}
		else
		{
			fBool = 0.0f;
			// ## Update the old key points!
			OLD_KEY_POINT_LOC_PEK = KEY_POINT_LOC_PEK;
			OLD_KEY_POINT_LOC_TUM = KEY_POINT_LOC_TUM;
			OLD_KEY_POINT_LOC_MID = KEY_POINT_LOC_MID;//-new

			lost_tracking = false;
			float delta_x = *(std::max_element(px + 0, px + 20)) - *(std::min_element(px + 0, px + 20));
			float delta_y = *(std::max_element(py + 0, py + 20)) - *(std::min_element(py + 0, py + 20));
			auto box_length = std::max(delta_x, delta_y);
			box_length = std::max(box_length, 76.0f);
			// ## Double the size of box length (Some marginal for the hand).
			box_length *= 2;

			// ## Find Centroid a.k.a mean x and mean y.
			int center_x = average(array_x, 21);
			int center_y = average(array_y, 21);

			// ## Assign our current_center[2] in Tracker Object -> center_x and center_y
			t_obj.current_center[0] = center_x;
			t_obj.current_center[1] = center_y;

			// ## Now that our Tracker Object has knowlegde of a new "center point" and "crop size" 
			// ## a new bounding box can be defined. Use for example: tr.bbox_creator(crop_size).
			t_obj.bbox_creator(box_length);
		}
		// %%%%%%%%%%%%%%%% -------------------- %%%%%%%%%%%%%%%%
	}
	else
	{
		lost_tracking = true;
		op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);

	}

	// LAST THING TO DO TO DEPTH MAT
	// Create cv::Mat form Depth Frame
	depth_mat = cv::Mat(depth_height, depth_width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));
	rs2::depth_frame fancy_dframe = ALIGNED_FRAMESET.get_depth_frame();

	// #### NEAREST POINT STUFF ####
	// Turn all zeros to max clamp value.
	//depth_mat.setTo(MAX_CLAMP_VAL, depth_mat == 0.0f);

	// Find min depth val. + its position in mat.
	//cv::minMaxLoc(depth_mat, &minVal, &maxVal, &minLoc, &maxLoc);
	//float tipDepthValue = fancy_dframe.get_distance(minLoc.x, minLoc.y);
	//std::cout << "\nFinger Tip D-val: [" << tipDepthValue << "]";
	//std::cout << "min val : " << minVal << " @:" << minLoc << std::endl;
	//std::cout << "max val: " << maxVal << "@:" << maxLoc << std::endl;
	


	//float p0_z = fancy_dframe.get_distance(367, 353);
	float p1_z = fancy_dframe.get_distance(341, 295);
	float p2_z = fancy_dframe.get_distance(606, 359);

	//Vector3 p0 = convert_depth_pixel_to_metric_coordinate(367.0f, 353.0f, p0_z);
	Vector3 p1 = convert_depth_pixel_to_metric_coordinate(341.0f, 295.0f, p1_z);
	Vector3 p2 = convert_depth_pixel_to_metric_coordinate(606.0f, 359.0f, p2_z);

	// OPENPOSE STUFF !
	// Keep last non-zero value incase the d-value suddenly jumps to zero mid-stream.
	// [ INDEX ]
	if (fancy_dframe.get_distance(KEY_POINT_LOC_PEK.x, KEY_POINT_LOC_PEK.y) == 0)
		FINGER_D_VALUE = OLD_FINGER_D_VALUE;
	else
	{
		// Update the old value w/ the current one.
		OLD_FINGER_D_VALUE = FINGER_D_VALUE;
		// Update the current one w/ the new one.
		FINGER_D_VALUE = fancy_dframe.get_distance(KEY_POINT_LOC_PEK.x, KEY_POINT_LOC_PEK.y);
	}
	// Keep last non-zero value incase the d-value suddenly jumps to zero mid-stream.
	// [ THUMB ]
	if (fancy_dframe.get_distance(KEY_POINT_LOC_TUM.x, KEY_POINT_LOC_TUM.y) == 0)
		TUM_D_VALUE = OLD_TUM_D_VALUE;
	else
	{
		// Update the old value w/ the current one.
		OLD_TUM_D_VALUE = TUM_D_VALUE;
		// Update the current one w/ the new one.
		TUM_D_VALUE = fancy_dframe.get_distance(KEY_POINT_LOC_TUM.x, KEY_POINT_LOC_TUM.y);
	}
	// Keep last non-zero value incase the d-value suddenly jumps to zero mid-stream.
	// [ MIDDLE ] //-NEW
	if (fancy_dframe.get_distance(KEY_POINT_LOC_MID.x, KEY_POINT_LOC_MID.y) == 0)
		MID_D_VALUE = OLD_MID_D_VALUE;
	else
	{
		// Update the old value w/ the current one.
		OLD_MID_D_VALUE = MID_D_VALUE;
		// Update the current one w/ the new one.
		MID_D_VALUE = fancy_dframe.get_distance(KEY_POINT_LOC_MID.x, KEY_POINT_LOC_MID.y);
	}


	//std::cout << "\nFinger Tip D-val: [" << FINGER_D_VALUE << "] @: [" << KEY_POINT_LOC.x << ", " << KEY_POINT_LOC.y << "]";
	cv::Mat scale_mat;
	depth_mat.convertTo(scale_mat, CV_8U, -255.0 / MAX_CLAMP_VAL, 255.0); // 0-10000 -> 255(white)-0(black)
	cv::applyColorMap(scale_mat, scale_mat, cv::COLORMAP_RAINBOW);
	// Draw a white circle on top of key point.
	cv::circle(scale_mat, KEY_POINT_LOC_PEK, w / 32, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);
	cv::circle(scale_mat, KEY_POINT_LOC_TUM, w / 32, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);
	cv::circle(scale_mat, KEY_POINT_LOC_MID, w / 40, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_8); //-new
	// Show the drawn image.
	///cv::imshow("Depth", scale_mat);
	/*
	if (datumsPtr != nullptr)
	{
		display(datumsPtr, t_obj);
	}
	else
		op::opLog("Image could not be processed.", op::Priority::High);
	*/
	//Vector3 output = { KEY_POINT_LOC.x, KEY_POINT_LOC.y, FINGER_D_VALUE };
	//Vector3 output = { minLoc.x, minLoc.y, tipDepthValue };
	//Vector3 coord = convert_depth_pixel_to_metric_coordinate(minLoc.x, minLoc.y, tipDepthValue);
	//Vector3 coord = convert_depth_pixel_to_metric_coordinate(848/2, 480/2, tipDepthValue);
	Vector3 coord1 = convert_depth_pixel_to_metric_coordinate(KEY_POINT_LOC_PEK.x, KEY_POINT_LOC_PEK.y, FINGER_D_VALUE);
	Vector3 coord2 = convert_depth_pixel_to_metric_coordinate(KEY_POINT_LOC_TUM.x, KEY_POINT_LOC_TUM.y, TUM_D_VALUE);
	Vector3 coord3 = convert_depth_pixel_to_metric_coordinate(KEY_POINT_LOC_MID.x, KEY_POINT_LOC_MID.y, MID_D_VALUE); //-new
	//########### CHANGING COORDINATE SYSTEM #####################
	/*
		// Identity Matrix
	cv::Mat _M = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f, 0.0f
										,  0.0f, 1.0f, 0.0f, 0.0f
										,  0.0f, 0.0f, 1.0f, 0.0f
										,  0.0f, 0.0f, 0.0f, 1.0f);
	*/
	// [!] Index Finger input vector.
	cv::Mat input = (cv::Mat_<float>(1, 4) << coord1.x, coord1.y, coord1.z, 1.0f);
	// Rotation Matrix
	float rad = 0.785398;// 35 degrees  0.610865
	cv::Mat rot_M = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f, 0.0f
		, 0.0f, (cos(rad)), (sin(rad)), 0.0f
		, 0.0f, (-sin(rad)), (cos(rad)), 0.0f
		, 0.0f, 0.0f, 0.0f, 1.0f);
	// Translation Matrix
	cv::Mat tran_M = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f, -0.005f
		, 0.0f, 1.0f, 0.0f, -0.173f
		, 0.0f, 0.0f, 1.0f, 0.550f
		, 0.0f, 0.0f, 0.0f, 1.0f);
	float sh = 0.5f;
	// Shear Matrix
	cv::Mat shear_M = (cv::Mat_<float>(4, 4) << 1.0f, 0.0f, 0.0f, 0.0f
		, 0.0f, 1.0f, 0.0f, 0.0f
		, 0.0f, sh, 1.0f, 0.0f
		, 0.0f, 0.0f, 0.0f, 1.0f);
	//=============================================================================================
	// Why Shearing? Reason is, that while moving the finger point up and down
	// along the Y-axis IRL would results in a diagonal movement in-game. Diagonal 
	// towards the viewer. It seems to work properly when moving the finger along
	// the X-axis though. Anyway, since the reason for this fault is as of now unknown,
	// a shearing matrix is used to combat this. We shear on the Y-axis towards the Z-axis. ( IS THIS "TANGENTIAL DISTORTION" ?)
	//=============================================================================================
	/*
			// Big transformation Matrix
	cv::Mat _T = (cv::Mat_<float>(4, 4) << 0.7106f, 0.0094f, -0.0431f, -0.0537f
										, 0.0565f, 0.7945f, 0.2910f, -0.3020f
										, 0.0370f, -0.2092f, 0.5640f, -0.5962f
										, 0.0000f, -0.0000f, 0.0000f, 1.0000f);

	*/


	// HEMMA
	cv::Mat _T = (cv::Mat_<float>(4, 4) << -0.7120f, 0.0296f, 0.0538f, -0.0248f
		, -0.0279f, -0.6965f, -0.3156f, 0.1891f
		, 0.0281f, -0.2262f, 0.4967f, -0.2756f
		, 0.0000f, 0.0000f, 0.0000f, 1.0000f);




	// Exjobbsrummet 1 SCREEN # Lite fel mätningar på x-led.
	cv::Mat _Txs = (cv::Mat_<float>(4, 4) << -1.0729f, 0.0060f, -0.0030f, 0.0858f
		, 0.0199f, -0.7085f, -0.7729f, 0.8758f
		, -0.0067f, -0.8293f, 0.7601f, -0.5466f
		, 0.0f, 0.0f, 0.0f, 1.0f);
	// Exjobbsrummet 1 SCREEN # Den vi körde med mest!
	cv::Mat _Txs2 = (cv::Mat_<float>(4, 4) << -0.9938f, 0.0056f, -0.0028f, 0.0794f
		, 0.0199f, -0.7085f, -0.7729f, 0.8758f
		, -0.0063f, -0.7682f, 0.7041f, -0.5463f
		, 0.0f, 0.0f, 0.0f, 1.0f);

	// Exjobbsrummet 2 Absolute
	cv::Mat _Txa = (cv::Mat_<float>(4, 4) << -1.0142f, 0.4264f, 0.1603f, -0.2185f
		, -0.0022f, -0.9322f, -0.3396f, 0.5846f
		, 0.0046f, -0.3448f, 0.9464f, -0.9202f
		, 0.0f, 0.0f, 0.0f, 1.0f);


	// Exjobbsrummet KAMERA nr.2 (sedan 22 juni.)
	cv::Mat _TxCAM2 = (cv::Mat_<float>(4, 4) << -0.9883f, -0.0134f, 0.0273f, 0.0794f
											, 0.0060f, -0.4127f, -0.8941f, 0.9447f
											, 0.0233f, -0.8835f, 0.4080f, -0.2694f
											, 0.0f, 0.0f, 0.0f, 1.0f);

	// New coordinate
	cv::Mat Mat_out =  input.t();// tran_M * _Txs2 * input.t();
	Mat_out = Mat_out.t();
	// Finger new output vector.
	Vector3 output1 = { Mat_out.at<float>(0,0), Mat_out.at<float>(0,1), Mat_out.at<float>(0,2) };

	// [!] Thumb input vector.
	cv::Mat input2 = (cv::Mat_<float>(1, 4) << coord2.x, coord2.y, coord2.z, 1.0f);
	// Calc. new coordinate.
	cv::Mat Mat_out2 =  input2.t(); // tran_M * _Txs2 * input2.t();
	Mat_out2 = Mat_out2.t();
	// Thumb new output vector.
	Vector3 output2 = { Mat_out2.at<float>(0,0), Mat_out2.at<float>(0,1), Mat_out2.at<float>(0,2) };

	// [!] Middle input vector.
	cv::Mat input3 = (cv::Mat_<float>(1, 4) << coord3.x, coord3.y, coord3.z, 1.0f); //-new
	// Calc. new coordinate.
	cv::Mat Mat_out3 =  input3.t();
	Mat_out3 = Mat_out3.t();
	// Thumb new output vector.
	Vector3 output3 = { Mat_out3.at<float>(0,0), Mat_out3.at<float>(0,1), Mat_out3.at<float>(0,2) };

	//########## LAST PART ########### 
	// Transfer both output vector values to global (float) output array.
	for (int i = 0; i < 3; i++) {
		*fp++ = Mat_out.at<float>(0, i);
	}
	for (int i = 0; i < 3; i++) {
		*fp++ = Mat_out2.at<float>(0, i);
	}

	for (int i = 0; i < 3; i++) {
		*fp++ = Mat_out3.at<float>(0, i); //-new
	}


	//*fp++ = p0.x;
	//*fp++ = p0.y;
	//*fp++ = p0.z;
	*fp++ = p1.x;
	*fp++ = p1.y;
	*fp++ = p1.z;
	*fp++ = p2.x;
	*fp++ = p2.y;
	*fp++ = p2.z;
	*fp++ = fBool;





}

void close_all()
{
	// openpose 
	opWrapper.stop();
	// realsense
	p->stop();
	delete p;
	// opencv
	cv::destroyAllWindows();
	// our array 
	//delete[] OUT_ARRAY;
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

/// THREE FINGERS BETA !

