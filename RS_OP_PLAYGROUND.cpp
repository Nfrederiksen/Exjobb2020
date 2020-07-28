// ----------------------------- OpenPose C++ API Tutorial - Example 7 - Face from Image -----------------------------
// It reads an image and the hand location, process it, and displays the hand keypoints. In addition,
// it includes all the OpenPose configuration flags.
// Input: An image and the hand rectangle locations.
#include <Windows.h>
#include "tracker.h"
#include <algorithm>
// Third-party dependencies
//#include "realsense.h"

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

// Custom OpenPose flags
// Producer

// This worker will just read and return all the jpg files in a directory
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

void printKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr)
{
	try
	{
		// Example: How to use the pose keypoints
		if (datumsPtr != nullptr && !datumsPtr->empty())
		{
			op::opLog("Body keypoints: " + datumsPtr->at(0)->poseKeypoints.toString(), op::Priority::High);
			op::opLog("Face keypoints: " + datumsPtr->at(0)->faceKeypoints.toString(), op::Priority::High);
			op::opLog("Left hand keypoints: " + datumsPtr->at(0)->handKeypoints[0].toString(), op::Priority::High);
			op::opLog("Right hand keypoints: " + datumsPtr->at(0)->handKeypoints[1].toString(), op::Priority::High);
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

// ## Key Point-Tracker Functions ##
void tracking_by_keypoints(tracker tr)
{
	if (tr.loss_track)
	{	// Move back the boundingbox to center of image.
		tr.default_bbox_creator();
	}
	else
	{

	}
}

// C++ program to calculate average of array elements 
#include <iostream> 
using namespace std;

// Function that return average of an array. 
float average(float a[], int n)
{
	// Find sum of array element 
	int sum = 0;
	for (int i = 0; i < n; i++)
		sum += a[i];

	return sum / n;
}

struct Vector3 {
	float x;
	float y;
	float z;
};



int tutorialApiCpp()
{

	bool lost_tracking = false;
	float FINGER_D_VALUE = 0.0f;
	float OLD_FINGER_D_VALUE = 0.0f;
	float TUM_D_VALUE = 0.0f;
	float OLD_TUM_D_VALUE = 0.0f;
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

	cv::Point KEY_POINT_LOC_PEK;
	cv::Point KEY_POINT_LOC_TUM;
	cv::Point OLD_KEY_POINT_LOC_PEK;
	cv::Point OLD_KEY_POINT_LOC_TUM;

	uint32_t depth_width = 848;
	uint32_t depth_height = 480;
	uint32_t depth_fps = 30;
	
	try
	{
		// Requiredflags to enable heatmaps
		FLAGS_body = 0;
		FLAGS_hand = true;
		FLAGS_hand_detector = 2;

		// Configuring OpenPose
		op::opLog("Configuring OpenPose...", op::Priority::High);
		op::Wrapper opWrapper{ op::ThreadManagerMode::Asynchronous };
		configureWrapper(opWrapper);

		// Starting OpenPose
		op::opLog("Starting thread(s)...", op::Priority::High);
		opWrapper.start();

		//############################################################
		//															##
		cv::setUseOptimized(true);
		// Set Device Config
		rs2::config config;
		config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps); // *NEW* Color stream
		config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps); // Depth Stream
		// Start Pipeline
		pipeline_profile = pipeline.start(config);

		// Retrieve Device
		rs2::device device = pipeline_profile.get_device();
		
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%
		rs2::sensor myColorSensor = device.first<rs2::color_sensor>();
		rs2::sensor myDepthSensor = device.first<rs2::depth_sensor>();
		// Set
		try {
			myColorSensor.set_option(RS2_OPTION_EXPOSURE, 156.0f);
			myDepthSensor.set_option(RS2_OPTION_EXPOSURE, 4000.0f);
		}
		catch (const rs2::invalid_value_error& e) {

			std::cout <<"\nGOOD BYE"<< e.what() << std::endl;
		}
		// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
			depth_table_control.depthClampMin = 50; // Min Depth 1.0m
			depth_table_control.depthClampMax = 1000; // Max Depth 2.0m
			depth_table_control.disparityShift = 0;	// Disparity Mode
			advanced_mode.set_depth_table(depth_table_control);
			std::cout << "\nHELLO line 83";
		
			// e.g. Apply Visual Preset (https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets)

			std::ifstream ifs("../VisualPresets/HandGesturePreset.json");
			if (ifs.is_open()) {
				std::ostringstream oss;
				oss << ifs.rdbuf();
				advanced_mode.load_json(oss.str());
			}
		}
		else
		{
			std::cout << "\nFAIL line 70 IF-sats";
		}


		// Tracker Object!
		tracker t_obj(color_width, color_height);

		while (true)
		{
			// Update Frame
			rs2::frameset frameset = pipeline.wait_for_frames();

			//*NEW*
		// Retrieve Aligned Frame
			rs2::align align(rs2_stream::RS2_STREAM_COLOR);
			aligned_frameset = align.process(frameset);
			if (!aligned_frameset.size()) {
				return 0;
			}
			// Retrieve Color Frame
			color_frame = aligned_frameset.get_color_frame();

			// Retrive Frame Information
			color_width = color_frame.as<rs2::video_frame>().get_width();
			color_height = color_frame.as<rs2::video_frame>().get_height();

			// Retrieve Depth Frame
			depth_frame = aligned_frameset.get_depth_frame();

			// Retrive Frame Size
			depth_width = depth_frame.as<rs2::video_frame>().get_width();
			depth_height = depth_frame.as<rs2::video_frame>().get_height();

			// Create cv::Mat form Color Frame
			color_mat = cv::Mat(color_height, color_width, CV_8UC3, const_cast<void*>(color_frame.get_data()));


			// Read image and hand rectangle locations
			//const cv::Mat cvImageToProcess = cv::imread(FLAGS_image_path);
			const op::Matrix imageToProcess = OP_CV2OPCONSTMAT(color_mat);//cvImageToProcess);

		
			//---------------------------------------------------------------------------------------- Hand tracker bounding box !!!!!!!!!!!!!!!!!!!!!!!
			const std::vector<std::array<op::Rectangle<float>, 2>> handRectangles
			{
				// Left/Right hands of person 0
				std::array<op::Rectangle<float>, 2>{
				op::Rectangle<float>{0.0f, 0.0f, 0.0f, 0.0f}, // Left hand
				//op::Rectangle<float>{533.0f, 138.0f, 250.0f, 250.0f}, // Left hand
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


				float array_x[21];
				float array_y[21];
				float *px, *py;
				auto total_cs = 0.0f;
				for (int i = 0; i < 21; i++)
				{
					auto bi = datumsPtr->at(0)->handKeypoints[1].getSize(2)*(0 * numberHandParts + i);
					array_x[i] = datumsPtr->at(0)->handKeypoints[1][bi];
					array_y[i] = datumsPtr->at(0)->handKeypoints[1][bi+1];
					total_cs += datumsPtr->at(0)->handKeypoints[1][bi+2];
				}

				px = array_x;
				py = array_y;

				total_cs /= 21;

				// %%%%%%%%%%%%%%%% REINITIALIZE OR NOT? %%%%%%%%%%%%%%%%
				if (total_cs <= 0.08f)
				{
					KEY_POINT_LOC_PEK = OLD_KEY_POINT_LOC_PEK;
					KEY_POINT_LOC_TUM = OLD_KEY_POINT_LOC_TUM;
					lost_tracking = true;
					t_obj.default_bbox_creator();
				}
					
				else
				{
					OLD_KEY_POINT_LOC_PEK = KEY_POINT_LOC_PEK;
					OLD_KEY_POINT_LOC_TUM = KEY_POINT_LOC_TUM;

					lost_tracking = false;
					float delta_x = *(std::max_element(px + 0, px + 20)) - *(std::min_element(px + 0,px + 20));
					float delta_y = *(std::max_element(py + 0, py + 20)) - *(std::min_element(py + 0, py + 20));
					auto box_length = std::max(delta_x, delta_y);
					box_length = std::max(box_length, 76.0f);
					// Double the size of box length (Some marginal for the hand).
					box_length *= 2;

					// Find Centroid a.k.a mean x and mean y.
					int center_x = average(array_x, 21);
					int center_y = average(array_y, 21);
					
					// #### TODO #### Assign our current_center[2] in Tracker Object -> center_x and center_y
					t_obj.current_center[0] = center_x;
					t_obj.current_center[1] = center_y;

					// #### TODO #### Now that our Tracker Object has knowlegde of a new "center point" and "crop size" 
					//					a new bounding box can be defined. Use for example: tr.bbox_creator(crop_size).
					t_obj.bbox_creator(box_length);

				}

			}
			else
			{
				lost_tracking = true;
				op::opLog("Nullptr or empty datumsPtr found.", op::Priority::High);

			}
				
			// #### ADD TRACKER STUFF HERE #####

			// LAST THING TO DO TO DEPTH MAT**
			// Create cv::Mat form Depth Frame
			depth_mat = cv::Mat(depth_height, depth_width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));
			rs2::depth_frame fancy_dframe = aligned_frameset.get_depth_frame();







			// Keep last non-zero value incase the d-value suddenly jumps to zero mid-stream.
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
			if (fancy_dframe.get_distance(KEY_POINT_LOC_TUM.x, KEY_POINT_LOC_TUM.y) == 0)
				TUM_D_VALUE = OLD_TUM_D_VALUE;
			else
			{
				// Update the old value w/ the current one.
				OLD_TUM_D_VALUE = TUM_D_VALUE;
				// Update the current one w/ the new one.
				TUM_D_VALUE = fancy_dframe.get_distance(KEY_POINT_LOC_TUM.x, KEY_POINT_LOC_TUM.y);
			}

			// ## Just a print of the COORD ##
			std::cout << "\nFinger Tip D-val: [" << FINGER_D_VALUE << "] @: [" << KEY_POINT_LOC.x << ", " << KEY_POINT_LOC.y << "]";

			cv::Mat scale_mat;
			depth_mat.convertTo(scale_mat, CV_8U, -255.0 / 1000.0, 255.0); // 0-10000 -> 255(white)-0(black)
			cv::applyColorMap(scale_mat, scale_mat, cv::COLORMAP_RAINBOW);
			// Draw a white circle on top of key point.
			cv::circle(scale_mat, KEY_POINT_LOC_PEK, w / 32, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);
			cv::circle(scale_mat, KEY_POINT_LOC_TUM, w / 32, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_8);
			// Show the drawn image.
			cv::imshow("Depth", scale_mat);
			//----|
			if (datumsPtr != nullptr)
			{
				//printKeypoints(datumsPtr);
				//if (!FLAGS_no_display)
				display(datumsPtr, t_obj);
			}
			else
				op::opLog("Image could not be processed.", op::Priority::High);



			if (cv::waitKey(10) == 27)
			{
				std::cout << "Esc key is pressed by user. Stoppig the video" << std::endl;
				pipeline.stop();
				break;
			}
		}

		//RETURN
		return 0;
		//															##
		//############################################################

	}
	catch (const std::exception&)
	{
		std::cout << "\nYou see, oshtin Powersh. I Lawv Gooold";
		return -1;
	}
}

int main(int argc, char *argv[])
{


	// Parsing command line flags
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// Running tutorialApiCpp
	tutorialApiCpp();

	// Close Windows
	cv::destroyAllWindows();
	// Stop Pipline


	return 0;
}
