#pragma once
#include <Windows.h>
class tracker
{
public:
	tracker();
	
	tracker(int img_width, int img_height);

	~tracker();

	// Mem
	void bbox_creator(int crop_size);
	void default_bbox_creator();

	// Tracker Variables
	int img_shape[2];
	bool loss_track = false;
	int init_center[2];
	int current_center[2];
	const int default_crop_size = 268;
	int bbox[4];
	int prev_crop_size = default_crop_size;
	const float alpha = 0.2;



};

