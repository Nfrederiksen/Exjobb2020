#include "pch.h"
#include "tracker.h"

tracker::tracker()
{
	// The input image frame size (resolution).
	img_shape[0] = 0;
	img_shape[1] = 0;
	init_center[0] = 0;
	init_center[1] = 0;
}

tracker::tracker(int img_width, int img_height)
{
	img_shape[0] = img_width;
	img_shape[1] = img_height;
	init_center[0] = img_width / 2;
	init_center[1] = img_height / 2;

	this->default_bbox_creator();
}

tracker::~tracker()
{

}

void tracker::bbox_creator(int crop_size)
{
	// Update prev_crop_size using the newly calculated crop_size.
	prev_crop_size = alpha *crop_size + (1-alpha)*prev_crop_size;

	// Define the new bounding box around the current center point.
	int cx = current_center[0] - (prev_crop_size / 2);
	if (cx < 0) cx = 0;
	int cy = current_center[1] - (prev_crop_size / 2);
	if (cy < 0) cy = 0;

	if (current_center[0] + (prev_crop_size / 2) > img_shape[0] - 1)
	{
		cx = img_shape[0] - prev_crop_size;
	}
	else if (current_center[1] + (prev_crop_size / 2) > img_shape[1] - 1)
	{
		cy = img_shape[1] - prev_crop_size;
	}

	bbox[0] = cx;
	bbox[1] = cy;
	bbox[2] = prev_crop_size;
	bbox[3] = prev_crop_size;
}

void tracker::default_bbox_creator()
{
	bbox[0] = init_center[0] - (img_shape[1] / 4);
	bbox[1] = init_center[1] - (img_shape[1] / 4);
	bbox[2] = default_crop_size;
	bbox[3] = default_crop_size;

	current_center[0] = init_center[0];
	current_center[1] = init_center[1];

}
