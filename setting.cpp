#include <opencv2/opencv.hpp>
#include "param_settings.hpp"

ParamSettings paramSettings;

/* --------------------------------------------------------------------
 (shift_width, shift_height): how far away the birdview looks outside
 of the calibration pattern in horizontal and vertical directions
*/
int shift_w = paramSettings.shift_w;
int shift_h = paramSettings.shift_h;

int inn_shift_w = paramSettings.inn_shift_w;
int inn_shift_h = paramSettings.inn_shift_h;

// total width/height of the stitched image
int total_w = paramSettings.total_w;
int total_h = paramSettings.total_h;

int xl = paramSettings.xl;
int xr = paramSettings.xr;
int yt = paramSettings.yt;
int yb = paramSettings.yb;

cv::Size project_shapes[4] = {
	cv::Size(total_w, yt),
	cv::Size(total_w, yt),
	cv::Size(total_h, xl),
	cv::Size(total_h, xl)};

cv::Size project_keypoints[4][4] = {
	// front
	{
		cv::Size(shift_w + paramSettings.fb_xl, shift_h + paramSettings.fb_yt),
		cv::Size(shift_w + paramSettings.fb_xr, shift_h + paramSettings.fb_yt),
		cv::Size(shift_w + paramSettings.fb_xl, shift_h + paramSettings.fb_yb),
		cv::Size(shift_w + paramSettings.fb_xr, shift_h + paramSettings.fb_yb),
	},
	// back
	{
		cv::Size(shift_w + paramSettings.fb_xl, shift_h + paramSettings.fb_yt),
		cv::Size(shift_w + paramSettings.fb_xr, shift_h + paramSettings.fb_yt),
		cv::Size(shift_w + paramSettings.fb_xl, shift_h + paramSettings.fb_yb),
		cv::Size(shift_w + paramSettings.fb_xr, shift_h + paramSettings.fb_yb),
	},
	// left
	{
		cv::Size(shift_h + paramSettings.lr_xl, shift_w + paramSettings.lr_yt),
		cv::Size(shift_h + paramSettings.lr_xr, shift_w + paramSettings.lr_yt),
		cv::Size(shift_h + paramSettings.lr_xl, shift_w + paramSettings.lr_yb),
		cv::Size(shift_h + paramSettings.lr_xr, shift_w + paramSettings.lr_yb),

	},
	// right
	{
		cv::Size(shift_h + paramSettings.lr_xl, shift_w + paramSettings.lr_yt),
		cv::Size(shift_h + paramSettings.lr_xr, shift_w + paramSettings.lr_yt),
		cv::Size(shift_h + paramSettings.lr_xl, shift_w + paramSettings.lr_yb),
		cv::Size(shift_h + paramSettings.lr_xr, shift_w + paramSettings.lr_yb),
	}};