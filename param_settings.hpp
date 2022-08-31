#pragma once
#include <string>
#include <opencv2/opencv.hpp>

class ParamSettings
{
public:
	ParamSettings()
	{
		cv::FileStorage fs(FilePath, cv::FileStorage::READ, "utf-8");

		fs["car_w"] >> car_w;
		fs["car_h"] >> car_h;
		fs["grid_w"] >> grid_w;
		fs["grid_h"] >> grid_h;
		fs["mask_w"] >> mask_w;
		fs["mask_h"] >> mask_h;
		fs["shift_w"] >> shift_w;
		fs["shift_h"] >> shift_h;
		fs["fb_xl"] >> fb_xl;
		fs["fb_xr"] >> fb_xr;
		fs["fb_yt"] >> fb_yt;
		fs["fb_yb"] >> fb_yb;
		fs["lr_xl"] >> lr_xl;
		fs["lr_xr"] >> lr_xr;
		fs["lr_yt"] >> lr_yt;
		fs["lr_yb"] >> lr_yb;

		total_w = mask_w + 2 * shift_w;
		total_h = mask_h + 2 * shift_h;

		inn_shift_w = (mask_w - car_w) / 2.0f - grid_w;
		inn_shift_h = (mask_h - car_h) / 2.0f - grid_h;

		xl = shift_w + grid_w + inn_shift_w;
		xr = total_w - xl;
		yt = shift_h + grid_h + inn_shift_h;
		yb = total_h - yt;
	}

	// ÅªÀÉ
	int car_w;
	int car_h;
	int grid_w;
	int grid_h;
	int mask_w;
	int mask_h;
	int shift_w;
	int shift_h;
	int fb_xl;
	int fb_xr;
	int fb_yt;
	int fb_yb;
	int lr_xl;
	int lr_xr;
	int lr_yt;
	int lr_yb;

	// ­pºâ
	int total_w;
	int total_h;
	int inn_shift_w;
	int inn_shift_h;
	int xl;
	int xr;
	int yt;
	int yb;

private:
	const char* FilePath = "./dataset/yaml/param_settings.yaml";
};