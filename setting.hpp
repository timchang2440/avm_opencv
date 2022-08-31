#ifndef __SETTING_H__
#define __SETTING_H__

/* --------------------------------------------------------------------
 (shift_width, shift_height): how far away the birdview looks outside
 of the calibration pattern in horizontal and vertical directions
*/
extern int shift_w;
extern int shift_h;

// size of the gap between the calibration pattern and the car
// in horizontal and vertical directions
extern int inn_shift_w;
extern int inn_shift_h;

// total width/height of the stitched image
extern int total_w;
extern int total_h;

// four corners of the rectangular region occupied by the car
// top-left (x_left, y_top), bottom-right (x_right, y_bottom)
extern int xl;
extern int xr;
extern int yt;
extern int yb;

extern cv::Size project_shapes[4];

// pixel locations of the four points to be chosen.
// you must click these pixels in the same order when running
// the get_projection_map.py script
extern cv::Size project_keypoints[4][4];

enum {
	CAM_FRONT,
	CAM_BACK,
	CAM_LEFT,
	CAM_RIGHT,
	NUM_CAM,
};

enum {
    IN_TYPE_PIC,
    IN_TYPE_VIDEO,
    IN_TYPE_CAM,
};

enum {
    DISP_QUAD,
    DISP_FRONT,
    DISP_BACK,
    DISP_LEFT,
    DISP_RIGHT,
    DISP_BEV,
};

typedef struct camera_param_t {
    cv::VideoCapture cap;
    cv::Mat camMat;
    cv::Mat distortion;
    cv::Mat resolution;
    cv::Mat projectMat;
    cv::Mat scaleXY;
    cv::Mat shiftXY;
    cv::Mat map[2];
} camera_param;

#endif /* __SETTING_H__ */
