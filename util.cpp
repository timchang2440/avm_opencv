#include "pch.h"
#include "opencv2/opencv.hpp"
#include "setting.hpp"

using namespace std;

cv::Mat HL(cv::Mat &img)
{
    return img(cv::Rect(0, 0, xl, yt));
}

cv::Mat HM(cv::Mat &img)
{
    return img(cv::Rect(xl, 0, xr - xl, yt));
}

cv::Mat HR(cv::Mat &img)
{
    return img(cv::Rect(xr, 0, xl, yt));
}

cv::Mat VU(cv::Mat &img)
{
    return img(cv::Rect(0, 0, xl, yt));
}

cv::Mat VM(cv::Mat &img)
{
    return img(cv::Rect(0, yt, xl, yb -yt));
}

cv::Mat VD(cv::Mat &img)
{
    return img(cv::Rect(0, yb, xl, yt));
}

cv::Mat get_mask(cv::Mat &img)
{
    /*
    Convert an image to a mask array.
    */
	cv::Mat gray, mask;

	cv:cvtColor(img, gray, cv::COLOR_BGR2GRAY);

	cv::threshold(gray, mask, 0, 255.0f, cv::THRESH_BINARY);

	return mask;
}

cv::Mat get_overlap_region_mask(cv::Mat &imgA, cv::Mat &imgB)
{
    /*
    Given two images of the save size, get their overlapping region and
    convert this region to a mask array.
    */
	cv::Mat overlap, mask;

    cv::bitwise_and(imgA, imgB, overlap);

    mask = get_mask(overlap);

    cv::dilate(mask, mask,
    	cv::Mat::ones(2, 2, CV_8U), cv::Point(-1,-1), 2
    );

    return mask;
}

int getMaxAreaContourId(vector <vector<cv::Point>> contours)
{
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function

vector<cv::Point> get_outmost_polygon_boundary(cv::Mat &img)
{
    /*
    Given a mask image with the mask describes the overlapping region of
    two images, get the outmost contour of this region.
    */
 	cv::Mat mask;
	vector<vector<cv::Point>> contours;
	vector<cv::Vec4i> hierarchy;
	vector<cv::Point> polygon;

    mask = get_mask(img);

    cv::dilate(mask, mask,
    	cv::Mat::ones(2, 2, CV_8U), cv::Point(-1,-1), 2
    );

    // With CHAIN_APPROX_SIMPLE length of contours stored is 4
	cv::findContours(mask, contours, hierarchy,
		cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// get the contour with largest area
	vector<cv::Point> C = contours.at(getMaxAreaContourId(contours));

	// polygon approximation
    cv::approxPolyDP(C, polygon, 0.009 * cv::arcLength(C, true), true);

    return polygon;
}

cv::Mat get_weight(cv::Mat &img)
{
     return (get_mask(img) / 255.0);
}

unsigned int where(cv::Mat& mat, unsigned int*** r)
{
    int h = mat.rows;
    int w = mat.cols;
    
    int matchSize = 0;
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            uchar v = mat.at<uchar>(i, j);
            if (v == 255)
            {
                ++matchSize;
            }
        }
    }

    unsigned int** result;
    result = new unsigned int* [2];
    result[0] = new unsigned int[matchSize];
    result[1] = new unsigned int[matchSize];

    unsigned int count = 0;
    for (int i = 0; i < h; ++i)
    {
        for (int j = 0; j < w; ++j)
        {
            uchar v = mat.at<uchar>(i, j);
            if (v == 255)
            {
                result[0][count] = i;
                result[1][count] = j;
                ++count;
            }
        }
    }

    *r = result;
    return matchSize;
}

string openCVType2str(cv::Mat mat) {
    int type = mat.type();

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    string r;
    switch (depth) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

cv::Mat get_weight_mask_matrix(cv::Mat& imA, cv::Mat& imB, float dist_threshold = 5)
{
    string matType = "";
    cv::Mat G = get_mask(imA);
    G.convertTo(G, CV_32F);
    G = G / 255.0f;

    cv::Mat overlapMask = get_overlap_region_mask(imA, imB);

    unsigned int** indices = 0;
    unsigned matchSize = where(overlapMask, &indices);

    cv::Mat overlapMaskInv;
    cv::bitwise_not(overlapMask, overlapMaskInv);

    cv::Mat imA_diff, imB_diff;
    cv::bitwise_and(imA, imA, imA_diff, overlapMaskInv);
    cv::bitwise_and(imB, imB, imB_diff, overlapMaskInv);
    matType = openCVType2str(imA_diff);

    vector<cv::Point> polyA, polyB;
    polyA = get_outmost_polygon_boundary(imA_diff);
    polyB = get_outmost_polygon_boundary(imB_diff);

    float distToA, distToB;
    unsigned int size = matchSize;
    for (int i = 0; i < size; ++i)
    {
        int y = indices[0][i];
        int x = indices[1][i];

        distToB = cv::pointPolygonTest(polyB, cv::Point2d(x, y), true);
        if (distToB < dist_threshold)
        {
            distToA = cv::pointPolygonTest(polyA, cv::Point2d(x, y), true);
            distToB *= distToB;
            distToA *= distToA;
            if ((distToA + distToB) == 0)
            {
                G.at<float>(y, x) = 1;
            }
            else
            {
                G.at<float>(y, x) = distToB / (distToA + distToB);
            }
        }
    }

    // §R°£2ºû°}¦C
    delete[] indices[0];
    delete[] indices[1];
    delete[] indices;  

    return G;
}

int cnt = 0;
cv::Mat gWeights[4];
void merge(cv::Mat &imgA, cv::Mat &imgB, cv::Mat &mergeImg, int position)
{
    bool useOldLogic = false;
    if (useOldLogic)
    {
        cv::Mat3b allOne(imgA.size(), cv::Vec3b(1,1,1));
        cv::Mat G = get_weight(imgA);
        cv::cvtColor(G, G, cv::COLOR_GRAY2BGR);
        mergeImg = (imgA.mul(G) + imgB.mul(allOne - G));
    }
    else
    {
        int aColorB = 0;
        int aColorG = 0;
        int aColorR = 0;

        int bColorB = 0;
        int bColorG = 0;
        int bColorR = 0;

        if (cnt < 4)
        {
            cnt++;
            gWeights[position] = get_weight_mask_matrix(imgA, imgB);
        }

        cv::Mat G = gWeights[position];

        cv::Mat mergeResult(imgA.rows, imgA.cols, CV_8UC3);

        for (int y = 0; y < imgA.rows; ++y)
        {
            for (int x = 0; x < imgA.cols; ++x)
            {
                aColorB = imgA.at<cv::Vec3b>(y, x)[0] * G.at<float>(y, x);
                aColorG = imgA.at<cv::Vec3b>(y, x)[1] * G.at<float>(y, x);
                aColorR = imgA.at<cv::Vec3b>(y, x)[2] * G.at<float>(y, x);

                bColorB = imgB.at<cv::Vec3b>(y, x)[0] * (1 - G.at<float>(y, x));
                bColorG = imgB.at<cv::Vec3b>(y, x)[1] * (1 - G.at<float>(y, x));
                bColorR = imgB.at<cv::Vec3b>(y, x)[2] * (1 - G.at<float>(y, x));

                mergeResult.at<cv::Vec3b>(y, x)[0] = aColorB + bColorB;
                mergeResult.at<cv::Vec3b>(y, x)[1] = aColorG + bColorG;
                mergeResult.at<cv::Vec3b>(y, x)[2] = aColorR + bColorR;
            }
        }

        mergeImg = mergeResult;
    }
}

static double tune(double x)
{
	if(x >= 1)
	    return x * exp((1 - x) * 0.5);
	else
	    return x * exp((1 - x) * 0.8);
}

double get_mean_statistisc(cv::Mat &gray, cv::Mat &mask)
{
    /*
    Get the total values of a gray image in a region defined by a mask matrix.
    The mask matrix must have values either 0 or 1.
    */
	return cv::sum(gray.mul(mask))[0];
}

double mean_luminance_ratio(cv::Mat &grayA, cv::Mat &grayB, cv::Mat &mask)
{
	return get_mean_statistisc(grayA, mask) / get_mean_statistisc(grayB, mask);
}

cv::Mat adjust_luminance(cv::Mat &gray, double factor)
{
    /*
    Adjust the luminance of a grayscale image by a factor.
    */
	cv::Mat output;
    cv::Mat1b white(gray.size(), 255);

    cv::min((gray * factor), white, output);

    return output;
}

void make_luminance_balance(cv::Mat &f, cv::Mat &b, cv::Mat &l, cv::Mat &r)
{
    cv::Mat src1, src2;
	cv::Mat mask[4];
	cv::Mat fbgr[3], bbgr[3], lbgr[3], rbgr[3];

    src1 = HL(f);
    src2 = VU(l);
	mask[0] = get_overlap_region_mask(src1, src2) / 255;

    src1 = HR(f);
    src2 = VU(r);
	mask[1] = get_overlap_region_mask(src1, src2) / 255;

    src1 = HL(b);
    src2 = VD(l);
	mask[2] = get_overlap_region_mask(src1, src2) / 255;

    src1 = HR(b);
    src2 = VD(r);
	mask[3] = get_overlap_region_mask(src1, src2) / 255;

	cv::split(f, fbgr);
	cv::split(b, bbgr);
	cv::split(l, lbgr);
	cv::split(r, rbgr);

    src1 = VU(rbgr[0]);
    src2 = HR(fbgr[0]);
    double a1 = mean_luminance_ratio(src1, src2, mask[1]);

    src1 = VU(rbgr[1]);
    src2 = HR(fbgr[1]);
    double a2 = mean_luminance_ratio(src1, src2, mask[1]);

    src1 = VU(rbgr[2]);
    src2 = HR(fbgr[2]);
    double a3 = mean_luminance_ratio(src1, src2, mask[1]);

    src1 = HR(bbgr[0]);
    src2 = VD(rbgr[0]);
    double b1 = mean_luminance_ratio(src1, src2, mask[3]);

    src1 = HR(bbgr[1]);
    src2 = VD(rbgr[1]);
    double b2 = mean_luminance_ratio(src1, src2, mask[3]);

    src1 = HR(bbgr[2]);
    src2 = VD(rbgr[2]);
    double b3 = mean_luminance_ratio(src1, src2, mask[3]);

    src1 = VD(lbgr[0]);
    src2 = HL(bbgr[0]);
    double c1 = mean_luminance_ratio(src1, src2, mask[2]);

    src1 = VD(lbgr[1]);
    src2 = HL(bbgr[1]);
    double c2 = mean_luminance_ratio(src1, src2, mask[2]);

    src1 = VD(lbgr[2]);
    src2 = HL(bbgr[2]);
    double c3 = mean_luminance_ratio(src1, src2, mask[2]);

    src1 = HL(fbgr[0]);
    src2 = VU(lbgr[0]);
    double d1 = mean_luminance_ratio(src1, src2, mask[0]);

    src1 = HL(fbgr[1]);
    src2 = VU(lbgr[1]);
    double d2 = mean_luminance_ratio(src1, src2, mask[0]);

    src1 = HL(fbgr[2]);
    src2 = VU(lbgr[2]);
    double d3 = mean_luminance_ratio(src1, src2, mask[0]);

    double t1 = pow(a1 * b1 * c1 * d1, 0.25);
    double t2 = pow(a2 * b2 * c2 * d2, 0.25);
    double t3 = pow(a3 * b3 * c3 * d3, 0.25);

    double x1 = t1 / pow(d1 / a1, 0.5);
    double x2 = t2 / pow(d2 / a2, 0.5);
    double x3 = t3 / pow(d3 / a3, 0.5);

    x1 = tune(x1);
    x2 = tune(x2);
    x3 = tune(x3);

    fbgr[0] = adjust_luminance(fbgr[0], x1);
    fbgr[1] = adjust_luminance(fbgr[1], x2);
    fbgr[2] = adjust_luminance(fbgr[2], x3);

    double y1 = t1 / pow(b1 / c1, 0.5);
    double y2 = t2 / pow(b2 / c2, 0.5);
    double y3 = t3 / pow(b3 / c3, 0.5);

    y1 = tune(y1);
    y2 = tune(y2);
    y3 = tune(y3);

    bbgr[0] = adjust_luminance(bbgr[0], y1);
    bbgr[1] = adjust_luminance(bbgr[1], y2);
    bbgr[2] = adjust_luminance(bbgr[2], y3);

    double z1 = t1 / pow(c1 / d1, 0.5);
    double z2 = t2 / pow(c2 / d2, 0.5);
    double z3 = t3 / pow(c3 / d3, 0.5);

    z1 = tune(z1);
    z2 = tune(z2);
    z3 = tune(z3);

    lbgr[0] = adjust_luminance(lbgr[0], z1);
    lbgr[1] = adjust_luminance(lbgr[1], z2);
    lbgr[2] = adjust_luminance(lbgr[2], z3);

    double w1 = t1 / pow(a1 / b1, 0.5);
    double w2 = t2 / pow(a2 / b2, 0.5);
    double w3 = t3 / pow(a3 / b3, 0.5);

    w1 = tune(w1);
    w2 = tune(w2);
    w3 = tune(w3);

    rbgr[0] = adjust_luminance(rbgr[0], w1);
    rbgr[1] = adjust_luminance(rbgr[1], w2);
    rbgr[2] = adjust_luminance(rbgr[2], w3);

    cv::merge(fbgr, 3, f);
    cv::merge(bbgr, 3, b);
    cv::merge(lbgr, 3, l);
    cv::merge(rbgr, 3, r);
}

void make_white_balance(cv::Mat &img)
{
    /*
    Adjust white balance of an image base on the means of its channels.
    */
    cv::Mat bgr[3];

    cv::split(img, bgr);

    double m1 = cv::mean(bgr[0])[0];
    double m2 = cv::mean(bgr[1])[0];
    double m3 = cv::mean(bgr[2])[0];

    double K = (m1 + m2 + m3) / 3;
    double c1 = K / m1;
    double c2 = K / m2;
    double c3 = K / m3;

    bgr[0] = adjust_luminance(bgr[0], c1);
    bgr[1] = adjust_luminance(bgr[1], c2);
    bgr[2] = adjust_luminance(bgr[2], c3);

    cv::merge(bgr, 3, img);
}

void set_blocking(int fd, int should_block, struct termios *pOrigTty)
{
#ifndef WINDOWS
    struct termios tty;

    memset(&tty, 0, sizeof tty);

    tcgetattr(fd, pOrigTty);

    memcpy(&tty, pOrigTty, sizeof tty);

    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,

    if(should_block) {
        tty.c_cc[VMIN]  = 1;
        tty.c_cc[VTIME] = 0;
    } else {
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;
    }

    tcsetattr(fd, TCSANOW, &tty);
#endif // !WINDOWS
}
