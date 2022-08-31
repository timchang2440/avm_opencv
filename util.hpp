#ifndef __UTIL_H__
#define __UTIL_H__

void merge(cv::Mat &imgA, cv::Mat &imgB, cv::Mat &mergeImg, int position);
void make_luminance_balance(cv::Mat &f, cv::Mat &b, cv::Mat &l, cv::Mat &r);
void make_white_balance(cv::Mat &img);

cv::Mat HL(cv::Mat &img);
cv::Mat HM(cv::Mat &img);
cv::Mat HR(cv::Mat &img);
cv::Mat VU(cv::Mat &img);
cv::Mat VM(cv::Mat &img);
cv::Mat VD(cv::Mat &img);

void set_blocking(int fd, int should_block, struct termios *pOrigTty);

#endif /* __UTIL_H__ */
