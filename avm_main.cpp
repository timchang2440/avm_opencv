#include "pch.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <fcntl.h>    /* For O_RDWR */
#include "setting.hpp"
#include "util.hpp"

#define CVUI_IMPLEMENTATION
#include "cvui.h"

#define NAME "Retronix AVM"

#define KEY_ESC 27
#define KEY_D   'd'
#define KEY_S   's'
#define KEY_V   'v'
#define KEY_Q   'q'
#define KEY_T   't'
#define KEY_M   'm'

#define KEY_0   '0'
#define KEY_1   '1'
#define KEY_2   '2'
#define KEY_3   '3'
#define KEY_4   '4'
#define KEY_5   '5'

#define CAR_IMG "./dataset/input/image/car.png"

#define CTRL_USE_SERIAL

const char *camParamFiles[NUM_CAM] = {
    "./dataset/yaml/front.yaml",
    "./dataset/yaml/back.yaml",
    "./dataset/yaml/left.yaml",
    "./dataset/yaml/right.yaml"
};

const char *inputSrcFiles[NUM_CAM] = {
    "./dataset/input/image/front.png",
    "./dataset/input/image/back.png",
    "./dataset/input/image/left.png",
    "./dataset/input/image/right.png"
};

camera_param camera[NUM_CAM];
cv::Mat bevFrame;
cv::Mat frame[NUM_CAM];

bool gUseMultiThread = true;

tm* GetLocalTime()
{
#ifdef WINDOWS
    tm now;
    __time64_t long_time;
    _time64(&long_time);
    _localtime64_s(&now, &long_time);
    return &now;
#else
    time_t t = time(0);
    struct tm* now = localtime(&t);
    return now;
#endif
}

std::string gstreamer_pipeline(int device, int capture_width, int capture_height, int framerate, int display_width, int display_height) {
    return
        " v4l2src device=/dev/video"+ std::to_string(device) + " !"
        " video/x-raw,format=BGRA,"
        " width=(int)" + std::to_string(capture_width) + ","
        " height=(int)" + std::to_string(capture_height) + ","
        " framerate=(fraction)" + std::to_string(framerate) +"/1 !"
        " videoconvert ! videoscale !"
        " video/x-raw,"
        " width=(int)" + std::to_string(display_width) + ","
        " height=(int)" + std::to_string(display_height) + " ! appsink";
}

static int loadCameraParam(const char *filePath, int camId)
{
    cv::FileStorage fs(filePath, cv::FileStorage::READ);

    if(camId >= NUM_CAM) {
        std::cout << "Error: Camera ID must in 0-3 range." << std::endl;
        return -1;
    }

    fs["camera_matrix"]  >> camera[camId].camMat;
    fs["dist_coeffs"]    >> camera[camId].distortion;
    fs["resolution"]     >> camera[camId].resolution;
    fs["project_matrix"] >> camera[camId].projectMat;
    fs["scale_xy"]       >> camera[camId].scaleXY;
    fs["shift_xy"]       >> camera[camId].shiftXY;

    if (!fs.isOpened()) {
        std::cout << "Error: Couldn't load camera parameters from " << filePath << std::endl;
        return -1;
    } else {
        std::cout << "======== " + std::string(filePath) + " ========" << std::endl;
        std::cout << "camera_matrix "   << camera[camId].camMat << std::endl;
        std::cout << "dist_coeffs "     << camera[camId].distortion << std::endl;
        std::cout << "resolution "      << camera[camId].resolution << std::endl;
        std::cout << "project_matrix "  << camera[camId].projectMat << std::endl;
        std::cout << "scale_xy "        << camera[camId].scaleXY << std::endl;
        std::cout << "shift_xy "        << camera[camId].shiftXY << std::endl;
    }

    fs.release();
    return 0;
}

static void getUndistortRectifyMap(int camId)
{
    cv::Mat camMat = camera[camId].camMat;
    cv::Mat distortion = camera[camId].distortion;
    cv::Mat resolution = camera[camId].resolution;;
    cv::Mat scaleXY = camera[camId].scaleXY;;
    cv::Mat shiftXY = camera[camId].shiftXY;;
    cv::Mat newCamMat;

    camMat.copyTo(newCamMat);

    newCamMat.at<double>(0, 0) *= scaleXY.at<float>(0);
    newCamMat.at<double>(1, 1) *= scaleXY.at<float>(1);
    newCamMat.at<double>(0, 2) += shiftXY.at<float>(0);
    newCamMat.at<double>(1, 2) += shiftXY.at<float>(1);

    cv::fisheye::initUndistortRectifyMap(
        camMat, distortion, cv::Matx33d::eye(),
        newCamMat, cv::Size(resolution.at<int>(0), resolution.at<int>(1)),
        CV_16SC2, camera[camId].map[0], camera[camId].map[1]
    );
}

static void putTextOnFrame(int dispMode, cv::Mat &frame, int time, bool dispFps, bool dispVideoRec)
{
    int x = 10;
    int y = 30;
    int color = 0xFFFFFF;
    double scale = 1.0;

    switch(dispMode) {
        case DISP_QUAD:
            cvui::text(frame, x, y, "Front", scale, color);
            cvui::text(frame, x + (frame.cols / 2), y, "Back", scale, color);
            cvui::text(frame, x + (frame.cols / 2), y + (frame.rows / 2), "Right", scale, color);
            cvui::text(frame, x, y + (frame.rows / 2), "Left", scale, color);
            break;

        case DISP_FRONT:
            cvui::text(frame, x, y, "Front", scale, color);
            break;

        case DISP_BACK:
            cvui::text(frame, x, y, "Back", scale, color);
            break;

        case DISP_LEFT:
            cvui::text(frame, x, y, "Left", scale, color);
            break;

        case DISP_RIGHT:
            cvui::text(frame, x, y, "Right", scale, color);
            break;

        case DISP_BEV:
            cvui::text(frame, x, y, "Bird's eye", scale, color);
            break;
    }

    if(dispFps)
        cvui::printf(frame, x, 2 * y, 0.5, color,
            "Frame process time = %d ms (%.2f fps)", time, 1000.0 / time);

    if(dispVideoRec) {
        cvui::printf(frame, x, 3 * y, 0.5, 0xFF0000, "Video recordig!!!");
    }
}

static cv::Mat getFrameFromInSrc(int camId, int inputType)
{
    cv::Mat frame;

    switch(inputType) {
        default:
        case IN_TYPE_PIC:
            frame = cv::imread(inputSrcFiles[camId]);
            break;
        case IN_TYPE_VIDEO:
            break;
        case IN_TYPE_CAM:
            camera[camId].cap >> frame;
            break;
    }

    return frame;
}

cv::Mat undistort(cv::Mat &img, int camId)
{
    cv::Mat undistortImg;

    cv::remap(img, undistortImg,
        camera[camId].map[0], camera[camId].map[1], cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    return undistortImg;
}

cv::Mat project(cv::Mat &img, int camId)
{
    cv::Mat projectImg;

    cv::warpPerspective(img, projectImg, camera[camId].projectMat, project_shapes[camId], cv::INTER_LINEAR);

    return projectImg;
}

cv::Mat flip(cv::Mat &img, int camId)
{
    cv::Mat flipImg;

    switch(camId) {
        default:
        case CAM_FRONT:
            return img;

        case CAM_BACK:
            cv::flip(img, flipImg, -1);
            break;
        case CAM_LEFT:
            cv::transpose(img, flipImg);
            cv::flip(flipImg, flipImg, 0);
            break;
        case CAM_RIGHT:
            cv::transpose(img, flipImg);
            cv::flip(flipImg, flipImg, 1);
            break;
    }

    return flipImg;
}

cv::Mat stitch(cv::Mat &f, cv::Mat &b, cv::Mat &l, cv::Mat &r)
{
    cv::Mat src1, src2, dst;
    cv::Rect rect;

    // Create a black image
    cv::Mat3b img(total_h, total_w, cv::Vec3b(0,0,0));

    // Front Middle
    rect = cv::Rect(xl, 0, (xr - xl), yt);
    HM(f).copyTo(img(rect));

    // Back Middle
    rect = cv::Rect(xl, yb, xr - xl, total_h - yb);
    HM(b).copyTo(img(rect));

    // Left Middle
    rect = cv::Rect(0, yt, xl, yb - yt);
    VM(l).copyTo(img(rect));

    // Right Middle
    rect = cv::Rect(xr, yt, total_w - xr, yb - yt);
    VM(r).copyTo(img(rect));

    // Front Left
    src1 = HL(f);
    src2 = VU(l);
    merge(src1, src2, dst, 0);
    dst.copyTo(img(cv::Rect(0, 0, xl, yt)));

    // Front Right
    src1 = HR(f);
    src2 = VU(r);
    merge(src1, src2, dst, 1);
    dst.copyTo(img(cv::Rect(xr, 0, total_w - xr, yt)));

    // Back Left
    src1 = HL(b);
    src2 = VD(l);
    merge(src1, src2, dst, 2);
    dst.copyTo(img(cv::Rect(0, yb, xl, total_h - yb)));

    // Back Right
    src1 = HR(b);
    src2 = VD(r);
    merge(src1, src2, dst, 3);
    dst.copyTo(img(cv::Rect(xr, yb, total_w - xr, total_h - yb)));

    return img;
}

void copyCar(cv::Mat &img)
{
    cv::Mat car = cv::imread(CAR_IMG);

    cv::resize(car, car, cv::Size(xr - xl, yb - yt));

    car.copyTo(img(cv::Rect(xl, yt, xr - xl, yb - yt)));
}

static cv::Mat dispOneCam(int camId, bool undistortion)
{
    cv::Mat frame = getFrameFromInSrc(camId, INPUT_SOURCE);

    if(undistortion)
        frame = undistort(frame, camId);

    return frame;
}

static cv::Mat dispQuadCam(bool undistortion)
{
    cv::Mat frame[NUM_CAM];
    cv::Mat hFrame0, hFrame1, quadFrame;

    for(int i = 0; i < NUM_CAM; i++) {
        frame[i] = getFrameFromInSrc(i, INPUT_SOURCE);

        if(undistortion)
            frame[i] = undistort(frame[i], i);

        cv::resize(frame[i], frame[i], frame[i].size() / 2);
    }

    cv::hconcat(frame[0], frame[1], hFrame0);
    cv::hconcat(frame[2], frame[3], hFrame1);
    cv::vconcat(hFrame0, hFrame1, quadFrame);

    return quadFrame;
}

void* CreateFrame(void* pIndex)
{
    int index = *((int*)pIndex);

#ifndef WINDOWS
    //int cpu_id = index; // set thread to cpu3
    //cpu_set_t cpuset;
    //CPU_ZERO(&cpuset);
    //CPU_SET(cpu_id, &cpuset);
    //sched_setaffinity(0, sizeof(cpuset), &cpuset);
#endif // WINDOWS
    
    printf("t%d thread sched_getcpu = %d\n", index, sched_getcpu());
    usleep(1);
    printf("t%d thread sched_getcpu = %d\n", index, sched_getcpu());

    frame[index] = getFrameFromInSrc(index, INPUT_SOURCE);
    frame[index] = undistort(frame[index], index);
    frame[index] = project(frame[index], index);
    frame[index] = flip(frame[index], index);

#ifdef WINDOWS
    return nullptr;
#else
    pthread_exit(NULL);
#endif // WINDOWS
}

static cv::Mat dispBEV(bool undistortion)
{
    if (gUseMultiThread)
    {
#ifdef WINDOWS
        int i0 = 0;
        int i1 = 1;
        int i2 = 2;
        int i3 = 3;
        std::thread f0(CreateFrame, (void*)&i0);
        std::thread f1(CreateFrame, (void*)&i1);
        std::thread f2(CreateFrame, (void*)&i2);
        std::thread f3(CreateFrame, (void*)&i3);

        {
            LogProcessTime logProcessTime("f0");
            f0.join();
        }

        f1.join();
        f2.join();
        f3.join();
#else
        int i0 = 0;
        int i1 = 1;
        int i2 = 2;
        int i3 = 3;
        pthread_t t0, t1, t2, t3; // �ŧi pthread �ܼ�
        pthread_create(&t0, NULL, CreateFrame, (void*)&i0); // �إߤl�����
        pthread_create(&t1, NULL, CreateFrame, (void*)&i1); // �إߤl�����
        pthread_create(&t2, NULL, CreateFrame, (void*)&i2); // �إߤl�����
        pthread_create(&t3, NULL, CreateFrame, (void*)&i3); // �إߤl�����

        // �]�w�ܤ��P��CPU
        int cpu_id0 = 0;
        cpu_set_t cpuset0;
        CPU_ZERO(&cpuset0);
        CPU_SET(cpu_id0, &cpuset0);
        pthread_setaffinity_np(t0, sizeof(cpuset0), &cpuset0);

        int cpu_id1 = 1;
        cpu_set_t cpuset1;
        CPU_ZERO(&cpuset1);
        CPU_SET(cpu_id1, &cpuset1);
        pthread_setaffinity_np(t1, sizeof(cpuset1), &cpuset1);

        int cpu_id2 = 2;
        cpu_set_t cpuset2;
        CPU_ZERO(&cpuset2);
        CPU_SET(cpu_id2, &cpuset2);
        pthread_setaffinity_np(t2, sizeof(cpuset2), &cpuset2);

        int cpu_id3 = 3;
        cpu_set_t cpuset3;
        CPU_ZERO(&cpuset3);
        CPU_SET(cpu_id3, &cpuset3);
        pthread_setaffinity_np(t3, sizeof(cpuset3), &cpuset3);

        {
            LogProcessTime logProcessTime("dispBEV Process Time = ");

            pthread_join(t0, NULL); // ���ݤl��������槹��
            pthread_join(t1, NULL); // ���ݤl��������槹��
            pthread_join(t2, NULL); // ���ݤl��������槹��
            pthread_join(t3, NULL); // ���ݤl��������槹��
        }
#endif // WINDOWS
    }
    else
    {
        for (int i = 0; i < NUM_CAM; i++) {
            frame[i] = getFrameFromInSrc(i, INPUT_SOURCE);
            frame[i] = undistort(frame[i], i);
            frame[i] = project(frame[i], i);
            frame[i] = flip(frame[i], i);
        }
    }

//    make_luminance_balance(frame[0], frame[1], frame[2], frame[3]);

    bevFrame = stitch(frame[0], frame[1], frame[2], frame[3]);

//    make_white_balance(bevFrame);

    copyCar(bevFrame);

    cv::resize(bevFrame, bevFrame, bevFrame.size() / 2);

    return bevFrame;
}

int main(int argc, char** argv)
{
    char key = 0;
    double width;
    double height;
    double fps = 30;
    bool undistortionEnable = false;
    bool snapshotEnable = false;
    bool videoRecEnable = false;
    bool dispFps = false;
    int dispMode = DISP_QUAD;

    cv::VideoWriter writer[4];
    cv::Mat finalFrame[4];
    cv::Mat hFrame0, hFrame1, quadFrame;
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;
    int processTime;

#ifndef WINDOWS
#ifdef CTRL_USE_SERIAL
    struct termios origTty;

    int fd = open("/dev/ttySC0", O_RDWR | O_NOCTTY | O_SYNC);

    if(fd < 0)
    {
        return -1;
    }

    set_blocking(fd, 0, &origTty);
#endif /* CTRL_USE_SERIAL */
#endif // !WINDOWS

    cvui::init(NAME);

    for(int i = 0; i < NUM_CAM; i++) {
        loadCameraParam(camParamFiles[i], i);

        getUndistortRectifyMap(i);

        if(INPUT_SOURCE == IN_TYPE_CAM) {
            camera[i].cap.open(i, cv::CAP_V4L2);
            camera[i].cap.set(cv::CAP_PROP_FPS, 30);
            camera[i].cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
            camera[i].cap.set(cv::CAP_PROP_FRAME_HEIGHT, 800);
            camera[i].cap.set(cv::CAP_PROP_CONVERT_RGB, 1);
        }
    }

    width  = camera[0].resolution.at<int>(0);
    height = camera[0].resolution.at<int>(1);

    for(;;) {
        start = std::chrono::steady_clock::now();

        // switch(dispMode) {
        //     case DISP_QUAD:
        //         finalFrame = dispQuadCam(undistortionEnable);
        //         break;

        //     case DISP_FRONT:
        //         finalFrame = dispOneCam(CAM_FRONT, undistortionEnable);
        //         break;

        //     case DISP_BACK:
        //         finalFrame = dispOneCam(CAM_BACK, undistortionEnable);
        //         break;

        //     case DISP_LEFT:
        //         finalFrame = dispOneCam(CAM_LEFT, undistortionEnable);
        //         break;

        //     case DISP_RIGHT:
        //         finalFrame = dispOneCam(CAM_RIGHT, undistortionEnable);
        //         break;

        //     case DISP_BEV:
        //         finalFrame = dispBEV(undistortionEnable);
        //         break;
        // }
        finalFrame[0] = dispOneCam(CAM_FRONT, undistortionEnable);
        finalFrame[1] = dispOneCam(CAM_BACK, undistortionEnable);
        finalFrame[2] = dispOneCam(CAM_LEFT, undistortionEnable);
        finalFrame[3] = dispOneCam(CAM_RIGHT, undistortionEnable);
        for(int i = 0;i < 4;i++)
            cv::resize(finalFrame[i], finalFrame[i], finalFrame[i].size() / 2);    
        cv::hconcat(finalFrame[0], finalFrame[1], hFrame0);
        cv::hconcat(finalFrame[2], finalFrame[3], hFrame1);
        cv::vconcat(hFrame0, hFrame1, quadFrame);

        end = std::chrono::steady_clock::now();
        processTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        if(videoRecEnable) {
            for(int i = 0;i < 4;i++){
                if(!writer[i].isOpened()) {
                    // get time now
                    struct tm* now = GetLocalTime();

                    char fileName[80];
                    //strftime (fileName, sizeof fileName, "./dataset/record/%F-%H-%M-%S.avi",now);
                    writer[i].open(argv[i+1],
                            cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
                            fps, cv::Size(width, height));
                }
                writer[i] << finalFrame[i];
            }
            
        } else {
            for(int i = 0;i < 4;i++){
                if(writer[i].isOpened()) {
                    writer[i].release();
                }
            }
        }

        //putTextOnFrame(dispMode, finalFrame, processTime, dispFps, videoRecEnable);

        cvui::imshow(NAME, quadFrame);

        if(snapshotEnable) {
            // get time now
            //struct tm* now = GetLocalTime();

            //char fileName[80];
            //strftime (fileName, sizeof fileName, "./dataset/snapshot/%F-%H-%M-%S.png",now);
            for(int i = 0;i < 4;i++){

                imwrite(argv[i+1], finalFrame[i]);
                
            }
            snapshotEnable = false;
        }

        key = cv::waitKey(1);

#ifndef WINDOWS
#ifdef CTRL_USE_SERIAL
        int n = read(fd, &key, 1);
#endif /* CTRL_USE_SERIAL */
#endif // !WINDOWS
        switch(key) {
            case KEY_D:
                undistortionEnable ^= 1;
                break;
            case KEY_S:
                snapshotEnable = true;
                break;
            case KEY_V:
                videoRecEnable ^= 1;
                break;
            case KEY_T:
                dispFps ^= 1;
                break;
            case KEY_M:
                gUseMultiThread = !gUseMultiThread;
                break;
            case KEY_0:
                dispMode = DISP_QUAD;
                break;
            case KEY_1:
                dispMode = DISP_FRONT;
                break;
            case KEY_2:
                dispMode = DISP_BACK;
                break;
            case KEY_3:
                dispMode = DISP_LEFT;
                break;
            case KEY_4:
                dispMode = DISP_RIGHT;
                break;
            case KEY_5:
                dispMode = DISP_BEV;
                break;

            case KEY_Q:
            case KEY_ESC:
#ifndef WINDOWS
#ifdef CTRL_USE_SERIAL
                tcsetattr(fd, TCSANOW, &origTty);
#endif /* CTRL_USE_SERIAL */
#endif // !WINDOWS
                for(int i = 0; i < NUM_CAM; i++) {
                    camera[i].cap.release();
                }
                return 0;
        }
    }
    return 0;
}
