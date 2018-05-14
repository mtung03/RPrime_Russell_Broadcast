#include <iostream>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <chrono>
#include <string.h>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <mutex>

//#define ADDRESS "192.168.1.116" // RPrime
//#define ADDRESS "38.122.127.226" // Tufts Guest
//#define ADDRESS "130.64.218.78" // Tufts Wireless
#define ADDRESS "130.64.132.179" // Ocucomp in Blake
//#define ADDRESS "71.184.115.76" //Max Address
//#define ADDRESS "130.64.216.210" // max laptop on tufts wireless
#define LPORT 8000
#define RPORT 8001

#define MAX_PACKET_SIZE 65000
#define NUM_ITERATIONS 1000
#define JPEG_QUALITY 40

std::mutex cam_guard;

// converts a sl::Mat to cv::Mat without copying image data
cv::Mat slMat2cvMat(sl::Mat& input) {
    int cv_type = -1;
    switch(input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

int setup_socket(bool isLeft, struct sockaddr_in& target) {
    int s;
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        std::cerr << "cannot create socket\n";
        return -1;
    }
    memset((char *) &target, 0, sizeof(target)); // clear struct
    target.sin_family = AF_INET;
    target.sin_port = htons(isLeft? LPORT : RPORT);

    if (inet_aton(ADDRESS, &target.sin_addr) == 0) {
        std::cerr << "inet aton failed\n";
        return -1;
    }

    return s;
}

void retrieve_encode_send(bool isLeft, sl::Camera *russ, std::vector<int> comp_params) {
    sl::Mat frame;
    cv::Mat cv_frame;
    std::vector<uchar> buf;
 
    struct sockaddr_in target;
    int s = setup_socket(isLeft, target);

    while (true) {
        cam_guard.lock();
        // retrieve sl::Mat image
        russ->retrieveImage(frame, (isLeft ? sl::VIEW_LEFT : sl::VIEW_RIGHT));
        cam_guard.unlock();

        // convert to cv::Mat image
        cv_frame = slMat2cvMat(frame);

        cv::imencode(".jpg", cv_frame, buf, comp_params);

        // if the compressed image is too large for one packet skip the frame
        if (buf.size() > MAX_PACKET_SIZE) {
            std::cerr << "compressed frame too big\n";
            continue;
        }

        // send compressed data
        if (sendto(s, (char *)buf.data(), buf.size(), 0, (struct sockaddr  *) &target, sizeof(target)) == -1) {
            std::cerr <<  "sending failed\n";
            return;
        }
    }
}

 

int main() {
    // setup camera for capture
    sl::Camera russell;
    sl::InitParameters params;
    params.sdk_verbose = false;
    params.camera_resolution = sl::RESOLUTION_HD720;
    params.camera_fps = 30;

    sl::ERROR_CODE e = russell.open(params);
    if (e != sl::SUCCESS) {
        std::cerr << "failed to open\n";
        return 1;
    }

    std::vector<int> comp_params;
    comp_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    comp_params.push_back(JPEG_QUALITY); // out of 100

    sl::RuntimeParameters run_params;
    run_params.enable_depth = false;

    // left and right eyes will be broadcast in separate threads
    std::thread leftEye(retrieve_encode_send, true, &russell, comp_params);
    std::thread rightEye(retrieve_encode_send, false, &russell, comp_params);

    while (true) {
        cam_guard.lock();
        russell.grab(run_params);
        cam_guard.unlock();
    }

    leftEye.join();
    rightEye.join();

    russell.close();

    return 0;
}
