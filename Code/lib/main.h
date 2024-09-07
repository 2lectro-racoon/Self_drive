#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <pigpio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>     // Core 기능
#include <opencv2/imgproc.hpp>  // 이미지 처리 기능
#include <opencv2/highgui.hpp>  // GUI 창, 비디오 캡처 등
#include <fstream>
#include <string>
#include <cpr/cpr.h>
#include <nlohmann/json.hpp>
#include <vector>
#include "base64.h"

#define SIGNAL0 24
#define SIGNAL1 23
#define SIGNAL2 18
#define LEDL    17
#define LEDR    27
#define ECHOP    3
#define TRIGP    2
#define RETURN0 20
#define RETURN1 16
#define RETURN2 21


#define LOW     0
#define HIGH    1

#define LEFT_PIXEL_THR  2000
#define RIGHT_PIXEL_THR 2000
#define BOUND           1000
#define TIMEOUT         10000

void videoProcessing();
void signalProcessing();
std::string encode_image(const cv::Mat& frame);
void process_request(const std::string& api_key, const std::string& image_path);

#endif
