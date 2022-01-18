#define _CRT_SECURE_NO_WARNINGS
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>
#include "visuals.h"


void binarize(cv::Mat frame, visuals* vis, int valhmin, int valsmin, int valvmin, int valhmax, int valsmax, int valvmax) {
    vis->dst = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
    cvtColor(vis->dst, vis->hsv, cv::COLOR_BGR2HSV);
    cv::inRange(vis->hsv, cv::Scalar(valhmin, valsmin, valvmin), cv::Scalar(valhmax, valsmax, valvmax), vis->mask);
    cv::minMaxLoc(vis->mask, &vis->min, &vis->max);
    cv::bitwise_and(vis->dst, vis->dst, vis->res, vis->mask);
}