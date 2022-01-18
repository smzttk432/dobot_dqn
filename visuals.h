#pragma once
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>

struct visuals {
    double min, max;
    cv::Mat frame, hsv, mask, res, dst;
    cv::Mat labelImage, statres, centroidsres;
    cv::Point grav;
    int nLabels;
};

void binarize(cv::Mat frame, visuals* vis, int valhmin, int valsmin, int valvmin, int valhmax, int valsmax, int valvmax);