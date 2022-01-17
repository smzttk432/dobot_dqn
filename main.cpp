#define _CRT_SECURE_NO_WARNINGS
#include "opencv2/opencv.hpp"
#include "tiny_dnn/tiny_dnn.h"
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include "agent.h"

int main(int argc, char* argv[]) {

	// 画像データを入れるオブジェクト
	cv::Mat image(256, 256, CV_8UC3);

	// テキストを描画する
	cv::String text = "Hello, world";
	cv::Point org(0, 100);
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1.0;
	cv::Scalar color(0, 255, 127); // Blue, Green, Red
	cv::putText(image, text, org, fontFace, fontScale, color);

	// 画像を出力する
	cv::String path = "hello-world.png";
	cv::imwrite(path, image);

	return 0;
}