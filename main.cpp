#define _CRT_SECURE_NO_WARNINGS
#include "opencv2/opencv.hpp"

int main(int argc, char* argv[]) {

	// �摜�f�[�^������I�u�W�F�N�g
	cv::Mat image(256, 256, CV_8UC3);

	// �e�L�X�g��`�悷��
	cv::String text = "Hello, world";
	cv::Point org(0, 100);
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 1.0;
	cv::Scalar color(0, 255, 127); // Blue, Green, Red
	cv::putText(image, text, org, fontFace, fontScale, color);

	// �摜���o�͂���
	cv::String path = "hello-world.png";
	cv::imwrite(path, image);

	return 0;
}