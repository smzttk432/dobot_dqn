#define _CRT_SECURE_NO_WARNINGS
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#pragma comment(lib, "DobotDll.lib")
#include <stdio.h>
#include "DobotDll.h"
#include "dobotcontrol.h"
#include "tiny_dnn/tiny_dnn.h"
#include <iostream>
#include <random>
#include <vector>
#include <cmath>
#include "agent.h"

int valhmin = 99;
void changehmin(int position, void* userdata); //trackbar�R�[���o�b�N�֐��iR�̕ύX�j
int valsmin = 78;
void changesmin(int position, void* userdata); //trackbar�R�[���o�b�N�֐��iG�̕ύX�j
int valvmin = 27;
void changevmin(int position, void* userdata); //trackbar�R�[���o�b�N�֐��iB�̕ύX�j
int valhmax = 136;
void changehmax(int position, void* userdata); //trackbar�R�[���o�b�N�֐��iR�̕ύX�j
int valsmax = 255;
void changesmax(int position, void* userdata); //trackbar�R�[���o�b�N�֐��iG�̕ύX�j
int valvmax = 255;
void changevmax(int position, void* userdata); //trackbar�R�[���o�b�N�֐��iB�̕ύX

int main(void)
{
    ////dobot_move();
    cv::VideoCapture cap(0);//�f�o�C�X�̃I�[�v��
    if (!cap.isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
    {
        //�ǂݍ��݂Ɏ��s�����Ƃ��̏���
        return -1;
    }
    else {
        printf("Device open!!\n");
    }
    /*qlearn�֘A*/
    tiny_dnn::network<tiny_dnn::sequential> mynet;
    mynet << tiny_dnn::fully_connected_layer(MAXX * MAXY, 256, false) << tiny_dnn::relu_layer()
        << tiny_dnn::fully_connected_layer(256, 4, false) << tiny_dnn::tanh_layer();
    agent age;
    tiny_dnn::adam optim;
    tiny_dnn::vec_t act;
    tiny_dnn::vec_t s;
    int i, j;
    int action;
    float reward;
    float bdist, ndist;
    int nx, ny;
    int gen = 0;
    srand((unsigned int)time(NULL));
    vector<tiny_dnn::vec_t> state, rewards;
    /*Dobot�֘A*/
    bool m_bConnectStatus = false;
    bool connection = false;
    connection = dobotconnect(m_bConnectStatus, connection);
    HOMEParams home;
    HOMECmd homecommand;
    PTPCmd cmd;
    Pose pose;
    cmd.ptpMode = 2;
    GetPose(&pose);
    uint64_t queuedCmdIndex = 0;
    uint64_t executedCmdIndex = 0;
    int pointx, pointy;
    cmd.x = pose.x;
    cmd.y = pose.y;
    cmd.z = pose.z;
    pointx = cmd.x;
    pointy = cmd.y;
    printf("%f %f %f\n", cmd.x, cmd.y, cmd.z);
    /*�����܂�Dobot�֘A*/
    double min, max;
    cv::Mat frame, hsv, mask, res, dst;
    cv::Mat labelImage, statres, centroidsres; //�t���[��,�O���[�X�P�[��,�G�b�W���o�p�I�u�W�F�N�g
    cv::Point grav;
    cv::namedWindow("state");
    //�g���b�N�o�[�̐ݒu
    cv::createTrackbar("H_MIN", "state", &valhmin, 255, changehmin);
    cv::createTrackbar("S_MIN", "state", &valsmin, 255, changesmin);
    cv::createTrackbar("V_MIN", "state", &valvmin, 255, changevmin);
    cv::createTrackbar("H_MAX", "state", &valhmax, 255, changehmax);
    cv::createTrackbar("S_MAX", "state", &valsmax, 255, changesmax);
    cv::createTrackbar("V_MAX", "state", &valvmax, 255, changevmax);
    while (cap.read(frame)) { //���[�v
        dst = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
        cvtColor(dst, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(valhmin, valsmin, valvmin), cv::Scalar(valhmax, valsmax, valvmax), mask);
        cv::minMaxLoc(mask, &min, &max);
        cv::bitwise_and(dst, dst, res, mask);
        printf("clear\n");
        state.clear();
        rewards.clear();
        int nLabels = cv::connectedComponentsWithStats(mask, labelImage, statres, centroidsres, 8, 4);
        for (int i = 1; i < nLabels; ++i) {
            double* param = centroidsres.ptr<double>(1);
            
            int x = static_cast<int>(param[0]);
            int y = static_cast<int>(param[1]);
            age.setposition((((x)-(x % 48))), (y - (y % 48)));
            //state = ((((y - (y % 48)) * 10) + (((x)-(x % 48)))) / 48) % 100;
            printf("%d, %d\n", x, y);
            if (learn_end(age.now_pos(), gen) != 1) {
                s = age.pos_vec();
                gen += 1;
                if (rand() % 10 < 3) {
                    action = (rand() % 4);
                }
                else {
                    act = mynet.predict(age.pos_vec());
                    action = act_by_net(act);   
                }
                bdist = age.dist_goal();
                state.push_back(age.pos_vec());
                // snext = nexts(s, a);
                while (s == age.pos_vec()) {
                    if (action == 3) cmd.x += 2;
                    if (action == 2) cmd.x -= 2;
                    if (action == 1) cmd.y += 2;
                    if (action == 0) cmd.y -= 2;
                    ptpmove(executedCmdIndex, queuedCmdIndex, cmd);
                    cap.read(frame);
                    cvtColor(dst, hsv, cv::COLOR_BGR2HSV);
                    cv::inRange(hsv, cv::Scalar(valhmin, valsmin, valvmin), cv::Scalar(valhmax, valsmax, valvmax), mask);
                    cv::minMaxLoc(mask, &min, &max);
                    cv::bitwise_and(dst, dst, res, mask);
                    int nLabels = cv::connectedComponentsWithStats(mask, labelImage, statres, centroidsres, 8, 4);
                    double* param = centroidsres.ptr<double>(1);
                    age.setposition((static_cast<int>(param[0]) % 10), (static_cast<int>(param[1]) % 10));
                    cv::imshow("canny", mask);
                }
                ndist = age.dist_goal();
                reward = compute_reward(bdist, ndist, age.now_pos());
                rewards.push_back(rewards_vec(action, reward));
            }
        }
        printf("train\n");
        mynet.train<tiny_dnn::mse>(optim, state, rewards, 1, 1);
        //row_grav=centroidsres.at<double>(1,1);
        //centroidsres(1, 1);
        cv::imshow("canny", mask); //�摜�̕\��



        int key = cv::waitKey(1); //�L�[����
        if (key == 'q') {

            cv::destroyWindow("canny"); //�E�B���h�E�����
            cap.release();
            break; //while���[�v���甲����
        }
        else if (key == 's'/*115*/)//s�������ꂽ�Ƃ�
        {
            //�t���[���摜��ۑ�����D
            cv::imwrite("img.png", frame);
        }
    }
    //res=cv::Mat::zeros(res);
    cv::destroyAllWindows();

    //SetHOMECmd(&homecommand, true, &queuedCmdIndex);
    //homemove(homecommand, executedCmdIndex, queuedCmdIndex);
    dobotdisconnect(connection);
    return 0;
}

//trackbar�R�[���o�b�N�֐��iR�̕ύX�j
void changehmin(int position, void* userdata)
{
    valhmin = position; //�g���b�N�o�[�̈ʒu�ɂ����R�̒l���X�V
}
//trackbar�R�[���o�b�N�֐��iG�̕ύX�j
void changesmin(int position, void* userdata)
{
    valsmin = position; //�g���b�N�o�[�̈ʒu�ɂ����G�̒l���X�V
}
// trackbar�R�[���o�b�N�֐��iB�̕ύX�j
void changevmin(int position, void* userdata)
{
    valvmin = position; //�g���b�N�o�[�̈ʒu�ɂ����B�̒l���X�V
}

void changehmax(int position, void* userdata)
{
    valhmax = position; //�g���b�N�o�[�̈ʒu�ɂ����R�̒l���X�V
}
//trackbar�R�[���o�b�N�֐��iG�̕ύX�j
void changesmax(int position, void* userdata)
{
    valsmax = position; //�g���b�N�o�[�̈ʒu�ɂ����G�̒l���X�V
}
//trackbar�R�[���o�b�N�֐��iB�̕ύX�j
void changevmax(int position, void* userdata)
{
    valvmax = position; //�g���b�N�o�[�̈ʒu�ɂ����B�̒l���X�V
}