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
#include "visuals.h"

int valhmin = 99;
void changehmin(int position, void* userdata); //trackbarコールバック関数（Rの変更）
int valsmin = 78;
void changesmin(int position, void* userdata); //trackbarコールバック関数（Gの変更）
int valvmin = 27;
void changevmin(int position, void* userdata); //trackbarコールバック関数（Bの変更）
int valhmax = 136;
void changehmax(int position, void* userdata); //trackbarコールバック関数（Rの変更）
int valsmax = 255;
void changesmax(int position, void* userdata); //trackbarコールバック関数（Gの変更）
int valvmax = 255;
void changevmax(int position, void* userdata); //trackbarコールバック関数（Bの変更

int main(void)
{
    ////dobot_move();
    cv::VideoCapture cap(0);//デバイスのオープン
    if (!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
    {
        //読み込みに失敗したときの処理
        return -1;
    }
    else {
        printf("Device open!!\n");
    }
    InitRand();
    /*qlearn関連*/
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
    /*Dobot関連*/
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
    /*ここまでDobot関連*/
    visuals vis;
    cv::Mat frame;
    cv::namedWindow("state");
    //トラックバーの設置
    cv::createTrackbar("H_MIN", "state", &valhmin, 255, changehmin);
    cv::createTrackbar("S_MIN", "state", &valsmin, 255, changesmin);
    cv::createTrackbar("V_MIN", "state", &valvmin, 255, changevmin);
    cv::createTrackbar("H_MAX", "state", &valhmax, 255, changehmax);
    cv::createTrackbar("S_MAX", "state", &valsmax, 255, changesmax);
    cv::createTrackbar("V_MAX", "state", &valvmax, 255, changevmax);
    for (i = 0; i < 1000; i++) {
        gen = 0;
        printf("clear\n");
        state.clear();
        rewards.clear();
        cap.read(frame);
        frame = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
        binarize(frame, &vis, valhmin, valsmin, valvmin, valhmax, valsmax, valvmax);
        printf("%d\n", vis.nLabels);
        while (vis.nLabels == 1) {
            cmd.x = pointx + (random_int(20) * rand2());
            cmd.y = pointy + (random_int(20) * rand2());
            ptpmove(executedCmdIndex, queuedCmdIndex, cmd);
            cap.read(frame);
            frame = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
            binarize(frame, &vis, valhmin, valsmin, valvmin, valhmax, valsmax, valvmax);
            cv::imshow("canny", vis.mask);
        }
        double* param = vis.centroidsres.ptr<double>(1);
        int x = static_cast<int>(param[0]);
        int y = static_cast<int>(param[1]);
        age.setposition((((x)-(x % MAXX)) / (480 / MAXX)), (y - (y % MAXY)) / (480 / MAXY));
        while (check_gen(age.now_pos(),gen,vis.nLabels)) {
            cap.read(frame);
            frame = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
            binarize(frame, &vis, valhmin, valsmin, valvmin, valhmax, valsmax, valvmax);
            param = vis.centroidsres.ptr<double>(1);
            x = static_cast<int>(param[0]);
            y = static_cast<int>(param[1]);
            age.setposition((((x)-(x % MAXX)) / (480 / MAXX)), (y - (y % MAXY)) / (480 / MAXY));
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
            printf("%d\n", action);
            while (s == age.pos_vec()&&vis.nLabels!=1) {
                if (action == 3) cmd.x += 2;
                if (action == 2) cmd.x -= 2;
                if (action == 1) cmd.y += 2;
                if (action == 0) cmd.y -= 2;
                ptpmove(executedCmdIndex, queuedCmdIndex, cmd);
                cap.read(frame);
                frame = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
                binarize(frame, &vis, valhmin, valsmin, valvmin, valhmax, valsmax, valvmax);
                param = vis.centroidsres.ptr<double>(1);
                x = static_cast<int>(param[0]);
                y = static_cast<int>(param[1]);
                age.setposition((((x)-(x % MAXX)) / (480 / MAXX)), (y - (y % MAXY)) / (480 / MAXY));
                cv::imshow("canny", vis.mask);
            }
            if (vis.nLabels == 1) {
                reward = -10.0;
            }
            else {
                ndist = age.dist_goal();
                reward = compute_reward(bdist, ndist, age.now_pos());
            }
            rewards.push_back(rewards_vec(action, reward));
            printf("%d, %d\n", (((x)-(x % 48))/10), (y - (y % 48))/10);
            cv::imshow("canny", vis.mask);
            gen++;
        }
        printf("train\n");
        mynet.train<tiny_dnn::mse>(optim, state, rewards, 1, 1);
        while (check_goal(age.now_pos()) == 1) {
            cmd.x = pointx + (random_int(20) * rand2());
            cmd.y = pointy + (random_int(20) * rand2());
            ptpmove(executedCmdIndex, queuedCmdIndex, cmd);
            cap.read(frame);
            frame = cv::Mat(frame, cv::Rect(80, 0, 480, 480));
            binarize(frame, &vis, valhmin, valsmin, valvmin, valhmax, valsmax, valvmax);
            param = vis.centroidsres.ptr<double>(1);
            x = static_cast<int>(param[0]);
            y = static_cast<int>(param[1]);
            age.setposition((((x)-(x % MAXX)) / (480 / MAXX)), (y - (y % MAXY)) / (480 / MAXY));
            cv::imshow("canny", vis.mask);
        }
    }
}

//trackbarコールバック関数（Rの変更）
void changehmin(int position, void* userdata)
{
    valhmin = position; //トラックバーの位置によってRの値を更新
}
//trackbarコールバック関数（Gの変更）
void changesmin(int position, void* userdata)
{
    valsmin = position; //トラックバーの位置によってGの値を更新
}
// trackbarコールバック関数（Bの変更）
void changevmin(int position, void* userdata)
{
    valvmin = position; //トラックバーの位置によってBの値を更新
}

void changehmax(int position, void* userdata)
{
    valhmax = position; //トラックバーの位置によってRの値を更新
}
//trackbarコールバック関数（Gの変更）
void changesmax(int position, void* userdata)
{
    valsmax = position; //トラックバーの位置によってGの値を更新
}
//trackbarコールバック関数（Bの変更）
void changevmax(int position, void* userdata)
{
    valvmax = position; //トラックバーの位置によってBの値を更新
}