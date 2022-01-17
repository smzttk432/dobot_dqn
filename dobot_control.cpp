// dobottest.cpp : このファイルには 'main' 関数が含まれています。プログラム実行の開始と終了がそこで行われます。
//
#pragma comment(lib, "DobotDll.lib")
#include <stdio.h>
#include "DobotDll.h"
#include "dobotcontrol.h"

int dobotconnect(bool m_bConnectStatus, bool connection) {
    if (!m_bConnectStatus) {
        if (ConnectDobot("", 115200, 0, 0, 0) != DobotConnect_NoError) {
            printf("Cannot connect Dobot!\n");
            connection = false;
        }
        else {
            printf("Connected!!\n");
            connection = true;
        }
    }
    return connection;
}
void ptpmove(uint64_t executedCmdIndex, uint64_t queuedCmdIndex, PTPCmd cmd) {
    SetQueuedCmdStartExec();
    SetPTPCmd(&cmd, true, &queuedCmdIndex);
    while (executedCmdIndex < queuedCmdIndex) {
        GetQueuedCmdCurrentIndex(&executedCmdIndex);
    }
    SetQueuedCmdStopExec();
}

void dobotdisconnect(bool connection) {
    if (connection == true) {
        DisconnectDobot();
        printf("Disconnected!!\n");
    }
    else {
        printf("No Connect\n");
    }
}
