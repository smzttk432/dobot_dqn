#pragma once

#include "DobotDll.h"
int dobotconnect(bool m_bConnectStatus, bool connection);
void ptpmove(uint64_t executedCmdIndex, uint64_t queuedCmdIndex, PTPCmd cmd);
void dobotdisconnect(bool connection);