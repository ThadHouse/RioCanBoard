// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "niEmbCan.h"

#include "FRC_FPGA_ChipObject/tSystem.h"

int main() {
  tNiEmbCANHandle canHandle;

  int32_t status = 0;

    status = niEmbCANOpenSession(0, 1000000, 32, &canHandle);

    if (status != 0) {
      return 1;
    }

    status = niEmbCANStart(canHandle);

    if (status != 0) {
      niEmbCANCloseSession(canHandle);
      return 1;
    }

    niEmbCANStop(canHandle);
    niEmbCANCloseSession(canHandle);
}
