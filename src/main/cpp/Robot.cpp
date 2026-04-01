// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "niEmbCan.h"

#include "FRC_FPGA_ChipObject/RoboRIO_FRC_ChipObject_Aliases.h"
#include "FRC_FPGA_ChipObject/nRoboRIO_FPGANamespace/nInterfaceGlobals.h"
#include "FRC_FPGA_ChipObject/nRoboRIO_FPGANamespace/tDIO.h"
#include "FRC_FPGA_ChipObject/tSystem.h"

#include <memory>
#include <signal.h>
#include <time.h>

// FRC standard CAN device ID fields:
//   Bits 28-24: Device type  (5 bits) – sensor = 4
//   Bits 23-16: Manufacturer (8 bits) – team use = 8
//   Bits 15-10: API class    (6 bits)
//   Bits  9- 6: API index    (4 bits)
//   Bits  5- 0: Device number(6 bits)
static const uint32_t kDeviceType   = 4;  // Sensor
static const uint32_t kManufacturer = 8;  // Team Use
static const uint32_t kApiClass     = 0;
static const uint32_t kApiIndex     = 0;
static const uint32_t kDeviceNumber = 0;

static const uint32_t kCanId =
    (kDeviceType << 24) | (kManufacturer << 16) |
    (kApiClass << 10) | (kApiIndex << 6) | kDeviceNumber;

// 20 ms loop period in nanoseconds
static const long kLoopPeriodNs = 20 * 1000 * 1000L;

static volatile sig_atomic_t g_running = 1;

static void signalHandler(int) {
  g_running = 0;
}

int main() {
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  int32_t status = 0;

  // Set target class for roboRIO before accessing the FPGA
  nFPGA::nRoboRIO_FPGANamespace::g_currentTargetClass = 0x40;

  // Open ChipObject session to the FRC FPGA image
  nFPGA::tSystem fpgaSystem(&status);
  if (status != 0) {
    return 1;
  }

  // Create DIO interface from the FPGA
  std::unique_ptr<nFPGA::nRoboRIO_FPGANamespace::tDIO> dio{
      nFPGA::nRoboRIO_FPGANamespace::tDIO::create(&status)};
  if (status != 0 || dio == nullptr) {
    return 1;
  }

  // Open CAN session at 1 Mbit/s
  tNiEmbCANHandle canHandle;
  status = niEmbCANOpenSession(0, 1000000, 32, &canHandle);
  if (status != 0) {
    return 1;
  }

  status = niEmbCANStart(canHandle);
  if (status != 0) {
    niEmbCANCloseSession(canHandle);
    return 1;
  }

  // Read DIO from the FPGA and send over CAN every 20 ms.
  // Use TIMER_ABSTIME so processing time does not accumulate into the period.
  struct timespec nextWake;
  clock_gettime(CLOCK_MONOTONIC, &nextWake);

  while (g_running) {
    // Advance wake time by one period
    nextWake.tv_nsec += kLoopPeriodNs;
    if (nextWake.tv_nsec >= 1000000000L) {
      nextWake.tv_nsec -= 1000000000L;
      nextWake.tv_sec++;
    }

    // Read the digital input register from the FPGA
    status = 0;
    nFPGA::nRoboRIO_FPGANamespace::tDIO::tDI di = dio->readDI(&status);

    if (status == 0) {
      tNiEmbCANFrame frame;
      frame.m_identifier = kCanId;
      frame.m_isExtended = 1;
      frame.m_isRemote   = 0;
      frame.m_dataLength = 4;
      frame.m_payload[0] = static_cast<uint8_t>((di.value >> 0) & 0xFF);
      frame.m_payload[1] = static_cast<uint8_t>((di.value >> 8) & 0xFF);
      frame.m_payload[2] = static_cast<uint8_t>((di.value >> 16) & 0xFF);
      frame.m_payload[3] = static_cast<uint8_t>((di.value >> 24) & 0xFF);

      niEmbCANWrite(canHandle, &frame);
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextWake, nullptr);
  }

  niEmbCANStop(canHandle);
  niEmbCANCloseSession(canHandle);
  return 0;
}
