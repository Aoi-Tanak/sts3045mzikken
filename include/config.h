#pragma once
#include <stdint.h>

// Core2 PORT C: yellow=TX GPIO14, white=RX GPIO13.
constexpr int kServoTxPin = 14;
constexpr int kServoRxPin = 13;
constexpr uint8_t kServoId = 1;
constexpr uint32_t kServoBaud = 1000000;
constexpr int kServoZeroPosition = 2048;
constexpr int kServoDirection = 1;
constexpr int kServoPositionMax = 4095;
constexpr float kDegreesPerStep = 360.0f / 4096.0f;
constexpr int kStepHeightCounts = 4;
// Apply target +5 / -3 according to the measured direction of travel.
constexpr int kPositiveOffsetCounts = 5;
constexpr int kNegativeOffsetCounts = 3;
constexpr int kCommandLimitCounts = 40;
constexpr uint16_t kServoSpeed = 0;
constexpr uint8_t kServoAcceleration = 0;
// Same echoing TTL interface as the reference project.
constexpr bool kExpectTxEcho = true;

// Core2 PORT A: yellow=SDA GPIO32, white=SCL GPIO33.
constexpr int kTofSdaPin = 32;
constexpr int kTofSclPin = 33;
constexpr uint8_t kTofAddress = 0x29;
constexpr uint32_t kTofStaleMs = 200;
constexpr uint32_t kServoFeedbackUpdateMs = 100;
constexpr uint32_t kDisplayUpdateMs = 100;
static_assert(kServoDirection == 1 || kServoDirection == -1, "Invalid direction");
static_assert(kStepHeightCounts + kPositiveOffsetCounts <= kCommandLimitCounts &&
              kNegativeOffsetCounts <= kCommandLimitCounts,
              "Directional offset exceeds command limits");
static_assert(kServoZeroPosition - kCommandLimitCounts >= 0 &&
              kServoZeroPosition + kCommandLimitCounts <= kServoPositionMax,
              "Invalid command limits");
