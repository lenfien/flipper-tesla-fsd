//
// Created by HongBo Zhang on 2026/4/13.
///*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program. If not, see <https://www.gnu.org/licenses/>.
*/

#include <memory>
#include <SPI.h>
#include <string>
#include <iomanip>
#include <sstream>
#include <mcp2515.h>

#define LEGACY LegacyHandler
#define HW3 HW3Handler
#define HW4 HW4Handler
#define HW HW3  // Change to HW4 or LEGACY depending on your vehicle

bool enablePrint = true;

#define LED_PIN PIN_LED                // onboard red LED (GPIO13)
#define CAN_CS PIN_CAN_CS              // GPIO19 on this Feather
#define CAN_INT_PIN PIN_CAN_INTERRUPT  // GPIO22 (unused here for now)
#define CAN_STBY PIN_CAN_STANDBY       // GPIO16
#define CAN_RESET PIN_CAN_RESET        // GPIO18

std::unique_ptr<MCP2515> mcp;

struct CarManagerBase {
  int speedProfile = 1;
  bool FSDEnabled = true;
  virtual void handleMessage(can_frame& frame);  // Fixed: handelMessage → handleMessage
};

inline uint8_t readMuxID(const can_frame& frame) {
  return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame& frame) {
  return true;
  //return (frame.data[4] >> 6) & 0x01;
}

inline void setSpeedProfileV12V13(can_frame& frame, int profile) {
  frame.data[6] &= ~0x06;
  frame.data[6] |= (profile << 1);
}

// ===================================================================
// setBit helper: flips a specific bit inside the 8-byte CAN payload
// ===================================================================
inline void setBit(can_frame& frame, int bit, bool value) {
  int byteIndex = bit / 8;
  int bitIndex = bit % 8;
  uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
  if (value) {
    frame.data[byteIndex] |= mask;
  } else {
    frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
  }
}

struct LegacyHandler : public CarManagerBase {
  virtual void handleMessage(can_frame& frame) override {  // Fixed
    if (frame.can_id == 1006) {
      auto index = readMuxID(frame);

      // index 0: Main FSD status / speed profile message
      if (index == 0) {
        // auto off = (uint8_t)((frame.data[3] >> 1) & 0x3F) - 30;
        // switch (off) {
        //   case 2: speedProfile = 2; break;
        //   case 1: speedProfile = 1; break;
        //   case 0: speedProfile = 0; break;
        //   default: break;
        // }

        setBit(frame, 46, true);  // FSD enable / activation bit
        setSpeedProfileV12V13(frame, speedProfile);
        mcp->sendMessage(&frame);
      }

      // index 1: Nag suppression message
      if (index == 1) {
        setBit(frame, 19, false);  // ← NAG SUPPRESSION
                                   // Clears the hands-on-wheel nag / attention bit
        mcp->sendMessage(&frame);
      }

      if (index == 0 && enablePrint) {
        Serial.printf("LegacyHandler: FSD: %d, Profile: %d\n", FSDEnabled, speedProfile);
      }
    }
  }
};

std::string ToBinaryStr(uint8_t i) {
  std::string b;
  b.reserve(8);
  for (int index = 0; index < sizeof(i) * 8; index += 1) {
    b += ((i << index) & 0b10000000) ? "1" : "0";
  }
  return b;
}


int clamp(int value, int min, int max) {
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

int rerange(int value, int min, int max, int t_min, int t_max) {
  int t_len = t_max - t_min;
  int len = max - min;
  int p = (value - min) * 100 / len;
  return t_min + p * t_len / 100;
}

// 根据目标速度和当前限速，计算出应该把车机设置为多少的限速百分比
int calcPercent(int targetSpeed, int speedLimit) {
  if (targetSpeed <= speedLimit)
    return 0;

  int rawPercent = (targetSpeed - speedLimit) * 100 / speedLimit;
  return rerange(clamp(rawPercent, 0, 60), 0, 60, 0, 240);
}


struct HW3Handler : public CarManagerBase {
  uint speedOffset = 0;
  uint8_t speedOffsetRaw = 0;
  int speedOffsetV2 = 0b0111100;
  uint8_t followDistance = 0;

  bool IsFSDEnabled = false;
  int8_t FSDSpeed = 0;

  int speedLimit = 0;
  int targetSpeed = 0;

  std::string CanStrVec[10];
  std::string CanStrVecM[10];

  int calcAutoCANOffset(int targetSpeed) {
    if (speedLimit == 0)
      return 0;
    return calcPercent(targetSpeed, speedLimit);
  }

  std::string
  ToString(uint8_t* ptr, size_t size) {
    std::ostringstream result;

    for (size_t i = 0; i < size; ++i) {
      result << std::hex << std::setw(2) << (int)ptr[i] << " ";
    }

    return result.str();
  }

  virtual void handleMessage(can_frame& frame) override {  // Fixed

    if (frame.can_id == 0x399) {
      speedLimit = (frame.data[1] & 0x1F) * 5;
    }

    if (frame.can_id == 1016) {

      std::string can_local = ToString(frame.data, 8);
      CanStrVecM[3] = (can_local == CanStrVec[3]) ? can_local : can_local + "(M)";
      CanStrVec[3] = can_local;

      IsFSDEnabled = (frame.data[1] & 0b00100000) != 0;
      FSDSpeed = ((frame.data[6] & 0b00001100) >> 2) - 1;
      if (FSDSpeed < 0)
        FSDSpeed = 0;

      speedProfile = FSDSpeed;

      // followDistance = (frame.data[5] & 0b11100000) >> 5;
      // switch (followDistance) {
      //   case 1:
      //   case 2:
      //   case 3:
      //     speedProfile = 2;
      //     break;
      //   case 4:
      //   case 5:
      //     speedProfile = 1;
      //     break;
      //   case 6:
      //     speedProfile = 0;
      //     break;
      //   default:
      //     break;
      // }
      return;
    }

    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);

      // index 0: Main FSD control message
      if (index == 0) {
        std::string can_local = ToString(frame.data, 8);
        CanStrVecM[index] = can_local == CanStrVec[index] ? can_local : can_local + "(M)";
        CanStrVec[index] = can_local;

        // Serial.printf("0: %s", ToString(frame.data, 8).c_str());

        if (IsFSDEnabled) {
          speedOffsetRaw = frame.data[3];
          speedOffset = std::max(std::min(((uint8_t)((frame.data[3] >> 1) & 0x3F) - 30) * 5, 100), 0);

          // 偏移模式是固定速度
          if ((0b10000000 & speedOffsetRaw) == 0) {
            speedOffsetV2 = 0x7F & speedOffsetRaw;  // 10 - 60 - 120
            speedOffsetV2 = rerange(clamp(speedOffsetV2, 60, 80), 60, 80, 0, 120);
          }
          // 偏移模式是百分比
          else {
            speedOffsetV2 = 0x7F & speedOffsetRaw;  // 10 - 60 - 120
            speedOffsetV2 = rerange(clamp(speedOffsetV2, 60, 120), 60, 120, 0, 120);
          }

          // 如果速度在UI上选择为0，表示希望使用自动限速
          if (speedOffsetV2 == 0 && speedLimit > 0) {
            if (speedLimit <= 30)
              targetSpeed = 30;
            else if (speedLimit < 40)
              targetSpeed = 40;
            else if (speedLimit < 50)
              targetSpeed = 50;
            else if (speedLimit <= 60)
              targetSpeed = 70;
            else if (speedLimit < 80)
              targetSpeed = 80;
            else if (speedLimit == 80)
              targetSpeed = 90;
            else if (speedLimit <= 90)
              targetSpeed = 100;
            else if (speedLimit <= 100)
              targetSpeed = 120;
            else if (speedLimit < 120)
              targetSpeed = 120;
            else
              targetSpeed = 130;

            speedOffsetV2 = calcAutoCANOffset(targetSpeed);
          }

          speedOffset = speedOffsetV2;

          //
          setBit(frame, 46, true);  // FSD enable / activation bit
          setBit(frame, 60, true);  // Additional HW4-specific FSD bit

          // set profile
          frame.data[6] &= ~0x06;
          frame.data[6] |= (speedProfile << 1);

          mcp->sendMessage(&frame);
        }
      }

      // index 1: Nag suppression message (HW3)
      if (index == 1) {

        if (IsFSDEnabled) {
          setBit(frame, 19, false);  // ← NAG SUPPRESSION // Clears the hands-on-wheel nag / attention bit
          setBit(frame, 47, true);   // Extra bit set only on HW4

          mcp->sendMessage(&frame);
        }
      }

      // index 2: Speed offset injection
      if (index == 2) {

        //  Serial.printf("2: %s\n", ToString(frame.data, 8).c_str());

        // std::string can_local = ToString(frame.data, 8);
        // CanStrVecM[index] = can_local == CanStrVec[index] ? can_local : can_local + "(M)";
        // CanStrVec[index] = can_local;

        if (IsFSDEnabled) {
          frame.data[0] &= ~(0b11000000);
          frame.data[1] &= ~(0b00111111);

          frame.data[0] |= (speedOffset & 0x03) << 6;
          frame.data[1] |= (speedOffset >> 2);
          mcp->sendMessage(&frame);
        }
      }

      if (index == 0 && enablePrint) {
        Serial.printf("HW3Handler: FSDEnable: %d, Follow: %d,  Profile: %d,  Offset: %d, OffsetV2Raw:%d, OffsetV2: %d, SpeedLimit: %d, targetSpeed: %d, reserve: %d, %d, %d\n",
                      IsFSDEnabled,
                      followDistance,
                      speedProfile,
                      speedOffset,
                      // ToBinaryStr(speedOffsetRaw).c_str(),
                      speedOffsetRaw & 0x7F,
                      speedOffsetV2,
                      speedLimit,
                      targetSpeed,
                      frame.data[4] & 0x3F,
                      (frame.data[3] >> 6) & 0x01,
                      (frame.data[3] >> 4) & 0x0F);

        // Serial.printf("0:%s\t 1:%s\t2:%s\t3:%s\n",
        //               CanStrVecM[0].c_str(),
        //               CanStrVecM[1].c_str(),
        //               CanStrVecM[2].c_str(),
        //               CanStrVecM[3].c_str());
      }
    }
  }
};

struct HW4Handler : public CarManagerBase {
  virtual void handleMessage(can_frame& frame) override {  // Fixed
    if (frame.can_id == 1016) {
      auto fd = (frame.data[5] & 0b11100000) >> 5;
      switch (fd) {
        case 1: speedProfile = 3; break;
        case 2: speedProfile = 2; break;
        case 3: speedProfile = 1; break;
        case 4: speedProfile = 0; break;
        case 5: speedProfile = 4; break;
      }
    }

    if (frame.can_id == 1021) {
      auto index = readMuxID(frame);
      auto FSDEnabled = isFSDSelectedInUI(frame);

      // index 0: Main FSD control message (HW4)
      if (index == 0 && true) {
        setBit(frame, 46, true);  // FSD enable / activation bit
        setBit(frame, 60, true);  // Additional HW4-specific FSD bit
        mcp->sendMessage(&frame);
      }

      // index 1: Nag suppression message (HW4)
      if (index == 1) {
        setBit(frame, 19, false);  // ← NAG SUPPRESSION Clears the hands-on-wheel nag / attention bit
        setBit(frame, 47, true);   // Extra bit set only on HW4
        mcp->sendMessage(&frame);
      }

      // index 2: Speed profile injection for HW4
      if (index == 2) {
        frame.data[7] &= ~(0x07 << 4);
        frame.data[7] |= (speedProfile & 0x07) << 4;
        mcp->sendMessage(&frame);
      }

      if (index == 0 && enablePrint) {
        //Serial.printf("HW4Handler: profile: %d\n", speedProfile);
      }
    }
  }
};

std::unique_ptr<CarManagerBase> handler;

void setup() {
  handler = std::make_unique<HW>();
  delay(1500);
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1000) {}

  mcp = std::make_unique<MCP2515>(CAN_CS);
  mcp->reset();
  MCP2515::ERROR e = mcp->setBitrate(CAN_500KBPS, MCP_16MHZ);
  if (e != MCP2515::ERROR_OK) Serial.println("setBitrate failed");
  mcp->setNormalMode();
  Serial.println("MCP25625 ready @ 500k");
}

__attribute__((optimize("O3"))) void loop() {
  can_frame frame;
  int r = mcp->readMessage(&frame);
  if (r != MCP2515::ERROR_OK) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  digitalWrite(LED_PIN, LOW);
  handler->handleMessage(frame);  // Fixed: handelMessage → handleMessage
}