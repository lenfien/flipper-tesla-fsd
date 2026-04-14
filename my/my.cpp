#include <memory>
#include <SPI.h>
#include <string>
#include <iomanip>
#include <sstream>
#include <mcp2515.h>

bool enablePrint = true;

#define LED_PIN PIN_LED                // onboard red LED (GPIO13)
#define CAN_CS PIN_CAN_CS              // GPIO19 on this Feather
#define CAN_INT_PIN PIN_CAN_INTERRUPT  // GPIO22 (unused here for now)
#define CAN_STBY PIN_CAN_STANDBY       // GPIO16
#define CAN_RESET PIN_CAN_RESET        // GPIO18

std::unique_ptr<MCP2515> mcp;

inline uint8_t readMuxID(const can_frame &frame) {
    return frame.data[0] & 0x07;
}

inline bool isFSDSelectedInUI(const can_frame &frame) {
    return true;
    //return (frame.data[4] >> 6) & 0x01;
}

inline void setSpeedProfileV12V13(can_frame &frame, int profile) {
    frame.data[6] &= ~0x06;
    frame.data[6] |= (profile << 1);
}

// ===================================================================
// setBit helper: flips a specific bit inside the 8-byte CAN payload
// ===================================================================
inline void setBit(can_frame &frame, int bit, bool value) {
    int byteIndex = bit / 8;
    int bitIndex = bit % 8;
    uint8_t mask = static_cast<uint8_t>(1U << bitIndex);
    if (value) {
        frame.data[byteIndex] |= mask;
    } else {
        frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
    }
}

struct FSDHandler {
    void handleMessage(can_frame &frame) {
        // 当前挡路限速
        if (frame.can_id == 0x399) {
            m_speed_limit = (frame.data[1] & 0x1F) * 5;
            return;
        }

        // 其他配置项
        if (frame.can_id == 1016) {
            // 是否跟随导航驾驶已经开启
            // 用这个来决定是否开启FSD
            m_is_fsd_enabled = (frame.data[1] & 0b00100000) != 0;

            // 跟随距离设置 1 - 6
            m_follow_distance = (frame.data[5] & 0b11100000) >> 5;

            // 在导航驾驶里面选择的速度类型 0 - 2
            m_speed_rule_selected = m_speed_profile = ((frame.data[6] & 0b00001100) >> 2);

            // 如果在界面上选择0，那就用HW4的代码
            m_use_hw4_code = m_speed_rule_selected == 0;

            //
            if (!m_use_hw4_code) {
                switch (m_follow_distance) {
                    case 1:
                    case 2:
                    case 3:
                        m_speed_profile = 2;
                        break;
                    case 4:
                    case 5:
                        m_speed_profile = 1;
                        break;
                    case 6:
                        m_speed_profile = 0;
                        break;
                    default:
                        m_speed_profile = 0;
                        break;
                }
            } else {
                switch (m_follow_distance) {
                    case 1:
                        m_speed_profile = 3;
                        break;
                    case 2:
                        m_speed_profile = 2;
                        break;
                    case 3:
                        m_speed_profile = 1;
                        break;
                    case 4:
                        m_speed_profile = 0;
                        break;
                    case 5:
                        m_speed_profile = 4;
                        break;
                    default:
                        m_speed_profile = 4;
                        break;
                }
            }

            return;
        }

        if (!m_is_fsd_enabled)
            return;

        if (frame.can_id == 1021) {
            auto index = readMuxID(frame);

            // index 0: Main FSD control message
            if (index == 0) {
                // 计算限速
                if (!m_use_hw4_code) {
                    m_speed_offset_raw = frame.data[3];

                    // 偏移模式是固定速度
                    if ((0b10000000 & m_speed_offset_raw) == 0) {
                        m_speed_offset_v2 = 0x7F & m_speed_offset_raw; // 10 - 60 - 120
                        m_speed_offset_v2 = Rerange(Clamp(m_speed_offset_v2, 60, 80), 60, 80, 0, 120);
                    }
                    // 偏移模式是百分比
                    else {
                        m_speed_offset_v2 = 0x7F & m_speed_offset_raw; // 10 - 60 - 120
                        m_speed_offset_v2 = Rerange(Clamp(m_speed_offset_v2, 60, 120), 60, 120, 0, 120);
                    }

                    // 如果速度在UI上选择为0，表示希望使用自动限速
                    if (m_speed_offset_v2 == 0 && m_speed_limit > 0) {
                        if (m_speed_limit <= 30)
                            m_target_speed = 30;
                        else if (m_speed_limit < 40)
                            m_target_speed = 40;
                        else if (m_speed_limit < 50)
                            m_target_speed = 50;
                        else if (m_speed_limit <= 60)
                            m_target_speed = 70;
                        else if (m_speed_limit < 80)
                            m_target_speed = 80;
                        else if (m_speed_limit == 80)
                            m_target_speed = 90;
                        else if (m_speed_limit <= 90)
                            m_target_speed = 100;
                        else if (m_speed_limit <= 100)
                            m_target_speed = 110;
                        else if (m_speed_limit < 120)
                            m_target_speed = 120;
                        else
                            m_target_speed = 130;

                        m_speed_offset_v2 = calcAutoCANOffset(m_target_speed);
                    }

                    m_speed_offset = m_speed_offset_v2;
                }

                // 打开FSD
                setBit(frame, 46, true); // FSD enable / activation bit
                if (m_use_hw4_code)
                    setBit(frame, 60, true); // Additional HW4-specific FSD bit

                // 打开紧急车辆检测
                setBit(frame, 59, true);

                // set profile
                if (!m_use_hw4_code) {
                    frame.data[6] &= ~0x06;
                    frame.data[6] |= (m_speed_profile << 1);
                }

                // FSD 停车/停点控制相关开关
                setBit(frame, 38, true);

                // HOV 相关开关，通常可理解为多人乘员车道/拼车道策略
                setBit(frame, 3, true);

                // FSD 可视化显示开关
                setBit(frame, 37, true);

                // m_frame_to_debug[index] = frame;
                mcp->sendMessage(&frame);
            }

            // index 1: Nag suppression message (HW3)
            if (index == 1) {
                setBit(frame, 19, false); // ← NAG SUPPRESSION Clears the hands-on-wheel nag / attention bit

                if (m_use_hw4_code)
                    setBit(frame, 47, true); // Extra bit set only on HW4

                // 禁用驾驶室内摄像头
                setBit(frame, 43, false);

                // 显示车到图
                setBit(frame, 45, true);

                // 轨道和目标标签
                setBit(frame, 46, true);

                m_frame_to_debug[index] = frame;
                mcp->sendMessage(&frame);
            }

            // index 2: Speed offset injection
            if (index == 2) {
                // HW3.SpeedOffset
                if (!m_use_hw4_code) {
                    frame.data[0] &= ~(0b11000000);
                    frame.data[1] &= ~(0b00111111);
                    frame.data[0] |= (m_speed_offset & 0x03) << 6;
                    frame.data[1] |= (m_speed_offset >> 2);
                }

                // HW4 Set profile
                if (m_use_hw4_code) {
                    frame.data[7] &= ~(0x07 << 4);
                    frame.data[7] |= (m_speed_profile & 0x07) << 4;
                }

                m_frame_to_debug[index] = frame;
                mcp->sendMessage(&frame);
            }

            if (index == 0 && enablePrint) {
                Serial.printf(
                    "FSDHandler: FSDEnable: %d(HW4Code:%s), Follow:%d,Profile:%d,Offset: %d, SpeedLimit:%d,TargetSpeed: %d, Raw: 0:%s 1:%s 2:%s, index: %d\n",
                    m_is_fsd_enabled,
                    m_use_hw4_code ? "Yes" : "No",
                    m_follow_distance,
                    m_speed_profile,
                    m_speed_offset,
                    m_speed_limit,
                    m_target_speed,
                    ToString(m_frame_to_debug[0]).c_str(),
                    ToString(m_frame_to_debug[1]).c_str(),
                    ToString(m_frame_to_debug[2]).c_str(),
                    m_index_to_use);
            }
        }

#define CAN_ID_EPAS_STATUS    0x370  // 880 - EPAS3P_sysStatus (nag killer target)
        if (frame.can_id == CAN_ID_EPAS_STATUS) {
            if (frame.can_dlc < 8) return;

            // only act when handsOnLevel == 0 (no hands detected)
            uint8_t hands_on = (frame.data[4] >> 6) & 0x03;
            if(hands_on != 0) return;
            //Serial.printf("1");

            m_frame_to_debug[0] = frame;

            // 0:11 f5  7 bd 20 1b 2e a6
            // 0:11 f5  7 bd 20 1b 2e a6
            // 0:11 ff  7 b9 1f 47 23 cc
            // 0:11 ff  7 b9 1f 47 23 cc
            // 0:12  b  8 55 20  a 23 3a
            // 0:12  b  8 55 20  a 23 3a
            //
            // if (m_index_use_counter ++ >= 1000) {
            //     m_index_to_use += 1;
            //     m_index_use_counter = 0;
            // }
            //
            // for (int i = 0; i < 8; ++i)
            //     frame.data[i] = m_to_use_data[m_index_to_use % 3][i];
            //
            // m_frame_to_debug[0] = frame;
            // mcp->sendMessage(&frame);
        }
    }

    // 工具函数
private:
    std::string ToBinaryStr(uint8_t i) {
        std::string b;
        b.reserve(8);
        for (int index = 0; index < sizeof(i) * 8; index += 1) {
            b += ((i << index) & 0b10000000) ? "1" : "0";
        }
        return b;
    }

    int Clamp(int value, int min, int max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    int Rerange(int value, int min, int max, int t_min, int t_max) {
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
        return Rerange(Clamp(rawPercent, 0, 60), 0, 60, 0, 240);
    }

    int calcAutoCANOffset(int targetSpeed) {
        if (m_speed_limit == 0)
            return 0;
        return calcPercent(targetSpeed, m_speed_limit);
    }

    std::string
    ToString(const can_frame &frame) {
        std::ostringstream result;
        for (size_t i = 0; i < sizeof(frame.data); ++i)
            result << "0x" << std::hex << (int) frame.data[i] << ",";
        return result.str();
    }

private:
    uint m_speed_offset = 0;
    uint8_t m_speed_offset_raw = 0;
    int m_speed_offset_v2 = 0b0111100;

    uint32_t m_follow_distance = 0;
    uint32_t m_speed_rule_selected = 0;

    bool m_is_fsd_enabled = false;
    int m_speed_limit = 0;
    int m_target_speed = 0;

    int m_speed_profile = 1;

    bool m_use_hw4_code = false;

    can_frame m_frame_to_debug[3];

    uint8_t m_to_use_data[5][8] = {
        // 0:11 f5  7 bd 20 1b 2e a6
        // 0:11 ff  7 b9 1f 47 23 cc
        // 0:12  b  8 55 20  a 23 3a

        {0x11,0xf6,0x7,0xbe,0x1e,0xae,0x27,0x32 },
        {0x11,0xf3,0x7,0xb0,0x20,0x37,0x24,0xa9},
        {0x11, 0xf5, 0x7, 0xbd, 0x20, 0x1b, 0x2e, 0xa6},
        {0x11, 0xff, 0x7, 0xb9, 0x1f, 0x47, 0x23, 0xcc},
           {0x12, 0x0b, 0x8, 0x55, 0x20, 0xa,  0x23, 0x3a},
    };

    uint32_t m_index_to_use = 0;
    uint32_t m_index_use_counter = 0;
};

std::unique_ptr<FSDHandler> handler;

void setup() {
    handler = std::make_unique<FSDHandler>();
    delay(1500);
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 1000) {
    }

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
    handler->handleMessage(frame); // Fixed: handelMessage → handleMessage
}
