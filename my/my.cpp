#include <memory>
#include <SPI.h>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <mcp2515.h>

bool enablePrint = true;

#define LED_PIN PIN_LED                // onboard red LED (GPIO13)
#define CAN_CS PIN_CAN_CS              // GPIO19 on this Feather
#define CAN_INT_PIN PIN_CAN_INTERRUPT  // GPIO22 (unused here for now)
#define CAN_STBY PIN_CAN_STANDBY       // GPIO16
#define CAN_RESET PIN_CAN_RESET        // GPIO18

enum EGear {
    EGear_D = 0b100,
    EGear_P = 0b001,
    EGear_R = 0b010,
    EGear_N = 0b011,
};

std::unique_ptr<MCP2515> mcp;

static uint32_t nag_prng_state = 0xDEADBEEF;
static uint32_t nag_xorshift32(void) {
    uint32_t x = nag_prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    nag_prng_state = x;
    return x;
}

struct FSDHandler {
    FSDHandler() {
        m_frame_to_debug_vec_for_0x3DF.resize(10);
        m_frame_to_debug_vec_for_0x3F8.resize(5);
        m_frame_to_debug_vec_for_0x370.resize(5);
    }

    __attribute__((optimize("O3"))) void
    Handle_0x399(can_frame &frame) {
        m_speed_limit = (frame.data[1] & 0x1F) * 5;
    }

    __attribute__((optimize("O3"))) void
    Handle_787(can_frame& frame) {
        frame.data[0] &= static_cast<uint8_t>(~0x03);
        frame.data[0] |= static_cast<uint8_t>(0x01);
        frame.data[7] = ComputeVehicleChecksum(frame);
        mcp->sendMessage(&frame);
    }

    __attribute__((optimize("O3")))  void
    Handle_0x3F8(can_frame& frame) {
        // 跟随距离设置 1 - 6
        m_follow_distance = (frame.data[5] & 0b11100000) >> 5;

        // 在导航驾驶里面选择的速度类型 0 - 2
        m_speed_rule_selected = ((frame.data[6] & 0b00001100) >> 2);

        // m_is_fsd_enabled = (frame.data[1] & 0b00100000) != 0;
        m_is_fsd_enabled = m_speed_rule_selected > 0 && (m_cur_gear == EGear_D || m_cur_gear == EGear_R);

        m_need_ensure_when_change_line = ((frame.data[0] & 0b00000010) != 0);
        m_shichuchaochedao_bit = ((frame.data[6] & 0b01000000) == 0);

        // 是否打开内相机，使用变道确认
        m_enable_camera = m_shichuchaochedao_bit;

        // 如果不是使用HW4的代码，那么就使用HW3的代码
        // speed profile
        {
            switch (m_speed_rule_selected) {
                case 0:
                    m_speed_profile_for_hw3 = 0;
                    break;
                default:
                    m_speed_profile_for_hw3 = m_speed_rule_selected - 1;
                    break;
            }

            m_speed_profile_for_hw3 = Clamp(m_speed_rule_selected - 1, 0, 2);

            switch (m_follow_distance) {
                case 1:
                    m_speed_profile_for_hw4 = 3; break;
                case 2:
                    m_speed_profile_for_hw4 = 2; break;
                case 3:
                    m_speed_profile_for_hw4 = 1; break;
                case 4:
                    m_speed_profile_for_hw4 = 0; break;
                case 5:
                    m_speed_profile_for_hw4 = 4; break;
                default:
                    m_speed_profile_for_hw4 = 4; break;
            }
        }

        if (m_enable_debug)
            m_frame_to_debug_vec_for_0x3F8[0] = frame;
    }

    __attribute__((optimize("O3"))) void
    Handle_0x3FD_Mux0(can_frame & frame) {
        // 计算限速
        // 速度偏移的原始值
        // 选择固定速度的，就代表用auto mode
        m_speed_offset_raw = frame.data[3];
        m_use_speed_offset_auto = (0b10000000 & m_speed_offset_raw) == 0;
        m_speed_offset_raw &= 0b01111111;

        // 打开FSD
        SetBit(frame, 46, true); // FSD enable / activation bit

        // 打开HW4-specific FSD bit
        if (m_use_hw4_code)
            SetBit(frame, 60, true); // Additional HW4-specific FSD bit

        // 打开紧急车辆检测
        if (m_use_hw4_code)
            SetBit(frame, 59, true);

        // HW3 的 Profile
        if (!m_use_hw4_code || m_use_hw3_profile_when_hw4) {
            frame.data[6] &= ~0x06;
            frame.data[6] |= (m_speed_profile_for_hw4 << 1);
        }

        // FSD 停车/停点控制相关开关
        SetBit(frame, 38, true);
        //
        // // HOV 相关开关，通常可理解为多人乘员车道/拼车道策略
        // SetBit(frame, 3, true);
        //
        // // FSD 可视化显示开关
        // SetBit(frame, 37, true);

        // 发送
        mcp->sendMessage(&frame);
    }

    __attribute__((optimize("O3"))) void
    Handle_0x3FD_Mux1(can_frame & frame) {
        m_biandao_tixing_zhendong = frame.data[4] & 0b00000001;
        m_biandao_tixing_fengming = frame.data[4] & 0b00100000;

        if (m_biandao_tixing_fengming && m_biandao_tixing_zhendong) {
            m_use_hw4_code = true;
            m_use_hw3_profile_when_hw4 = true;
        }
        else if (m_biandao_tixing_zhendong) {
            m_use_hw4_code = true;
            m_use_hw3_profile_when_hw4 = false;
        }
        else
            m_use_hw4_code = false;

        // 禁用Nag
        SetBit(frame, 19, false);

        if (m_use_hw4_code)
            SetBit(frame, 47, true); // Extra bit set only on HW4

        // enable EAP/summon
        SetBit(frame, 46, true);

        // 禁用驾驶室内摄像头
        SetBit(frame, 43, m_enable_camera);

        // 手握方向盘提醒
        // SetBit(frame, 17, true); //  ← 告知车机驾驶员在看路

        //
        // // 39
        // // UI_factorySummonEnable
        // SetBit(frame, 39, true);
        //
        // // 打开停止警告
        // // UI_enableAutopilotStopWarning
        // SetBit(frame, 44, true);

        //
        // 显示车到图
        SetBit(frame, 45, true);

        //
        // UI_enableMapStops 20
        SetBit(frame, 20, true);

        // 发送
        mcp->sendMessage(&frame);
    }

    __attribute__((optimize("O3"))) void
    Handle_0x3FD_Mux2(can_frame & frame) {

        // 偏移模式是固定速度
        // 偏移模式是百分比
        if (!m_use_speed_offset_auto)
            m_target_offset_percent = Rerange(Clamp(m_speed_offset_raw, 60, 120), 60, 120, 0, 30);
        else {
            if (m_speed_limit <= 40)
                m_target_offset_percent = 50;
            else if (m_speed_limit <= 60)
                m_target_offset_percent = 30;
            else if (m_speed_limit <= 80)
                m_target_offset_percent = 20;
            else if (m_speed_limit <= 100)
                m_target_offset_percent = 10;
            else
                m_target_offset_percent = 5;
        }

        m_speed_offset = m_use_hw4_code ? m_target_offset_percent : Rerange(m_target_offset_percent, 0, 60, 0, 240);

        // HW3 offset
        if (!m_use_hw4_code) {
            frame.data[0] &= ~(0b11000000);
            frame.data[1] &= ~(0b00111111);
            frame.data[0] |= (m_speed_offset & 0x03) << 6;
            frame.data[1] |= (m_speed_offset >> 2);
        }

        // HW4 profile
        if (m_use_hw4_code) {
            frame.data[7] &= ~(0x07 << 4);
            frame.data[7] |= (m_speed_profile_for_hw4 & 0x07) << 4;
        }

        // HW4 offset
        if (m_use_hw4_code)
            frame.data[1] = (frame.data[1] & 0xC0) | (m_speed_offset & 0x3F);

        // m_frame_to_debug[index] = frame;
        mcp->sendMessage(&frame);
    }

    __attribute__((optimize("O3"))) void
    Handle_880(can_frame & frame) {
        // return;
        // 0x3F8: 0:00010001;8:11101101;16:00000111;24:10100000;32:01100000;40:00001011;48:00101000;56:10101011;

        static int16_t nag_torq_walk = 2230;       // raw starting = 1.80 Nm
        static uint8_t nag_exc_frames = 0;         // frames in grip excursion
        static uint16_t nag_frames_until_exc = 175; // frames until next excursion

        if (m_enable_debug)
            m_frame_to_debug_vec_for_0x370[0] = frame;

        // only act when handsOnLevel == 0 (no hands detected)
        uint8_t hands_on = frame.data[4] >> 6 & 0x03;
        if(hands_on != 0) return;
        // if (m_das_hands_on_state == 0 || m_das_hands_on_state == 8) return;

        // --- Organic torque variation ---
        // torsionBarTorque encoding: tRaw = (Nm + 20.5) / 0.01
        // d[2] lower nibble = tRaw >> 8, d[3] = tRaw & 0xFF
        int16_t torq;
        if(nag_exc_frames > 0) {
            // Grip pulse: ~3.20 Nm ± small noise
            torq = 2350 + (int16_t)((nag_xorshift32() % 41) - 20);
            nag_exc_frames--;
        } else {
            // Normal random walk: step ±15 per frame
            int16_t step = (int16_t)((nag_xorshift32() % 31) - 15);
            nag_torq_walk += step;
            if(nag_torq_walk < 2150) nag_torq_walk = 2150; // min ~1.00 Nm
            if(nag_torq_walk > 2290) nag_torq_walk = 2290; // max ~2.40 Nm
            torq = nag_torq_walk;

            // Count down to next grip excursion
            if(nag_frames_until_exc > 0) {
                nag_frames_until_exc--;
            } else {
                nag_exc_frames = 3 + (nag_xorshift32() % 3); // 3-5 frames
                nag_frames_until_exc = 125 + (nag_xorshift32() % 100); // 5-9 sec
            }
        }

        can_frame echo{};
        memset(&echo, 0, sizeof(can_frame));

        echo.can_id = 880;
        echo.can_dlc = 8;

        echo.data[0] = frame.data[0];
        echo.data[1] = frame.data[1];
        echo.data[2] = (frame.data[2] & 0xF0) | (uint8_t)((torq >> 8) & 0x0F);
        echo.data[3] =  (uint8_t)(torq & 0xFF);
        echo.data[4] = frame.data[4] | 0x40;
        echo.data[5] = frame.data[5];

        // Counter + 1
        uint8_t cnt = (frame.data[6] & 0x0F);
        cnt = (cnt + 1) & 0x0F;
        echo.data[6] = (frame.data[6] & 0xF0) | cnt;

        // checksum: sum(byte0..6) + 0x70 + 0x03 (CAN ID 0x370 split)
        uint16_t sum = 0;
        for(int i = 0; i < 7; i++)
            sum += echo.data[i];
        sum += (880 & 0xFF) + (880 >> 8);
        echo.data[7] = (uint8_t)(sum & 0xFF);

        // 发送
        mcp->sendMessage(&echo);

        if (m_enable_debug)
            m_frame_to_debug_vec_for_0x370[1] = echo;
    }

    __attribute__((optimize("O3"))) void
    HandleMessage(can_frame &frame) {
        if (frame.can_dlc < 8) return;

        switch (frame.can_id) {
            case 0x399:
                Handle_0x399(frame);
                break;
            case 787:  // track
                //Handle_787(frame);
                break;
            case 0x3F8:
                Handle_0x3F8(frame);
                break;
            case 0x3FD: {
                if (!m_is_fsd_enabled) break;
                auto index = ReadMuxID(frame);

                if (m_enable_debug)
                    m_frame_to_debug_vec_for_0x3DF[index] = frame;

                switch (index) {
                    case 0:
                        Handle_0x3FD_Mux0(frame);
                        break;
                    case 1:
                        Handle_0x3FD_Mux1(frame);
                        break;
                    case 2:
                        Handle_0x3FD_Mux2(frame);
                        break;
                    default:
                        break;
                }
                break;
            }
            case 880:
                // Handle_880(frame);
                break;
            // case 923: {
            //     m_das_hands_on_state = (frame.data[5] >> 2) & 0x0F;
            //     m_923_counter += 1;
            //     break;
            // }
            case 280:
                if (m_enable_debug)
                    m_frame_to_debug_for_280 = frame;
                m_cur_gear = (EGear)(frame.data[2] >> 5);
                break;
        }

        if (m_enable_debug && m_last_print_counter++ % 1000 == 0) {
            Serial.printf(
                "FSD: %d(HW%d(%d)),Profile(3:%d|4:%d),SpeedLimit:%d,TOffsetPercent:%d,Offset:%d,UseAutoOffset:%d,Camera:%d,Gear:%s\n",
                m_is_fsd_enabled,
                m_use_hw4_code ? 4 : 3,
                m_use_hw3_profile_when_hw4,
                m_speed_profile_for_hw3,
                m_speed_profile_for_hw4,
                m_speed_limit,
                m_speed_offset,
                m_target_offset_percent,
                m_use_speed_offset_auto,
                m_enable_camera,
                ToString(m_cur_gear));

            // Serial.printf("0x3FD: 0:%s 1:%s 2:%s\n", ToBinaryString(m_frame_to_debug_vec_for_0x3DF[0]).c_str(), ToBinaryString(m_frame_to_debug_vec_for_0x3DF[1]).c_str(), ToBinaryString(m_frame_to_debug_vec_for_0x3DF[2]).c_str());
            // Serial.printf("0x3F8: %s \n", ToBinaryString(m_frame_to_debug_vec_for_0x3F8[0]).c_str());
            // Serial.printf("0x3F8: from %s, to %s \n", ToBinaryString(m_frame_to_debug_vec_for_0x370[0]).c_str(), ToBinaryString(m_frame_to_debug_vec_for_0x370[1]).c_str());
            // Serial.printf("280:  %s \n", ToBinaryString(m_frame_to_debug_for_280).c_str());
        }
    }

    // 工具函数
private:

    __attribute__((optimize("O3"))) std::string
    ToBinaryString(uint8_t i) {
        std::string b;
        b.reserve(8);
        for (int index = 0; index < sizeof(i) * 8; index += 1)
            b += ((i << index) & 0b10000000) ? "1" : "0";

        return b;
    }

    __attribute__((optimize("O3"))) inline int
    Clamp(int value, int min, int max) {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }

    __attribute__((optimize("O3"))) inline int
    Rerange(int value, int min, int max, int t_min, int t_max) {
        int t_len = t_max - t_min;
        int len = max - min;
        int p = (value - min) * 100 / len;
        return t_min + p * t_len / 100;
    }

    // 根据目标速度和当前限速，计算出应该把车机设置为多少的限速百分比
    __attribute__((optimize("O3"))) inline int
    CalcPercent(int targetSpeed, int speedLimit) {
        if (targetSpeed <= speedLimit)
            return 0;

        int rawPercent = (targetSpeed - speedLimit) * 100 / speedLimit;
        if (!m_use_hw4_code)
            return Rerange(Clamp(rawPercent, 0, 60), 0, 60, 0, 240);

        return Clamp(rawPercent, 0, 60), 0, 60, 0, 60;
    }

    __attribute__((optimize("O3"))) inline int
    CalcAutoCANOffset(int targetSpeed) {
        if (m_speed_limit == 0)
            return 0;
        return CalcPercent(targetSpeed, m_speed_limit);
    }

    __attribute__((optimize("O3"))) std::string
    ToHexString(const can_frame &frame) {
        std::ostringstream result;
        for (size_t i = 0; i < sizeof(frame.data); ++i)
            result << "0x" << std::hex << (int) frame.data[i] << ",";
        return result.str();
    }

    __attribute__((optimize("O3"))) std::string
    ToBinaryString(const can_frame &frame) {
        std::ostringstream result;
        for (size_t i = 0; i < sizeof(frame.data); ++i)
            result << i * 8 << ":" << ToBinaryString(frame.data[i]) << ";";

        return result.str();
    }

    __attribute__((optimize("O3"))) inline uint8_t
    ReadMuxID(const can_frame &frame) {
        return frame.data[0] & 0x07;
    }

    __attribute__((optimize("O3"))) inline void
    SetBit(can_frame &frame, int bit, bool value) {
        int byteIndex = bit / 8;
        int bitIndex = bit % 8;
        uint8_t mask = static_cast<uint8_t>(1U << bitIndex);

        if (value)
            frame.data[byteIndex] |= mask;
        else
            frame.data[byteIndex] &= static_cast<uint8_t>(~mask);
    }

    __attribute__((optimize("O3"))) inline uint8_t
    ComputeVehicleChecksum(const can_frame &frame, uint8_t checksumByteIndex = 7) {
        if (checksumByteIndex >= frame.can_dlc)
            return 0;

        uint16_t sum = static_cast<uint16_t>(frame.can_id & 0xFF) + static_cast<uint16_t>((frame.can_id >> 8) & 0xFF);
        for (uint8_t i = 0; i < frame.can_dlc; ++i)
        {
            if (i == checksumByteIndex)
                continue;
            sum += frame.data[i];
        }
        return static_cast<uint8_t>(sum & 0xFF);
    }

    const char*
    ToString(EGear gear) {
        switch (gear) {
            case EGear_P:
                return "P";
            case EGear_D:
                return "D";
            case EGear_R:
                return "R";
            case EGear_N:
                return "N";
            default:
                return "Unknown";
        }
    }

private:
    int m_speed_offset = 0;

    bool m_use_speed_offset_auto = false;
    uint8_t m_speed_offset_raw = 0;
    uint8_t m_speed_offset_raw_last_calc = 0;
    int m_speed_limit = 0;

    int m_speed_offset_v2 = 0;

    int32_t m_follow_distance = 0;
    int32_t m_speed_rule_selected = 0;

    bool m_is_fsd_enabled = false;
    int m_target_offset_percent = 0;

    int m_speed_profile_for_hw3 = 1;
    int m_speed_profile_for_hw4 = 1;

    bool m_use_hw4_code = true;
    bool m_use_hw3_profile_when_hw4 = false;

    std::vector<can_frame> m_frame_to_debug_vec_for_0x3DF;
    std::vector<can_frame> m_frame_to_debug_vec_for_0x3F8;
    std::vector<can_frame> m_frame_to_debug_vec_for_0x370;
    can_frame m_frame_to_debug_for_280;

    uint8_t m_to_use_data[5][8] = {
        // 0:11 f5  7 bd 20 1b 2e a6
        // 0:11 ff  7 b9 1f 47 23 cc
        // 0:12  b  8 55 20  a 23 3a

        {0x11,0xf6,0x7,0xbe,0x1e,0xae,0x27,0x32},
        {0x11,0xf3,0x7,0xb0,0x20,0x37,0x24,0xa9},
        {0x11, 0xf5, 0x7, 0xbd, 0x20, 0x1b, 0x2e, 0xa6},
        {0x11, 0xff, 0x7, 0xb9, 0x1f, 0x47, 0x23, 0xcc},
        {0x12, 0x0b, 0x8, 0x55, 0x20, 0xa,  0x23, 0x3a}
    };

    uint32_t m_index_to_use = 0;
    uint32_t m_index_use_counter = 0;

    // 是否应该使出超车道
    // 0x3F8 54位
    bool m_shichuchaochedao_bit = false;

    // 打开摄像头监控
    bool m_enable_camera = false;

    // 变道是否要确认?
    bool m_need_ensure_when_change_line = false;

    // 变道提醒震动和蜂鸣
    bool m_biandao_tixing_zhendong = false;
    bool m_biandao_tixing_fengming = false;

    // 当前档位
    EGear m_cur_gear = EGear_P;

    // 是否debug
    bool m_enable_debug = false;

    // 上一次打印的计数器
    uint32_t m_last_print_counter = 0;

    int32_t m_das_hands_on_state = 0;
    uint32_t m_923_counter = 0;
};

std::unique_ptr<FSDHandler> handler;

void
setup() {
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

__attribute__((optimize("O3"))) void
loop() {
    can_frame frame;
    int r = mcp->readMessage(&frame);
    if (r != MCP2515::ERROR_OK) {
        digitalWrite(LED_PIN, HIGH);
        return;
    }

    digitalWrite(LED_PIN, LOW);
    handler->HandleMessage(frame); // Fixed: handelMessage → handleMessage
}
