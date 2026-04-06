#ifndef ADS_MOTOR_IO_HPP
#define ADS_MOTOR_IO_HPP

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace ads_io {

// Beckhoff PLC 쪽 심볼 데이터 타입과 로컬 해석 타입을 맞추기 위한 설정이다.
// 피드백/명령 모두 4축 기준으로 읽고 쓰며, PLC 선언과 여기 타입이 일치해야 한다.
enum class PlcType {
    Real,
    LReal,
    Int16,
    UInt16,
    Int32,
    UInt32,
};

// PLC가 각도/속도를 degree 계열로 제공하는 경우 내부 제어는 rad 기준이므로 변환이 필요하다.
enum class AngleUnit {
    Rad,
    Deg,
};

// 제어 루프는 모터 토크[Nm]를 계산하지만, 드라이버에 따라 iq count를 직접 쓰는 경우도 있어
// ADS 출력 심볼에 어떤 값을 기록할지 선택할 수 있게 분리했다.
enum class CommandSource {
    Torque,
    Iq,
};

// PLC 심볼 바인딩 방식은 두 가지다.
// 1) 배열 심볼 하나: MAIN.motorPos 같은 ARRAY[0..3] OF ...
// 2) 개별 심볼 한 개 또는 네 개:
//    - 1개면 단일 MotorStatus/MotorCmd 스타일 PLC 를 1번 모터 채널에 연결
//    - 4개면 MAIN.motor1Pos, MAIN.motor2Pos, ... 처럼 축별 연결
struct SymbolBinding {
    std::string array_symbol;
    std::vector<std::string> symbols;

    bool empty() const;
    bool is_array() const;
    std::string describe() const;
};

struct Config {
    // remote_* 는 Beckhoff/TwinCAT 대상 정보이고, local_net_id 는 Ubuntu 클라이언트의 AMS 주소다.
    std::string remote_ip;
    std::string remote_net_id;
    std::string local_net_id;
    uint16_t port = 851;
    uint32_t timeout_ms = 5000;
    bool read_only = false;
    bool write_only = false;

    // 피드백 심볼은 position 필수, velocity 선택이다.
    // velocity 가 없으면 상위 계층에서 position 차분으로 속도를 추정한다.
    SymbolBinding motor_pos_symbols;
    SymbolBinding motor_vel_symbols;
    SymbolBinding motor_cmd_symbols;
    SymbolBinding motor_pos_ref_symbols;
    SymbolBinding motor_vel_ref_symbols;
    // 단일 MotorStatus / MotorCmd 구조체를 ADS blob read/write 하는 경로다.
    std::string motor_status_bundle_symbol;
    std::string motor_command_bundle_symbol;
    std::string motor_enable_symbol;
    std::string motor_kp_symbol;
    std::string motor_kd_symbol;
    // 예전 command/pos_ref/vel_ref 배열 번들을 그대로 유지해야 할 때 쓰는 호환 경로다.
    std::string motor_output_bundle_symbol;

    PlcType feedback_plc_type = PlcType::LReal;
    PlcType command_plc_type = PlcType::Real;
    PlcType reference_plc_type = PlcType::LReal;
    PlcType gain_plc_type = PlcType::LReal;
    AngleUnit motor_pos_unit = AngleUnit::Rad;
    AngleUnit motor_vel_unit = AngleUnit::Rad;
    CommandSource command_source = CommandSource::Torque;
};

struct Feedback {
    // 모든 값은 제어기에서 바로 쓸 수 있도록 rad / rad/s 기준으로 변환해 반환한다.
    std::array<double, 4> motor_pos_rad {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> motor_vel_rad {0.0, 0.0, 0.0, 0.0};
    std::array<bool, 4> pos_valid {false, false, false, false};
    std::array<bool, 4> vel_valid {false, false, false, false};
    uint64_t sample_seq = 0;
    std::chrono::steady_clock::time_point sample_time {};
};

struct Command {
    // 상위 제어기는 토크를 계산하고, 필요하면 동시에 iq 로도 변환해 둔다.
    // 실제 어떤 배열이 PLC 로 기록되는지는 Config::command_source 가 결정한다.
    std::array<double, 4> motor_torque_nm {0.0, 0.0, 0.0, 0.0};
    std::array<int16_t, 4> motor_iq {0, 0, 0, 0};

    // Beckhoff 쪽에서 참조 궤적을 함께 모니터링하거나 후단 제어에 사용할 수 있게
    // 모터축 기준 ref position / ref velocity 도 별도 심볼로 내보낸다.
    std::array<double, 4> motor_pos_ref_rad {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> motor_vel_ref_rad {0.0, 0.0, 0.0, 0.0};

    // 단일 MotorCmd 구조체를 쓰는 PLC 와 맞추기 위한 보조 필드다.
    bool enable = false;
    double motor_kp = 0.0;
    double motor_kd = 0.0;
};

class AdsMotorIo {
public:
    explicit AdsMotorIo(const Config& config);
    ~AdsMotorIo();

    AdsMotorIo(const AdsMotorIo&) = delete;
    AdsMotorIo& operator=(const AdsMotorIo&) = delete;

    // PLC -> Ubuntu: 현재 모터 위치/속도 읽기
    Feedback ReadFeedback() const;
    // Ubuntu -> PLC: 현재 제어 명령 쓰기
    void WriteCommand(const Command& command) const;
    // fault/shutdown 시 모든 출력 심볼을 0 으로 정리하는 용도
    void WriteZero() const;

    std::string describe_feedback_targets() const;
    std::string describe_command_targets() const;

private:
    struct Impl;
    Impl* impl_;
};

const char* to_string(PlcType type);
const char* to_string(AngleUnit unit);
const char* to_string(CommandSource source);

}  // namespace ads_io

#endif
