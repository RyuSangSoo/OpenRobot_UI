#include "ads/ads_motor_io.hpp"

#include "AdsDef.h"
#include "AdsDevice.h"
#include "AdsException.h"
#include "AdsLib.h"
#include "AdsNotificationOOI.h"

#include <algorithm>
#include <cstddef>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <memory>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace ads_io {

namespace {

constexpr size_t kMotorCount = 4;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;

enum class NotificationSlot {
    StatusBundle,
    PosArray,
    VelArray,
    PosSingle0,
    PosSingle1,
    PosSingle2,
    PosSingle3,
    VelSingle0,
    VelSingle1,
    VelSingle2,
    VelSingle3,
};

struct NotificationRoute {
    void* impl = nullptr;
    NotificationSlot slot = NotificationSlot::PosArray;
};

std::mutex g_notification_registry_mutex;
std::unordered_map<uint32_t, NotificationRoute> g_notification_registry;
uint32_t g_next_notification_user = 1;

std::string join_symbols(const std::vector<std::string>& symbols) {
    std::ostringstream out;
    for (size_t i = 0; i < symbols.size(); ++i) {
        if (i != 0) {
            out << ", ";
        }
        out << symbols[i];
    }
    return out.str();
}

size_t binding_value_count(const SymbolBinding& binding) {
    return binding.is_array() ? kMotorCount : binding.symbols.size();
}

template <typename T>
std::array<T, kMotorCount> read_values(const AdsDevice& device,
                                       const SymbolBinding& binding,
                                       const std::vector<AdsHandle>& handles) {
    std::array<T, kMotorCount> values {};
    // 배열 심볼은 ADS handle 하나로 4축 데이터를 한 번에 읽는다.
    // PLC 선언이 ARRAY[0..3] OF T 라면 sizeof(values) 와 정확히 맞아야 한다.
    if (binding.is_array()) {
        uint32_t bytes_read = 0;
        const auto error = device.ReadReqEx2(ADSIGRP_SYM_VALBYHND, *handles.front(),
                                             sizeof(values), values.data(), &bytes_read);
        if (error || bytes_read != sizeof(values)) {
            throw AdsException(error ? error : ADSERR_DEVICE_INVALIDSIZE);
        }
        return values;
    }

    // 개별 심볼 모드에서는 1개 또는 4개를 지원한다.
    const size_t count = binding_value_count(binding);
    for (size_t i = 0; i < count; ++i) {
        uint32_t bytes_read = 0;
        const auto error = device.ReadReqEx2(ADSIGRP_SYM_VALBYHND, *handles[i], sizeof(T),
                                             &values[i], &bytes_read);
        if (error || bytes_read != sizeof(T)) {
            throw AdsException(error ? error : ADSERR_DEVICE_INVALIDSIZE);
        }
    }
    return values;
}

template <typename T>
void write_values(const AdsDevice& device,
                  const SymbolBinding& binding,
                  const std::vector<AdsHandle>& handles,
                  const std::array<T, kMotorCount>& values) {
    // 명령도 read 와 동일하게 배열/개별 심볼 두 경로를 모두 지원한다.
    if (binding.is_array()) {
        const auto error = device.WriteReqEx(ADSIGRP_SYM_VALBYHND, *handles.front(),
                                             sizeof(values), values.data());
        if (error) {
            throw AdsException(error);
        }
        return;
    }

    const size_t count = binding_value_count(binding);
    for (size_t i = 0; i < count; ++i) {
        const auto error =
            device.WriteReqEx(ADSIGRP_SYM_VALBYHND, *handles[i], sizeof(T), &values[i]);
        if (error) {
            throw AdsException(error);
        }
    }
}

template <typename T>
void write_scalar_value(const AdsDevice& device, const AdsHandle& handle, const T value) {
    const auto error = device.WriteReqEx(ADSIGRP_SYM_VALBYHND, *handle, sizeof(T), &value);
    if (error) {
        throw AdsException(error);
    }
}

template <typename T>
std::array<double, kMotorCount> convert_feedback(const std::array<T, kMotorCount>& raw_values,
                                                 const AngleUnit unit) {
    std::array<double, kMotorCount> converted {};
    for (size_t i = 0; i < kMotorCount; ++i) {
        converted[i] = static_cast<double>(raw_values[i]);
        // 제어기 내부 기준은 rad 이므로 PLC 가 degree 로 보내는 경우 여기서 통일한다.
        if (unit == AngleUnit::Deg) {
            converted[i] *= kDegToRad;
        }
    }
    return converted;
}

template <typename T>
double convert_scalar_feedback(const T raw_value, const AngleUnit unit) {
    double converted = static_cast<double>(raw_value);
    if (unit == AngleUnit::Deg) {
        converted *= kDegToRad;
    }
    return converted;
}

template <typename T>
std::array<T, kMotorCount> convert_command(const Command& command, const CommandSource source) {
    std::array<T, kMotorCount> converted {};
    for (size_t i = 0; i < kMotorCount; ++i) {
        // PLC 쪽 명령 심볼이 토크용인지 iq용인지에 따라 기록 값을 선택한다.
        if (source == CommandSource::Iq) {
            converted[i] = static_cast<T>(command.motor_iq[i]);
        } else {
            converted[i] = static_cast<T>(command.motor_torque_nm[i]);
        }
    }
    return converted;
}

template <typename T>
std::array<T, kMotorCount> cast_array(const std::array<double, kMotorCount>& values) {
    std::array<T, kMotorCount> converted {};
    for (size_t i = 0; i < kMotorCount; ++i) {
        converted[i] = static_cast<T>(values[i]);
    }
    return converted;
}

template <typename TCommand, typename TReference>
struct OutputBundle {
    // PLC STRUCT도 이 순서와 타입을 그대로 맞춰야 한다.
    std::array<TCommand, kMotorCount> command;
    std::array<TReference, kMotorCount> pos_ref;
    std::array<TReference, kMotorCount> vel_ref;
};

#pragma pack(push, 1)
template <typename TReference, typename TCommand, typename TGain>
struct MotorCmdStructT {
    uint8_t bEnable;
    TReference nRefPos;
    TReference nRefVel;
    TCommand nRefTorque;
    TGain nKp;
    TGain nKd;
};

template <typename TFeedback>
struct MotorStatusStructT {
    uint8_t nSelectedMode;
    uint8_t nFaultCode;
    TFeedback nActPos;
    TFeedback nActVel;
    TFeedback nActTorque;
    uint16_t nBusVoltage;
    int8_t nTempMotor;
    int8_t nTempMosfet;
    uint16_t nEncoderPos;
    uint8_t bFault;
};
#pragma pack(pop)

template <typename TBundle>
void write_blob(const AdsDevice& device, const AdsHandle& handle, const TBundle& payload) {
    static_assert(std::is_standard_layout_v<TBundle>, "ADS bundle payload must be standard layout");
    const auto error = device.WriteReqEx(ADSIGRP_SYM_VALBYHND, *handle, sizeof(TBundle), &payload);
    if (error) {
        throw AdsException(error);
    }
}

template <typename TBundle>
TBundle read_blob(const AdsDevice& device, const AdsHandle& handle) {
    static_assert(std::is_standard_layout_v<TBundle>, "ADS bundle payload must be standard layout");
    TBundle payload {};
    uint32_t bytes_read = 0;
    const auto error = device.ReadReqEx2(ADSIGRP_SYM_VALBYHND, *handle, sizeof(TBundle),
                                         &payload, &bytes_read);
    if (error || bytes_read != sizeof(TBundle)) {
        throw AdsException(error ? error : ADSERR_DEVICE_INVALIDSIZE);
    }
    return payload;
}

template <typename T>
T cast_scalar_value(double value) {
    if constexpr (std::is_integral_v<T>) {
        const double min_value = static_cast<double>(std::numeric_limits<T>::lowest());
        const double max_value = static_cast<double>(std::numeric_limits<T>::max());
        value = std::clamp(value, min_value, max_value);
    }
    return static_cast<T>(value);
}

template <typename TFeedback>
Feedback convert_status_bundle_feedback(const MotorStatusStructT<TFeedback>& status,
                                        const Config& config) {
    Feedback feedback;
    feedback.motor_pos_rad[0] = convert_scalar_feedback<TFeedback>(status.nActPos, config.motor_pos_unit);
    feedback.motor_vel_rad[0] = convert_scalar_feedback<TFeedback>(status.nActVel, config.motor_vel_unit);
    feedback.pos_valid[0] = true;
    feedback.vel_valid[0] = true;
    return feedback;
}

template <typename TFeedback>
Feedback convert_status_bundle_feedback_array(const std::array<MotorStatusStructT<TFeedback>, kMotorCount>& statuses,
                                              const Config& config) {
    Feedback feedback;
    for (size_t i = 0; i < kMotorCount; ++i) {
        feedback.motor_pos_rad[i] = convert_scalar_feedback<TFeedback>(statuses[i].nActPos, config.motor_pos_unit);
        feedback.motor_vel_rad[i] = convert_scalar_feedback<TFeedback>(statuses[i].nActVel, config.motor_vel_unit);
        feedback.pos_valid[i] = true;
        feedback.vel_valid[i] = true;
    }
    return feedback;
}

uint32_t motor_status_bundle_size(const Config& config) {
    switch (config.feedback_plc_type) {
        case PlcType::Real:
            return static_cast<uint32_t>(sizeof(MotorStatusStructT<float>));
        case PlcType::LReal:
            return static_cast<uint32_t>(sizeof(MotorStatusStructT<double>));
        case PlcType::Int16:
            return static_cast<uint32_t>(sizeof(MotorStatusStructT<int16_t>));
        case PlcType::UInt16:
            return static_cast<uint32_t>(sizeof(MotorStatusStructT<uint16_t>));
        case PlcType::Int32:
            return static_cast<uint32_t>(sizeof(MotorStatusStructT<int32_t>));
        case PlcType::UInt32:
            return static_cast<uint32_t>(sizeof(MotorStatusStructT<uint32_t>));
    }
    throw std::runtime_error("Unsupported feedback bundle PLC type");
}

template <typename TFeedback>
Feedback parse_status_bundle_notification(const AdsNotificationHeader* notification,
                                          const Config& config) {
    if (notification->cbSampleSize != sizeof(MotorStatusStructT<TFeedback>)) {
        throw std::runtime_error("ADS notification status bundle size mismatch");
    }
    MotorStatusStructT<TFeedback> status {};
    std::memcpy(&status, notification + 1, sizeof(status));
    return convert_status_bundle_feedback(status, config);
}

template <typename TFeedback>
Feedback parse_status_bundle_array_notification(const AdsNotificationHeader* notification,
                                                const Config& config) {
    if (notification->cbSampleSize != sizeof(std::array<MotorStatusStructT<TFeedback>, kMotorCount>)) {
        throw std::runtime_error("ADS notification status bundle array size mismatch");
    }
    std::array<MotorStatusStructT<TFeedback>, kMotorCount> statuses {};
    std::memcpy(statuses.data(), notification + 1, sizeof(statuses));
    return convert_status_bundle_feedback_array(statuses, config);
}

Feedback read_status_bundle_feedback(const AdsDevice& device,
                                     const AdsHandle& handle,
                                     const Config& config) {
    switch (config.feedback_plc_type) {
        case PlcType::Real:
            return convert_status_bundle_feedback(read_blob<MotorStatusStructT<float>>(device, handle), config);
        case PlcType::LReal:
            return convert_status_bundle_feedback(read_blob<MotorStatusStructT<double>>(device, handle), config);
        case PlcType::Int16:
            return convert_status_bundle_feedback(read_blob<MotorStatusStructT<int16_t>>(device, handle), config);
        case PlcType::UInt16:
            return convert_status_bundle_feedback(read_blob<MotorStatusStructT<uint16_t>>(device, handle), config);
        case PlcType::Int32:
            return convert_status_bundle_feedback(read_blob<MotorStatusStructT<int32_t>>(device, handle), config);
        case PlcType::UInt32:
            return convert_status_bundle_feedback(read_blob<MotorStatusStructT<uint32_t>>(device, handle), config);
    }
    throw std::runtime_error("Unsupported feedback bundle PLC type");
}

Feedback read_status_bundle_feedback_array(const AdsDevice& device,
                                           const AdsHandle& handle,
                                           const Config& config) {
    switch (config.feedback_plc_type) {
        case PlcType::Real:
            return convert_status_bundle_feedback_array(read_blob<std::array<MotorStatusStructT<float>, kMotorCount>>(device, handle), config);
        case PlcType::LReal:
            return convert_status_bundle_feedback_array(read_blob<std::array<MotorStatusStructT<double>, kMotorCount>>(device, handle), config);
        case PlcType::Int16:
            return convert_status_bundle_feedback_array(read_blob<std::array<MotorStatusStructT<int16_t>, kMotorCount>>(device, handle), config);
        case PlcType::UInt16:
            return convert_status_bundle_feedback_array(read_blob<std::array<MotorStatusStructT<uint16_t>, kMotorCount>>(device, handle), config);
        case PlcType::Int32:
            return convert_status_bundle_feedback_array(read_blob<std::array<MotorStatusStructT<int32_t>, kMotorCount>>(device, handle), config);
        case PlcType::UInt32:
            return convert_status_bundle_feedback_array(read_blob<std::array<MotorStatusStructT<uint32_t>, kMotorCount>>(device, handle), config);
    }
    throw std::runtime_error("Unsupported feedback bundle PLC type");
}

void assign_feedback_index(Feedback& dst, const Feedback& src, const size_t index) {
    dst.motor_pos_rad[index] = src.motor_pos_rad[0];
    dst.motor_vel_rad[index] = src.motor_vel_rad[0];
    dst.pos_valid[index] = src.pos_valid[0];
    dst.vel_valid[index] = src.vel_valid[0];
}

double selected_command_value(const Command& command, const CommandSource source, const size_t index) {
    return source == CommandSource::Iq ? static_cast<double>(command.motor_iq[index])
                                       : command.motor_torque_nm[index];
}

template <typename TReference, typename TCommand, typename TGain>
MotorCmdStructT<TReference, TCommand, TGain> convert_motor_cmd_struct(const Command& command,
                                                                      const Config& config,
                                                                      const size_t motor_index) {
    MotorCmdStructT<TReference, TCommand, TGain> payload {};
    const double pos_ref = config.motor_pos_unit == AngleUnit::Deg
                               ? command.motor_pos_ref_rad[motor_index] * kRadToDeg
                               : command.motor_pos_ref_rad[motor_index];
    const double vel_ref = config.motor_vel_unit == AngleUnit::Deg
                               ? command.motor_vel_ref_rad[motor_index] * kRadToDeg
                               : command.motor_vel_ref_rad[motor_index];
    payload.bEnable = command.enable ? 1U : 0U;
    payload.nRefPos = cast_scalar_value<TReference>(pos_ref);
    payload.nRefVel = cast_scalar_value<TReference>(vel_ref);
    payload.nRefTorque = cast_scalar_value<TCommand>(selected_command_value(command, config.command_source, motor_index));
    payload.nKp = cast_scalar_value<TGain>(command.motor_kp);
    payload.nKd = cast_scalar_value<TGain>(command.motor_kd);
    return payload;
}

template <typename TReference, typename TCommand, typename TGain>
uint32_t motor_cmd_bundle_size() {
    return static_cast<uint32_t>(sizeof(MotorCmdStructT<TReference, TCommand, TGain>));
}

template <typename TReference, typename TCommand>
uint32_t motor_cmd_bundle_size_for_gain(const PlcType gain_type) {
    switch (gain_type) {
        case PlcType::Real:
            return motor_cmd_bundle_size<TReference, TCommand, float>();
        case PlcType::LReal:
            return motor_cmd_bundle_size<TReference, TCommand, double>();
        case PlcType::Int16:
            return motor_cmd_bundle_size<TReference, TCommand, int16_t>();
        case PlcType::UInt16:
            return motor_cmd_bundle_size<TReference, TCommand, uint16_t>();
        case PlcType::Int32:
            return motor_cmd_bundle_size<TReference, TCommand, int32_t>();
        case PlcType::UInt32:
            return motor_cmd_bundle_size<TReference, TCommand, uint32_t>();
    }
    throw std::runtime_error("Unsupported gain bundle PLC type");
}

template <typename TReference>
uint32_t motor_cmd_bundle_size_for_command(const Config& config) {
    switch (config.command_plc_type) {
        case PlcType::Real:
            return motor_cmd_bundle_size_for_gain<TReference, float>(config.gain_plc_type);
        case PlcType::LReal:
            return motor_cmd_bundle_size_for_gain<TReference, double>(config.gain_plc_type);
        case PlcType::Int16:
            return motor_cmd_bundle_size_for_gain<TReference, int16_t>(config.gain_plc_type);
        case PlcType::UInt16:
            return motor_cmd_bundle_size_for_gain<TReference, uint16_t>(config.gain_plc_type);
        case PlcType::Int32:
            return motor_cmd_bundle_size_for_gain<TReference, int32_t>(config.gain_plc_type);
        case PlcType::UInt32:
            return motor_cmd_bundle_size_for_gain<TReference, uint32_t>(config.gain_plc_type);
    }
    throw std::runtime_error("Unsupported command bundle PLC type");
}

uint32_t motor_cmd_bundle_size(const Config& config) {
    switch (config.reference_plc_type) {
        case PlcType::Real:
            return motor_cmd_bundle_size_for_command<float>(config);
        case PlcType::LReal:
            return motor_cmd_bundle_size_for_command<double>(config);
        case PlcType::Int16:
            return motor_cmd_bundle_size_for_command<int16_t>(config);
        case PlcType::UInt16:
            return motor_cmd_bundle_size_for_command<uint16_t>(config);
        case PlcType::Int32:
            return motor_cmd_bundle_size_for_command<int32_t>(config);
        case PlcType::UInt32:
            return motor_cmd_bundle_size_for_command<uint32_t>(config);
    }
    throw std::runtime_error("Unsupported reference bundle PLC type");
}

template <typename TReference, typename TCommand, typename TGain>
void write_motor_cmd_bundle_payload(const AdsDevice& device,
                                    const AdsHandle& handle,
                                    const Command& command,
                                    const Config& config,
                                    const size_t motor_index) {
    write_blob(device, handle, convert_motor_cmd_struct<TReference, TCommand, TGain>(command, config, motor_index));
}

template <typename TReference, typename TCommand, typename TGain>
void write_motor_cmd_bundle_array_payload(const AdsDevice& device,
                                          const AdsHandle& handle,
                                          const Command& command,
                                          const Config& config) {
    std::array<MotorCmdStructT<TReference, TCommand, TGain>, kMotorCount> payload {};
    for (size_t i = 0; i < kMotorCount; ++i) {
        payload[i] = convert_motor_cmd_struct<TReference, TCommand, TGain>(command, config, i);
    }
    write_blob(device, handle, payload);
}

template <typename TReference, typename TCommand>
void write_motor_cmd_bundle_for_gain(const AdsDevice& device,
                                     const AdsHandle& handle,
                                     const Command& command,
                                     const Config& config,
                                     const size_t motor_index,
                                     const bool array_mode) {
    switch (config.gain_plc_type) {
        case PlcType::Real:
            return array_mode ? write_motor_cmd_bundle_array_payload<TReference, TCommand, float>(device, handle, command, config)
                              : write_motor_cmd_bundle_payload<TReference, TCommand, float>(device, handle, command, config, motor_index);
        case PlcType::LReal:
            return array_mode ? write_motor_cmd_bundle_array_payload<TReference, TCommand, double>(device, handle, command, config)
                              : write_motor_cmd_bundle_payload<TReference, TCommand, double>(device, handle, command, config, motor_index);
        case PlcType::Int16:
            return array_mode ? write_motor_cmd_bundle_array_payload<TReference, TCommand, int16_t>(device, handle, command, config)
                              : write_motor_cmd_bundle_payload<TReference, TCommand, int16_t>(device, handle, command, config, motor_index);
        case PlcType::UInt16:
            return array_mode ? write_motor_cmd_bundle_array_payload<TReference, TCommand, uint16_t>(device, handle, command, config)
                              : write_motor_cmd_bundle_payload<TReference, TCommand, uint16_t>(device, handle, command, config, motor_index);
        case PlcType::Int32:
            return array_mode ? write_motor_cmd_bundle_array_payload<TReference, TCommand, int32_t>(device, handle, command, config)
                              : write_motor_cmd_bundle_payload<TReference, TCommand, int32_t>(device, handle, command, config, motor_index);
        case PlcType::UInt32:
            return array_mode ? write_motor_cmd_bundle_array_payload<TReference, TCommand, uint32_t>(device, handle, command, config)
                              : write_motor_cmd_bundle_payload<TReference, TCommand, uint32_t>(device, handle, command, config, motor_index);
    }
    throw std::runtime_error("Unsupported gain bundle PLC type");
}

template <typename TReference>
void write_motor_cmd_bundle_for_command(const AdsDevice& device,
                                        const AdsHandle& handle,
                                        const Command& command,
                                        const Config& config,
                                        const size_t motor_index,
                                        const bool array_mode) {
    switch (config.command_plc_type) {
        case PlcType::Real:
            return write_motor_cmd_bundle_for_gain<TReference, float>(device, handle, command, config, motor_index, array_mode);
        case PlcType::LReal:
            return write_motor_cmd_bundle_for_gain<TReference, double>(device, handle, command, config, motor_index, array_mode);
        case PlcType::Int16:
            return write_motor_cmd_bundle_for_gain<TReference, int16_t>(device, handle, command, config, motor_index, array_mode);
        case PlcType::UInt16:
            return write_motor_cmd_bundle_for_gain<TReference, uint16_t>(device, handle, command, config, motor_index, array_mode);
        case PlcType::Int32:
            return write_motor_cmd_bundle_for_gain<TReference, int32_t>(device, handle, command, config, motor_index, array_mode);
        case PlcType::UInt32:
            return write_motor_cmd_bundle_for_gain<TReference, uint32_t>(device, handle, command, config, motor_index, array_mode);
    }
    throw std::runtime_error("Unsupported command bundle PLC type");
}

void write_motor_cmd_bundle(const AdsDevice& device,
                            const AdsHandle& handle,
                            const Command& command,
                            const Config& config,
                            const size_t motor_index,
                            const bool array_mode) {
    switch (config.reference_plc_type) {
        case PlcType::Real:
            return write_motor_cmd_bundle_for_command<float>(device, handle, command, config, motor_index, array_mode);
        case PlcType::LReal:
            return write_motor_cmd_bundle_for_command<double>(device, handle, command, config, motor_index, array_mode);
        case PlcType::Int16:
            return write_motor_cmd_bundle_for_command<int16_t>(device, handle, command, config, motor_index, array_mode);
        case PlcType::UInt16:
            return write_motor_cmd_bundle_for_command<uint16_t>(device, handle, command, config, motor_index, array_mode);
        case PlcType::Int32:
            return write_motor_cmd_bundle_for_command<int32_t>(device, handle, command, config, motor_index, array_mode);
        case PlcType::UInt32:
            return write_motor_cmd_bundle_for_command<uint32_t>(device, handle, command, config, motor_index, array_mode);
    }
    throw std::runtime_error("Unsupported reference bundle PLC type");
}

void validate_binding(const SymbolBinding& binding, const char* what, const bool required) {
    if (binding.empty()) {
        if (required) {
            throw std::runtime_error(std::string(what) + " symbol is required");
        }
        return;
    }
    if (!binding.array_symbol.empty() && !binding.symbols.empty()) {
        throw std::runtime_error(std::string(what) + " must use array symbol or per-motor symbols");
    }
    if (!binding.symbols.empty() &&
        binding.symbols.size() != 1 &&
        binding.symbols.size() != kMotorCount) {
        throw std::runtime_error(std::string(what) + " must provide 1 or 4 symbols");
    }
}

std::vector<AdsHandle> make_handles(const AdsDevice& device, const SymbolBinding& binding) {
    std::vector<AdsHandle> handles;
    if (binding.empty()) {
        return handles;
    }
    // AdsHandle 은 심볼 이름 조회 비용을 초기화 시점에 한 번만 내고,
    // 주기 루프에서는 handle 기반 read/write 만 수행하도록 만든다.
    if (binding.is_array()) {
        handles.push_back(device.GetHandle(binding.array_symbol));
        return handles;
    }
    handles.reserve(binding.symbols.size());
    for (const auto& symbol : binding.symbols) {
        handles.push_back(device.GetHandle(symbol));
    }
    return handles;
}

std::unique_ptr<AdsHandle> make_optional_handle(const AdsDevice& device, const std::string& symbol) {
    if (symbol.empty()) {
        return std::unique_ptr<AdsHandle>();
    }
    return std::make_unique<AdsHandle>(device.GetHandle(symbol));
}

std::vector<AdsHandle> make_optional_handles(const AdsDevice& device, const std::string& symbols_csv) {
    std::vector<AdsHandle> handles;
    if (symbols_csv.empty()) {
        return handles;
    }

    size_t begin = 0;
    while (begin <= symbols_csv.size()) {
        const size_t end = symbols_csv.find(',', begin);
        std::string token = symbols_csv.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
        const size_t first = token.find_first_not_of(" 	");
        if (first != std::string::npos) {
            const size_t last = token.find_last_not_of(" 	");
            token = token.substr(first, last - first + 1);
            handles.push_back(device.GetHandle(token));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }

    if (!handles.empty() && handles.size() != 1 && handles.size() != kMotorCount) {
        throw std::runtime_error("ADS bundle symbols must provide 1 or 4 symbols");
    }
    return handles;
}

uint32_t query_symbol_size(const AdsDevice& device, const std::string& symbol) {
    if (symbol.empty() || symbol.find(',') != std::string::npos) {
        return 0;
    }

    std::array<uint8_t, 512> buffer {};
    uint32_t bytes_read = 0;
    const auto error = device.ReadWriteReqEx2(ADSIGRP_SYM_INFOBYNAMEEX, 0,
                                              static_cast<uint32_t>(buffer.size()), buffer.data(),
                                              static_cast<uint32_t>(symbol.size()), symbol.c_str(),
                                              &bytes_read);
    if (error || bytes_read < sizeof(AdsSymbolEntry)) {
        throw AdsException(error ? error : ADSERR_DEVICE_INVALIDSIZE);
    }
    const auto* entry = reinterpret_cast<const AdsSymbolEntry*>(buffer.data());
    return entry->size;
}

void set_binding_valid(std::array<bool, kMotorCount>& valid,
                       const SymbolBinding& binding) {
    valid.fill(false);
    const size_t count = std::min(kMotorCount, binding_value_count(binding));
    for (size_t i = 0; i < count; ++i) {
        valid[i] = true;
    }
}

template <typename T>
std::array<double, kMotorCount> parse_array_notification(const AdsNotificationHeader* header,
                                                         const AngleUnit unit) {
    if (header->cbSampleSize != sizeof(std::array<T, kMotorCount>)) {
        throw std::runtime_error("ADS notification array size mismatch");
    }

    std::array<T, kMotorCount> raw_values {};
    std::memcpy(raw_values.data(), header + 1, sizeof(raw_values));
    return convert_feedback(raw_values, unit);
}

template <typename T>
double parse_scalar_notification(const AdsNotificationHeader* header, const AngleUnit unit) {
    if (header->cbSampleSize != sizeof(T)) {
        throw std::runtime_error("ADS notification scalar size mismatch");
    }

    T raw_value {};
    std::memcpy(&raw_value, header + 1, sizeof(T));
    return convert_scalar_feedback(raw_value, unit);
}

uint32_t register_notification_route(void* impl, const NotificationSlot slot) {
    std::lock_guard<std::mutex> lock(g_notification_registry_mutex);
    const uint32_t user = g_next_notification_user++;
    g_notification_registry.emplace(user, NotificationRoute{impl, slot});
    return user;
}

void unregister_notification_route(const uint32_t user) {
    std::lock_guard<std::mutex> lock(g_notification_registry_mutex);
    g_notification_registry.erase(user);
}

}  // namespace

struct AdsMotorIo::Impl {
    static void NotificationCallback(const AmsAddr*,
                                     const AdsNotificationHeader* notification,
                                     const uint32_t hUser);
    void OnNotification(NotificationSlot slot, const AdsNotificationHeader* notification);
    void InstallNotifications();
    void PrimeFeedbackCache();

    explicit Impl(const Config& config_in)
        : config(config_in),
          device(create_device(config)),
          pos_handles(make_handles(device, config.motor_pos_symbols)),
          vel_handles(make_handles(device, config.motor_vel_symbols)),
          cmd_handles(make_handles(device, config.motor_cmd_symbols)),
          pos_ref_handles(make_handles(device, config.motor_pos_ref_symbols)),
          vel_ref_handles(make_handles(device, config.motor_vel_ref_symbols)),
          status_bundle_handles(make_optional_handles(device, config.motor_status_bundle_symbol)),
          command_bundle_handles(make_optional_handles(device, config.motor_command_bundle_symbol)),
          status_bundle_symbol_size(query_symbol_size(device, config.motor_status_bundle_symbol)),
          command_bundle_symbol_size(query_symbol_size(device, config.motor_command_bundle_symbol)),
          enable_handle(make_optional_handle(device, config.motor_enable_symbol)),
          kp_handle(make_optional_handle(device, config.motor_kp_symbol)),
          kd_handle(make_optional_handle(device, config.motor_kd_symbol)),
          bundle_handle(config.motor_output_bundle_symbol.empty()
                            ? std::unique_ptr<AdsHandle>()
                            : std::make_unique<AdsHandle>(device.GetHandle(config.motor_output_bundle_symbol))) {
        validate_binding(config.motor_pos_symbols, "ADS motor position",
                         !config.write_only && config.motor_status_bundle_symbol.empty());
        validate_binding(config.motor_vel_symbols, "ADS motor velocity", false);
        validate_binding(config.motor_cmd_symbols, "ADS motor command",
                         !config.read_only &&
                         config.motor_command_bundle_symbol.empty() &&
                         config.motor_output_bundle_symbol.empty());
        validate_binding(config.motor_pos_ref_symbols, "ADS motor position reference", false);
        validate_binding(config.motor_vel_ref_symbols, "ADS motor velocity reference", false);
        device.SetTimeout(config.timeout_ms);
        if (!config.write_only) {
            PrimeFeedbackCache();
            InstallNotifications();
        }
    }

    ~Impl() {
        notifications.clear();
        for (const auto user : notification_users) {
            unregister_notification_route(user);
        }
        notification_users.clear();
    }

    static AdsDevice create_device(const Config& config) {
        if (!config.local_net_id.empty()) {
            // Linux AdsLib 는 통신 시작 전에 로컬 AMS 주소를 먼저 지정해야
            // Beckhoff 쪽 route 와 일치하는 송신자로 인식된다.
            bhf::ads::SetLocalAddress(make_AmsNetId(config.local_net_id));
        }
        return AdsDevice{config.remote_ip, make_AmsNetId(config.remote_net_id), config.port};
    }

    Config config;
    AdsDevice device;
    std::vector<AdsHandle> pos_handles;
    std::vector<AdsHandle> vel_handles;
    std::vector<AdsHandle> cmd_handles;
    std::vector<AdsHandle> pos_ref_handles;
    std::vector<AdsHandle> vel_ref_handles;
    std::vector<AdsHandle> status_bundle_handles;
    std::vector<AdsHandle> command_bundle_handles;
    uint32_t status_bundle_symbol_size = 0;
    uint32_t command_bundle_symbol_size = 0;
    std::unique_ptr<AdsHandle> enable_handle;
    std::unique_ptr<AdsHandle> kp_handle;
    std::unique_ptr<AdsHandle> kd_handle;
    std::unique_ptr<AdsHandle> bundle_handle;
    std::mutex feedback_mutex;
    Feedback latest_feedback;
    std::vector<uint32_t> notification_users;
    std::vector<std::unique_ptr<AdsNotification>> notifications;
};

void AdsMotorIo::Impl::NotificationCallback(const AmsAddr*,
                                            const AdsNotificationHeader* notification,
                                            const uint32_t hUser) {
    NotificationRoute route;
    {
        std::lock_guard<std::mutex> lock(g_notification_registry_mutex);
        const auto it = g_notification_registry.find(hUser);
        if (it == g_notification_registry.end()) {
            return;
        }
        route = it->second;
    }

    static_cast<AdsMotorIo::Impl*>(route.impl)->OnNotification(route.slot, notification);
}

void AdsMotorIo::Impl::OnNotification(const NotificationSlot slot,
                                      const AdsNotificationHeader* notification) {
    std::lock_guard<std::mutex> lock(feedback_mutex);
    latest_feedback.sample_seq += 1;
    latest_feedback.sample_time = std::chrono::steady_clock::now();

    switch (slot) {
        case NotificationSlot::StatusBundle: {
            const uint32_t single_size = motor_status_bundle_size(config);
            const uint32_t array_size = single_size * static_cast<uint32_t>(kMotorCount);
            const bool array_mode = notification->cbSampleSize == array_size;
            const auto converted =
                config.feedback_plc_type == PlcType::Real ? (array_mode ? parse_status_bundle_array_notification<float>(notification, config)
                                                                        : parse_status_bundle_notification<float>(notification, config)) :
                config.feedback_plc_type == PlcType::LReal ? (array_mode ? parse_status_bundle_array_notification<double>(notification, config)
                                                                         : parse_status_bundle_notification<double>(notification, config)) :
                config.feedback_plc_type == PlcType::Int16 ? (array_mode ? parse_status_bundle_array_notification<int16_t>(notification, config)
                                                                         : parse_status_bundle_notification<int16_t>(notification, config)) :
                config.feedback_plc_type == PlcType::UInt16 ? (array_mode ? parse_status_bundle_array_notification<uint16_t>(notification, config)
                                                                          : parse_status_bundle_notification<uint16_t>(notification, config)) :
                config.feedback_plc_type == PlcType::Int32 ? (array_mode ? parse_status_bundle_array_notification<int32_t>(notification, config)
                                                                         : parse_status_bundle_notification<int32_t>(notification, config)) :
                                                              (array_mode ? parse_status_bundle_array_notification<uint32_t>(notification, config)
                                                                          : parse_status_bundle_notification<uint32_t>(notification, config));
            latest_feedback.motor_pos_rad = converted.motor_pos_rad;
            latest_feedback.motor_vel_rad = converted.motor_vel_rad;
            latest_feedback.pos_valid = converted.pos_valid;
            latest_feedback.vel_valid = converted.vel_valid;
            return;
        }
        case NotificationSlot::PosArray:
            switch (config.feedback_plc_type) {
                case PlcType::Real:
                    latest_feedback.motor_pos_rad =
                        parse_array_notification<float>(notification, config.motor_pos_unit);
                    break;
                case PlcType::LReal:
                    latest_feedback.motor_pos_rad =
                        parse_array_notification<double>(notification, config.motor_pos_unit);
                    break;
                case PlcType::Int16:
                    latest_feedback.motor_pos_rad =
                        parse_array_notification<int16_t>(notification, config.motor_pos_unit);
                    break;
                case PlcType::UInt16:
                    latest_feedback.motor_pos_rad =
                        parse_array_notification<uint16_t>(notification, config.motor_pos_unit);
                    break;
                case PlcType::Int32:
                    latest_feedback.motor_pos_rad =
                        parse_array_notification<int32_t>(notification, config.motor_pos_unit);
                    break;
                case PlcType::UInt32:
                    latest_feedback.motor_pos_rad =
                        parse_array_notification<uint32_t>(notification, config.motor_pos_unit);
                    break;
            }
            set_binding_valid(latest_feedback.pos_valid, config.motor_pos_symbols);
            return;
        case NotificationSlot::VelArray:
            switch (config.feedback_plc_type) {
                case PlcType::Real:
                    latest_feedback.motor_vel_rad =
                        parse_array_notification<float>(notification, config.motor_vel_unit);
                    break;
                case PlcType::LReal:
                    latest_feedback.motor_vel_rad =
                        parse_array_notification<double>(notification, config.motor_vel_unit);
                    break;
                case PlcType::Int16:
                    latest_feedback.motor_vel_rad =
                        parse_array_notification<int16_t>(notification, config.motor_vel_unit);
                    break;
                case PlcType::UInt16:
                    latest_feedback.motor_vel_rad =
                        parse_array_notification<uint16_t>(notification, config.motor_vel_unit);
                    break;
                case PlcType::Int32:
                    latest_feedback.motor_vel_rad =
                        parse_array_notification<int32_t>(notification, config.motor_vel_unit);
                    break;
                case PlcType::UInt32:
                    latest_feedback.motor_vel_rad =
                        parse_array_notification<uint32_t>(notification, config.motor_vel_unit);
                    break;
            }
            set_binding_valid(latest_feedback.vel_valid, config.motor_vel_symbols);
            return;
        case NotificationSlot::PosSingle0:
        case NotificationSlot::PosSingle1:
        case NotificationSlot::PosSingle2:
        case NotificationSlot::PosSingle3: {
            const size_t index = static_cast<size_t>(slot) - static_cast<size_t>(NotificationSlot::PosSingle0);
            switch (config.feedback_plc_type) {
                case PlcType::Real:
                    latest_feedback.motor_pos_rad[index] =
                        parse_scalar_notification<float>(notification, config.motor_pos_unit);
                    break;
                case PlcType::LReal:
                    latest_feedback.motor_pos_rad[index] =
                        parse_scalar_notification<double>(notification, config.motor_pos_unit);
                    break;
                case PlcType::Int16:
                    latest_feedback.motor_pos_rad[index] =
                        parse_scalar_notification<int16_t>(notification, config.motor_pos_unit);
                    break;
                case PlcType::UInt16:
                    latest_feedback.motor_pos_rad[index] =
                        parse_scalar_notification<uint16_t>(notification, config.motor_pos_unit);
                    break;
                case PlcType::Int32:
                    latest_feedback.motor_pos_rad[index] =
                        parse_scalar_notification<int32_t>(notification, config.motor_pos_unit);
                    break;
                case PlcType::UInt32:
                    latest_feedback.motor_pos_rad[index] =
                        parse_scalar_notification<uint32_t>(notification, config.motor_pos_unit);
                    break;
            }
            latest_feedback.pos_valid[index] = true;
            return;
        }
        case NotificationSlot::VelSingle0:
        case NotificationSlot::VelSingle1:
        case NotificationSlot::VelSingle2:
        case NotificationSlot::VelSingle3: {
            const size_t index = static_cast<size_t>(slot) - static_cast<size_t>(NotificationSlot::VelSingle0);
            switch (config.feedback_plc_type) {
                case PlcType::Real:
                    latest_feedback.motor_vel_rad[index] =
                        parse_scalar_notification<float>(notification, config.motor_vel_unit);
                    break;
                case PlcType::LReal:
                    latest_feedback.motor_vel_rad[index] =
                        parse_scalar_notification<double>(notification, config.motor_vel_unit);
                    break;
                case PlcType::Int16:
                    latest_feedback.motor_vel_rad[index] =
                        parse_scalar_notification<int16_t>(notification, config.motor_vel_unit);
                    break;
                case PlcType::UInt16:
                    latest_feedback.motor_vel_rad[index] =
                        parse_scalar_notification<uint16_t>(notification, config.motor_vel_unit);
                    break;
                case PlcType::Int32:
                    latest_feedback.motor_vel_rad[index] =
                        parse_scalar_notification<int32_t>(notification, config.motor_vel_unit);
                    break;
                case PlcType::UInt32:
                    latest_feedback.motor_vel_rad[index] =
                        parse_scalar_notification<uint32_t>(notification, config.motor_vel_unit);
                    break;
            }
            latest_feedback.vel_valid[index] = true;
            return;
        }
    }
}

void AdsMotorIo::Impl::InstallNotifications() {
    const auto make_attrib = [&](const uint32_t length) {
        return AdsNotificationAttrib{length, ADSTRANS_SERVERCYCLE, 0, {0}};
    };

    const auto register_array = [&](const std::string& symbol, const NotificationSlot slot,
                                    const uint32_t length) {
        const uint32_t user = register_notification_route(this, slot);
        notification_users.push_back(user);
        notifications.push_back(std::make_unique<AdsNotification>(
            device, symbol, make_attrib(length), &AdsMotorIo::Impl::NotificationCallback, user));
    };

    const auto register_scalar = [&](const std::string& symbol, const NotificationSlot slot,
                                     const uint32_t length) {
        const uint32_t user = register_notification_route(this, slot);
        notification_users.push_back(user);
        notifications.push_back(std::make_unique<AdsNotification>(
            device, symbol, make_attrib(length), &AdsMotorIo::Impl::NotificationCallback, user));
    };

    if (!config.motor_status_bundle_symbol.empty()) {
        if (status_bundle_handles.size() == 1) {
            const uint32_t single_size = motor_status_bundle_size(config);
            const uint32_t array_size = single_size * static_cast<uint32_t>(kMotorCount);
            const uint32_t notification_size = status_bundle_symbol_size == array_size ? array_size : single_size;
            register_array(config.motor_status_bundle_symbol, NotificationSlot::StatusBundle,
                           notification_size);
            return;
        }
        return;
    }

    const uint32_t scalar_length =
        config.feedback_plc_type == PlcType::Real ? sizeof(float) :
        config.feedback_plc_type == PlcType::LReal ? sizeof(double) :
        config.feedback_plc_type == PlcType::Int16 ? sizeof(int16_t) :
        config.feedback_plc_type == PlcType::UInt16 ? sizeof(uint16_t) :
        config.feedback_plc_type == PlcType::Int32 ? sizeof(int32_t) : sizeof(uint32_t);
    const uint32_t array_length = scalar_length * static_cast<uint32_t>(kMotorCount);

    if (config.motor_pos_symbols.is_array()) {
        register_array(config.motor_pos_symbols.array_symbol, NotificationSlot::PosArray, array_length);
    } else {
        for (size_t i = 0; i < config.motor_pos_symbols.symbols.size(); ++i) {
            register_scalar(config.motor_pos_symbols.symbols[i],
                            static_cast<NotificationSlot>(static_cast<size_t>(NotificationSlot::PosSingle0) + i),
                            scalar_length);
        }
    }

    if (config.motor_vel_symbols.empty()) {
        return;
    }

    if (config.motor_vel_symbols.is_array()) {
        register_array(config.motor_vel_symbols.array_symbol, NotificationSlot::VelArray, array_length);
    } else {
        for (size_t i = 0; i < config.motor_vel_symbols.symbols.size(); ++i) {
            register_scalar(config.motor_vel_symbols.symbols[i],
                            static_cast<NotificationSlot>(static_cast<size_t>(NotificationSlot::VelSingle0) + i),
                            scalar_length);
        }
    }
}

void AdsMotorIo::Impl::PrimeFeedbackCache() {
    Feedback primed;

    if (!status_bundle_handles.empty()) {
        if (status_bundle_handles.size() == 1) {
            const uint32_t single_size = motor_status_bundle_size(config);
            const uint32_t array_size = single_size * static_cast<uint32_t>(kMotorCount);
            if (status_bundle_symbol_size == array_size) {
                primed = read_status_bundle_feedback_array(device, status_bundle_handles.front(), config);
            } else {
                primed = read_status_bundle_feedback(device, status_bundle_handles.front(), config);
            }
        } else {
            for (size_t i = 0; i < status_bundle_handles.size(); ++i) {
                assign_feedback_index(primed, read_status_bundle_feedback(device, status_bundle_handles[i], config), i);
            }
        }
    } else {
        switch (config.feedback_plc_type) {
            case PlcType::Real: {
                const auto raw_pos = read_values<float>(device, config.motor_pos_symbols, pos_handles);
                primed.motor_pos_rad = convert_feedback(raw_pos, config.motor_pos_unit);
                break;
            }
            case PlcType::LReal: {
                const auto raw_pos = read_values<double>(device, config.motor_pos_symbols, pos_handles);
                primed.motor_pos_rad = convert_feedback(raw_pos, config.motor_pos_unit);
                break;
            }
            case PlcType::Int16: {
                const auto raw_pos = read_values<int16_t>(device, config.motor_pos_symbols, pos_handles);
                primed.motor_pos_rad = convert_feedback(raw_pos, config.motor_pos_unit);
                break;
            }
            case PlcType::UInt16: {
                const auto raw_pos = read_values<uint16_t>(device, config.motor_pos_symbols, pos_handles);
                primed.motor_pos_rad = convert_feedback(raw_pos, config.motor_pos_unit);
                break;
            }
            case PlcType::Int32: {
                const auto raw_pos = read_values<int32_t>(device, config.motor_pos_symbols, pos_handles);
                primed.motor_pos_rad = convert_feedback(raw_pos, config.motor_pos_unit);
                break;
            }
            case PlcType::UInt32: {
                const auto raw_pos = read_values<uint32_t>(device, config.motor_pos_symbols, pos_handles);
                primed.motor_pos_rad = convert_feedback(raw_pos, config.motor_pos_unit);
                break;
            }
        }
        set_binding_valid(primed.pos_valid, config.motor_pos_symbols);

        if (!config.motor_vel_symbols.empty()) {
            switch (config.feedback_plc_type) {
                case PlcType::Real: {
                    const auto raw_vel = read_values<float>(device, config.motor_vel_symbols, vel_handles);
                    primed.motor_vel_rad = convert_feedback(raw_vel, config.motor_vel_unit);
                    break;
                }
                case PlcType::LReal: {
                    const auto raw_vel = read_values<double>(device, config.motor_vel_symbols, vel_handles);
                    primed.motor_vel_rad = convert_feedback(raw_vel, config.motor_vel_unit);
                    break;
                }
                case PlcType::Int16: {
                    const auto raw_vel = read_values<int16_t>(device, config.motor_vel_symbols, vel_handles);
                    primed.motor_vel_rad = convert_feedback(raw_vel, config.motor_vel_unit);
                    break;
                }
                case PlcType::UInt16: {
                    const auto raw_vel = read_values<uint16_t>(device, config.motor_vel_symbols, vel_handles);
                    primed.motor_vel_rad = convert_feedback(raw_vel, config.motor_vel_unit);
                    break;
                }
                case PlcType::Int32: {
                    const auto raw_vel = read_values<int32_t>(device, config.motor_vel_symbols, vel_handles);
                    primed.motor_vel_rad = convert_feedback(raw_vel, config.motor_vel_unit);
                    break;
                }
                case PlcType::UInt32: {
                    const auto raw_vel = read_values<uint32_t>(device, config.motor_vel_symbols, vel_handles);
                    primed.motor_vel_rad = convert_feedback(raw_vel, config.motor_vel_unit);
                    break;
                }
            }
            set_binding_valid(primed.vel_valid, config.motor_vel_symbols);
        }
    }

    std::lock_guard<std::mutex> lock(feedback_mutex);
    primed.sample_seq = latest_feedback.sample_seq + 1;
    primed.sample_time = std::chrono::steady_clock::now();
    latest_feedback = primed;
}

bool SymbolBinding::empty() const {
    return array_symbol.empty() && symbols.empty();
}

bool SymbolBinding::is_array() const {
    return !array_symbol.empty();
}

std::string SymbolBinding::describe() const {
    if (is_array()) {
        return array_symbol;
    }
    return join_symbols(symbols);
}

AdsMotorIo::AdsMotorIo(const Config& config) : impl_(new Impl(config)) {
    // GetDeviceInfo() 가 성공하면
    // 1) TCP 연결
    // 2) AMS 주소 해석
    // 3) ADS 요청/응답
    // 까지는 정상이라는 뜻이다.
    const auto device_info = impl_->device.GetDeviceInfo();
    std::cout << "ADS I/O connected: device='" << device_info.name << "' remote="
              << impl_->config.remote_ip << ":" << impl_->config.port
              << " pos=" << describe_feedback_targets()
              << " cmd=" << describe_command_targets()
              << " command-source=" << to_string(impl_->config.command_source) << "\n";
}

AdsMotorIo::~AdsMotorIo() {
    delete impl_;
}

Feedback AdsMotorIo::ReadFeedback() const {
    if (impl_->config.write_only) {
        return Feedback{};
    }

    if (impl_->notifications.empty()) {
        impl_->PrimeFeedbackCache();
    }
    std::lock_guard<std::mutex> lock(impl_->feedback_mutex);
    return impl_->latest_feedback;
}

void AdsMotorIo::WriteCommand(const Command& command) const {
    if (!impl_->command_bundle_handles.empty()) {
        if (impl_->command_bundle_handles.size() == 1) {
            const uint32_t single_size = motor_cmd_bundle_size(impl_->config);
            const uint32_t array_size = single_size * static_cast<uint32_t>(kMotorCount);
            write_motor_cmd_bundle(impl_->device, impl_->command_bundle_handles.front(),
                                   command, impl_->config, 0,
                                   impl_->command_bundle_symbol_size == array_size);
        } else {
            for (size_t i = 0; i < impl_->command_bundle_handles.size(); ++i) {
                write_motor_cmd_bundle(impl_->device, impl_->command_bundle_handles[i],
                                       command, impl_->config, i, false);
            }
        }
        return;
    }

    if (impl_->enable_handle) {
        write_scalar_value<bool>(impl_->device, *impl_->enable_handle, command.enable);
    }

    if (impl_->kp_handle) {
        switch (impl_->config.gain_plc_type) {
            case PlcType::Real:
                write_scalar_value<float>(impl_->device, *impl_->kp_handle,
                                          static_cast<float>(command.motor_kp));
                break;
            case PlcType::LReal:
                write_scalar_value<double>(impl_->device, *impl_->kp_handle, command.motor_kp);
                break;
            case PlcType::Int16:
                write_scalar_value<int16_t>(impl_->device, *impl_->kp_handle,
                                            static_cast<int16_t>(command.motor_kp));
                break;
            case PlcType::UInt16:
                write_scalar_value<uint16_t>(impl_->device, *impl_->kp_handle,
                                             static_cast<uint16_t>(std::max(0.0, command.motor_kp)));
                break;
            case PlcType::Int32:
                write_scalar_value<int32_t>(impl_->device, *impl_->kp_handle,
                                            static_cast<int32_t>(command.motor_kp));
                break;
            case PlcType::UInt32:
                write_scalar_value<uint32_t>(impl_->device, *impl_->kp_handle,
                                             static_cast<uint32_t>(std::max(0.0, command.motor_kp)));
                break;
        }
    }

    if (impl_->kd_handle) {
        switch (impl_->config.gain_plc_type) {
            case PlcType::Real:
                write_scalar_value<float>(impl_->device, *impl_->kd_handle,
                                          static_cast<float>(command.motor_kd));
                break;
            case PlcType::LReal:
                write_scalar_value<double>(impl_->device, *impl_->kd_handle, command.motor_kd);
                break;
            case PlcType::Int16:
                write_scalar_value<int16_t>(impl_->device, *impl_->kd_handle,
                                            static_cast<int16_t>(command.motor_kd));
                break;
            case PlcType::UInt16:
                write_scalar_value<uint16_t>(impl_->device, *impl_->kd_handle,
                                             static_cast<uint16_t>(std::max(0.0, command.motor_kd)));
                break;
            case PlcType::Int32:
                write_scalar_value<int32_t>(impl_->device, *impl_->kd_handle,
                                            static_cast<int32_t>(command.motor_kd));
                break;
            case PlcType::UInt32:
                write_scalar_value<uint32_t>(impl_->device, *impl_->kd_handle,
                                             static_cast<uint32_t>(std::max(0.0, command.motor_kd)));
                break;
        }
    }

    if (impl_->bundle_handle) {
        const bool unsupported_bundle_type =
            impl_->config.command_plc_type == PlcType::Int32 ||
            impl_->config.command_plc_type == PlcType::UInt32 ||
            impl_->config.reference_plc_type == PlcType::Int32 ||
            impl_->config.reference_plc_type == PlcType::UInt32;
        if (unsupported_bundle_type) {
            throw std::runtime_error("ADS output bundle supports real/lreal/int16 only");
        }

        switch (impl_->config.command_plc_type) {
            case PlcType::Real: {
                switch (impl_->config.reference_plc_type) {
                    case PlcType::Real: {
                        const OutputBundle<float, float> payload {
                            convert_command<float>(command, impl_->config.command_source),
                            cast_array<float>(command.motor_pos_ref_rad),
                            cast_array<float>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::LReal: {
                        const OutputBundle<float, double> payload {
                            convert_command<float>(command, impl_->config.command_source),
                            command.motor_pos_ref_rad,
                            command.motor_vel_ref_rad,
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::Int16: {
                        const OutputBundle<float, int16_t> payload {
                            convert_command<float>(command, impl_->config.command_source),
                            cast_array<int16_t>(command.motor_pos_ref_rad),
                            cast_array<int16_t>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::UInt16: {
                        const OutputBundle<float, uint16_t> payload {
                            convert_command<float>(command, impl_->config.command_source),
                            cast_array<uint16_t>(command.motor_pos_ref_rad),
                            cast_array<uint16_t>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::Int32:
                    case PlcType::UInt32:
                        break;
                }
                break;
            }
            case PlcType::LReal: {
                switch (impl_->config.reference_plc_type) {
                    case PlcType::Real: {
                        const OutputBundle<double, float> payload {
                            convert_command<double>(command, impl_->config.command_source),
                            cast_array<float>(command.motor_pos_ref_rad),
                            cast_array<float>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::LReal: {
                        const OutputBundle<double, double> payload {
                            convert_command<double>(command, impl_->config.command_source),
                            command.motor_pos_ref_rad,
                            command.motor_vel_ref_rad,
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::Int16: {
                        const OutputBundle<double, int16_t> payload {
                            convert_command<double>(command, impl_->config.command_source),
                            cast_array<int16_t>(command.motor_pos_ref_rad),
                            cast_array<int16_t>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::UInt16: {
                        const OutputBundle<double, uint16_t> payload {
                            convert_command<double>(command, impl_->config.command_source),
                            cast_array<uint16_t>(command.motor_pos_ref_rad),
                            cast_array<uint16_t>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::Int32:
                    case PlcType::UInt32:
                        break;
                }
                break;
            }
            case PlcType::Int16: {
                switch (impl_->config.reference_plc_type) {
                    case PlcType::Real: {
                        const OutputBundle<int16_t, float> payload {
                            convert_command<int16_t>(command, impl_->config.command_source),
                            cast_array<float>(command.motor_pos_ref_rad),
                            cast_array<float>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::LReal: {
                        const OutputBundle<int16_t, double> payload {
                            convert_command<int16_t>(command, impl_->config.command_source),
                            command.motor_pos_ref_rad,
                            command.motor_vel_ref_rad,
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::Int16: {
                        const OutputBundle<int16_t, int16_t> payload {
                            convert_command<int16_t>(command, impl_->config.command_source),
                            cast_array<int16_t>(command.motor_pos_ref_rad),
                            cast_array<int16_t>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::UInt16: {
                        const OutputBundle<int16_t, uint16_t> payload {
                            convert_command<int16_t>(command, impl_->config.command_source),
                            cast_array<uint16_t>(command.motor_pos_ref_rad),
                            cast_array<uint16_t>(command.motor_vel_ref_rad),
                        };
                        write_blob(impl_->device, *impl_->bundle_handle, payload);
                        return;
                    }
                    case PlcType::Int32:
                    case PlcType::UInt32:
                        break;
                }
                break;
            }
            case PlcType::Int32:
            case PlcType::UInt32:
                break;
        }
    }

    // 토크/iq 명령 심볼과 ref 심볼은 PLC 타입이 다를 수 있다.
    // 예: torqueCmd = REAL, posRef/velRef = LREAL
    if (!impl_->config.motor_cmd_symbols.empty()) {
        switch (impl_->config.command_plc_type) {
            case PlcType::Real: {
                write_values<float>(impl_->device, impl_->config.motor_cmd_symbols, impl_->cmd_handles,
                                    convert_command<float>(command, impl_->config.command_source));
                break;
            }
            case PlcType::LReal: {
                write_values<double>(impl_->device, impl_->config.motor_cmd_symbols, impl_->cmd_handles,
                                     convert_command<double>(command, impl_->config.command_source));
                break;
            }
            case PlcType::Int16: {
                write_values<int16_t>(impl_->device, impl_->config.motor_cmd_symbols, impl_->cmd_handles,
                                      convert_command<int16_t>(command, impl_->config.command_source));
                break;
            }
            case PlcType::UInt16: {
                write_values<uint16_t>(impl_->device, impl_->config.motor_cmd_symbols, impl_->cmd_handles,
                                       convert_command<uint16_t>(command, impl_->config.command_source));
                break;
            }
            case PlcType::Int32: {
                write_values<int32_t>(impl_->device, impl_->config.motor_cmd_symbols, impl_->cmd_handles,
                                      convert_command<int32_t>(command, impl_->config.command_source));
                break;
            }
            case PlcType::UInt32: {
                write_values<uint32_t>(impl_->device, impl_->config.motor_cmd_symbols, impl_->cmd_handles,
                                       convert_command<uint32_t>(command, impl_->config.command_source));
                break;
            }
        }
    }

    switch (impl_->config.reference_plc_type) {
        case PlcType::Real: {
            if (!impl_->config.motor_pos_ref_symbols.empty()) {
                write_values<float>(impl_->device, impl_->config.motor_pos_ref_symbols,
                                    impl_->pos_ref_handles,
                                    cast_array<float>(command.motor_pos_ref_rad));
            }
            if (!impl_->config.motor_vel_ref_symbols.empty()) {
                write_values<float>(impl_->device, impl_->config.motor_vel_ref_symbols,
                                    impl_->vel_ref_handles,
                                    cast_array<float>(command.motor_vel_ref_rad));
            }
            break;
        }
        case PlcType::LReal: {
            if (!impl_->config.motor_pos_ref_symbols.empty()) {
                write_values<double>(impl_->device, impl_->config.motor_pos_ref_symbols,
                                     impl_->pos_ref_handles, command.motor_pos_ref_rad);
            }
            if (!impl_->config.motor_vel_ref_symbols.empty()) {
                write_values<double>(impl_->device, impl_->config.motor_vel_ref_symbols,
                                     impl_->vel_ref_handles, command.motor_vel_ref_rad);
            }
            break;
        }
        case PlcType::Int16: {
            if (!impl_->config.motor_pos_ref_symbols.empty()) {
                write_values<int16_t>(impl_->device, impl_->config.motor_pos_ref_symbols,
                                      impl_->pos_ref_handles,
                                      cast_array<int16_t>(command.motor_pos_ref_rad));
            }
            if (!impl_->config.motor_vel_ref_symbols.empty()) {
                write_values<int16_t>(impl_->device, impl_->config.motor_vel_ref_symbols,
                                      impl_->vel_ref_handles,
                                      cast_array<int16_t>(command.motor_vel_ref_rad));
            }
            break;
        }
        case PlcType::UInt16: {
            if (!impl_->config.motor_pos_ref_symbols.empty()) {
                write_values<uint16_t>(impl_->device, impl_->config.motor_pos_ref_symbols,
                                       impl_->pos_ref_handles,
                                       cast_array<uint16_t>(command.motor_pos_ref_rad));
            }
            if (!impl_->config.motor_vel_ref_symbols.empty()) {
                write_values<uint16_t>(impl_->device, impl_->config.motor_vel_ref_symbols,
                                       impl_->vel_ref_handles,
                                       cast_array<uint16_t>(command.motor_vel_ref_rad));
            }
            break;
        }
        case PlcType::Int32: {
            if (!impl_->config.motor_pos_ref_symbols.empty()) {
                write_values<int32_t>(impl_->device, impl_->config.motor_pos_ref_symbols,
                                      impl_->pos_ref_handles,
                                      cast_array<int32_t>(command.motor_pos_ref_rad));
            }
            if (!impl_->config.motor_vel_ref_symbols.empty()) {
                write_values<int32_t>(impl_->device, impl_->config.motor_vel_ref_symbols,
                                      impl_->vel_ref_handles,
                                      cast_array<int32_t>(command.motor_vel_ref_rad));
            }
            break;
        }
        case PlcType::UInt32: {
            if (!impl_->config.motor_pos_ref_symbols.empty()) {
                write_values<uint32_t>(impl_->device, impl_->config.motor_pos_ref_symbols,
                                       impl_->pos_ref_handles,
                                       cast_array<uint32_t>(command.motor_pos_ref_rad));
            }
            if (!impl_->config.motor_vel_ref_symbols.empty()) {
                write_values<uint32_t>(impl_->device, impl_->config.motor_vel_ref_symbols,
                                       impl_->vel_ref_handles,
                                       cast_array<uint32_t>(command.motor_vel_ref_rad));
            }
            break;
        }
    }
}

void AdsMotorIo::WriteZero() const {
    WriteCommand(Command{});
}

std::string AdsMotorIo::describe_feedback_targets() const {
    std::ostringstream out;
    if (!impl_->status_bundle_handles.empty()) {
        out << "status=[" << impl_->config.motor_status_bundle_symbol << "]";
        return out.str();
    }
    out << "pos=[" << impl_->config.motor_pos_symbols.describe() << "]";
    if (!impl_->config.motor_vel_symbols.empty()) {
        out << " vel=[" << impl_->config.motor_vel_symbols.describe() << "]";
    }
    return out.str();
}

std::string AdsMotorIo::describe_command_targets() const {
    std::ostringstream out;
    bool wrote_any = false;
    if (!impl_->command_bundle_handles.empty()) {
        out << "cmd_bundle=[" << impl_->config.motor_command_bundle_symbol << "]";
        wrote_any = true;
    } else if (impl_->bundle_handle) {
        out << "bundle=[" << impl_->config.motor_output_bundle_symbol << "]";
        wrote_any = true;
    } else if (!impl_->config.motor_cmd_symbols.empty()) {
        out << "cmd=[" << impl_->config.motor_cmd_symbols.describe() << "]";
        wrote_any = true;
    }
    if (!impl_->config.motor_pos_ref_symbols.empty()) {
        out << (wrote_any ? " " : "") << "pos_ref=[" << impl_->config.motor_pos_ref_symbols.describe() << "]";
        wrote_any = true;
    }
    if (!impl_->config.motor_vel_ref_symbols.empty()) {
        out << (wrote_any ? " " : "") << "vel_ref=[" << impl_->config.motor_vel_ref_symbols.describe() << "]";
        wrote_any = true;
    }
    if (!impl_->config.motor_enable_symbol.empty()) {
        out << (wrote_any ? " " : "") << "enable=[" << impl_->config.motor_enable_symbol << "]";
        wrote_any = true;
    }
    if (!impl_->config.motor_kp_symbol.empty()) {
        out << (wrote_any ? " " : "") << "kp=[" << impl_->config.motor_kp_symbol << "]";
        wrote_any = true;
    }
    if (!impl_->config.motor_kd_symbol.empty()) {
        out << (wrote_any ? " " : "") << "kd=[" << impl_->config.motor_kd_symbol << "]";
    }
    return out.str();
}

const char* to_string(const PlcType type) {
    switch (type) {
        case PlcType::Real: return "real";
        case PlcType::LReal: return "lreal";
        case PlcType::Int16: return "int16";
        case PlcType::UInt16: return "uint16";
        case PlcType::Int32: return "int32";
        case PlcType::UInt32: return "uint32";
    }
    return "unknown";
}

const char* to_string(const AngleUnit unit) {
    switch (unit) {
        case AngleUnit::Rad: return "rad";
        case AngleUnit::Deg: return "deg";
    }
    return "unknown";
}

const char* to_string(const CommandSource source) {
    switch (source) {
        case CommandSource::Torque: return "torque";
        case CommandSource::Iq: return "iq";
    }
    return "unknown";
}

}  // namespace ads_io
