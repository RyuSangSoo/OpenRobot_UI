#include "ui_ads_ver/ads_state_ui.hpp"

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl2.h"

#include <GLFW/glfw3.h>

#ifndef UI_ADS_VER_HAVE_MUJOCO
#define UI_ADS_VER_HAVE_MUJOCO 0
#endif

#if UI_ADS_VER_HAVE_MUJOCO
#include <mujoco/mujoco.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ui_ads_ver {
namespace {

constexpr size_t kPlotCapacity = 500;
constexpr size_t kMjuPhaseStride = 85;
constexpr size_t kSimTipPosOffset = 64;
constexpr size_t kRefTipPosOffset = kSimTipPosOffset + kMjuPhaseStride;
constexpr size_t kTestConfigFieldCount = 5;
constexpr double kUiRateHz = 30.0;
constexpr char kWindowTitle[] = "OpenRobot Motor Monitor";
const std::array<std::string, kJointCount> kJointNames = {"joint1", "joint2", "joint3", "joint4"};
const std::array<std::string, kJointCount> kMotorNames = {"motor1", "motor2", "motor3", "motor4"};
const std::array<const char*, kJointCount + 1> kTestJointLabels = {"motor1", "motor2", "motor3", "motor4", "all"};

struct UiFonts {
    ImFont* body = nullptr;
    ImFont* heading = nullptr;
    ImFont* mono = nullptr;
};

double RadToDeg(const double radians) {
    return radians * 180.0 / M_PI;
}

double FrequencyToPeriodSeconds(const double frequency_hz) {
    return frequency_hz > 1e-9 ? (1.0 / frequency_hz) : 0.0;
}

const char* ModeName(const int mode) {
    switch (mode) {
        case 0: return "초기화";
        case 1: return "준비";
        case 2: return "테스트";
        case 3: return "스탠드";
        case 4: return "워킹";
        case 5: return "수영 준비";
        case 6: return "수영";
        default: return "알 수 없음";
    }
}

ImU32 JointColor(const size_t index) {
    static const ImU32 colors[] = {
        IM_COL32(240, 80, 80, 255),
        IM_COL32(80, 160, 240, 255),
        IM_COL32(250, 180, 60, 255),
        IM_COL32(120, 220, 220, 255),
    };
    return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

struct PlotBuffer {
    std::vector<float> time_s;
    std::array<std::vector<float>, kJointCount> ref_deg;
    std::array<std::vector<float>, kJointCount> sim_deg;
    size_t head = 0;
    bool full = false;

    PlotBuffer() : time_s(kPlotCapacity, 0.0f) {
        for (size_t i = 0; i < kJointCount; ++i) {
            ref_deg[i].assign(kPlotCapacity, 0.0f);
            sim_deg[i].assign(kPlotCapacity, 0.0f);
        }
    }

    void Push(const float time_value,
              const std::array<double, kJointCount>& ref_value,
              const std::array<double, kJointCount>& sim_value) {
        time_s[head] = time_value;
        for (size_t i = 0; i < kJointCount; ++i) {
            ref_deg[i][head] = static_cast<float>(ref_value[i]);
            sim_deg[i][head] = static_cast<float>(sim_value[i]);
        }
        head = (head + 1) % time_s.size();
        if (head == 0) {
            full = true;
        }
    }

    size_t Size() const {
        return full ? time_s.size() : head;
    }
};

struct MujocoCanvasRequest {
    bool visible = false;
    ImVec2 min {};
    ImVec2 max {};
};

struct MujocoViewerState {
#if UI_ADS_VER_HAVE_MUJOCO
    mjModel* model = nullptr;
    mjData* data = nullptr;
    mjvCamera camera {};
    mjvOption option {};
    mjvScene scene {};
    mjrContext context {};
    std::array<int, kJointCount> qpos_address {-1, -1, -1, -1};
#endif
    std::array<char, 512> model_path {};
    std::string loaded_path;
    std::string status = "MuJoCo viewer is idle";
    std::string error;
    bool loaded = false;
    bool follow_joint_sim = false;
    bool show_geom_frame = false;
};

std::string FindFirstExistingPath(const std::vector<std::string>& candidates);

std::string DefaultMujocoModelPath() {
    const std::filesystem::path source_file(__FILE__);
    const std::filesystem::path workspace_src = source_file.parent_path().parent_path().parent_path();
    return FindFirstExistingPath({
        (workspace_src / "ui_ads_ver" / "models" / "scene.xml").string(),
        (workspace_src / "ADS" / "model" / "mujoco" / "openrobot_monitor.xml").string(),
        "/home/drcl-rss/.mujoco/mujoco-3.5.0/model/tendon_arm/arm26.xml",
    });
}

void CopyToCharBuffer(const std::string& value, std::array<char, 512>& buffer) {
    std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
}

MujocoViewerState CreateMujocoViewerState() {
    MujocoViewerState state;
    CopyToCharBuffer(DefaultMujocoModelPath(), state.model_path);
    return state;
}

#if UI_ADS_VER_HAVE_MUJOCO
void FreeMujocoModel(MujocoViewerState& state) {
    if (state.loaded) {
        mjv_freeScene(&state.scene);
        mjr_freeContext(&state.context);
    }
    if (state.data != nullptr) {
        mj_deleteData(state.data);
        state.data = nullptr;
    }
    if (state.model != nullptr) {
        mj_deleteModel(state.model);
        state.model = nullptr;
    }
    state.qpos_address.fill(-1);
    state.loaded = false;
}

void ResetMujocoCamera(MujocoViewerState& state) {
    if (state.model == nullptr) {
        return;
    }

    mjv_defaultCamera(&state.camera);
    state.camera.type = mjCAMERA_FREE;
    state.camera.lookat[0] = state.model->stat.center[0];
    state.camera.lookat[1] = state.model->stat.center[1];
    state.camera.lookat[2] = state.model->stat.center[2];
    state.camera.distance = std::max(1.4, 2.4 * static_cast<double>(state.model->stat.extent));
    state.camera.azimuth = 122.0;
    state.camera.elevation = -18.0;
}

void CacheMujocoJointAddress(MujocoViewerState& state) {
    state.qpos_address.fill(-1);
    if (state.model == nullptr) {
        return;
    }

    std::array<bool, kJointCount> assigned {};
    const std::array<std::array<const char*, 2>, kJointCount> preferred_joint_names {{
        {{"joint1", "LEG1_Joint"}},
        {{"joint2", "LEG2_Joint"}},
        {{"joint3", "LEG3_Joint"}},
        {{"joint4", "LEG4_Joint"}},
    }};
    for (size_t i = 0; i < kJointCount; ++i) {
        for (const char* joint_name : preferred_joint_names[i]) {
            const int joint_id = mj_name2id(state.model, mjOBJ_JOINT, joint_name);
            if (joint_id >= 0) {
                state.qpos_address[i] = state.model->jnt_qposadr[joint_id];
                assigned[i] = true;
                break;
            }
        }
    }

    int fallback_joint = 0;
    for (size_t i = 0; i < kJointCount; ++i) {
        if (assigned[i]) {
            continue;
        }
        while (fallback_joint < state.model->njnt) {
            const mjtJoint joint_type = static_cast<mjtJoint>(state.model->jnt_type[fallback_joint]);
            if (joint_type == mjJNT_HINGE || joint_type == mjJNT_SLIDE) {
                state.qpos_address[i] = state.model->jnt_qposadr[fallback_joint++];
                break;
            }
            ++fallback_joint;
        }
    }
}

bool LoadMujocoModel(MujocoViewerState& state, const std::string& model_path) {
    if (model_path.empty()) {
        state.loaded = false;
        state.error = "Model path is empty";
        state.status = "MuJoCo load failed";
        return false;
    }

    char error_buffer[1000] = "";
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error_buffer, sizeof(error_buffer));
    if (model == nullptr) {
        state.loaded = false;
        state.error = error_buffer;
        state.status = "MuJoCo load failed";
        return false;
    }

    mjData* data = mj_makeData(model);
    if (data == nullptr) {
        mj_deleteModel(model);
        state.loaded = false;
        state.error = "mj_makeData failed";
        state.status = "MuJoCo load failed";
        return false;
    }

    FreeMujocoModel(state);

    state.model = model;
    state.data = data;
    mjv_defaultOption(&state.option);
    mjv_defaultScene(&state.scene);
    mjr_defaultContext(&state.context);
    mjv_makeScene(state.model, &state.scene, 4000);
    mjr_makeContext(state.model, &state.context, mjFONTSCALE_150);
    mjr_setBuffer(mjFB_WINDOW, &state.context);
    CacheMujocoJointAddress(state);
    ResetMujocoCamera(state);

    state.loaded = true;
    state.loaded_path = model_path;
    state.status = "MuJoCo model loaded";
    state.error.clear();
    return true;
}

void SyncMujocoPoseFromSnapshot(MujocoViewerState& state, const UiSnapshot& snapshot) {
    if (!state.loaded || state.model == nullptr || state.data == nullptr) {
        return;
    }

    const std::array<double, kJointCount>* source_deg = nullptr;
    if (state.follow_joint_sim && snapshot.sim.received) {
        source_deg = &snapshot.sim.position_deg;
    } else if (!state.follow_joint_sim && snapshot.ref.received) {
        source_deg = &snapshot.ref.position_deg;
    } else if (snapshot.sim.received) {
        source_deg = &snapshot.sim.position_deg;
    } else if (snapshot.ref.received) {
        source_deg = &snapshot.ref.position_deg;
    }

    if (source_deg == nullptr) {
        return;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
        const int qpos_address = state.qpos_address[i];
        if (qpos_address < 0 || qpos_address >= state.model->nq) {
            continue;
        }
        state.data->qpos[qpos_address] = (*source_deg)[i] * M_PI / 180.0;
    }
    mj_forward(state.model, state.data);
}

void HandleMujocoCameraInput(MujocoViewerState& state,
                             const bool hovered,
                             const ImVec2& canvas_size) {
    if (!state.loaded || state.model == nullptr || !hovered || canvas_size.y <= 1.0f) {
        return;
    }

    ImGuiIO& io = ImGui::GetIO();
    if (std::fabs(io.MouseWheel) > 1e-6f) {
        mjv_moveCamera(state.model, mjMOUSE_ZOOM, 0.0, -0.05 * io.MouseWheel, &state.scene, &state.camera);
    }

    const double dx = static_cast<double>(io.MouseDelta.x);
    const double dy = static_cast<double>(io.MouseDelta.y);
    if (std::abs(dx) < 1e-9 && std::abs(dy) < 1e-9) {
        return;
    }

    mjtMouse action = mjMOUSE_ZOOM;
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        action = io.KeyShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else if (ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
        action = io.KeyShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (!ImGui::IsMouseDown(ImGuiMouseButton_Middle)) {
        return;
    }

    mjv_moveCamera(state.model, action, dx / canvas_size.y, dy / canvas_size.y, &state.scene, &state.camera);
}

void RenderMujocoCanvas(MujocoViewerState& state,
                        const UiSnapshot& snapshot,
                        const MujocoCanvasRequest& canvas_request,
                        const int framebuffer_height,
                        const ImVec2& framebuffer_scale) {
    if (!state.loaded || state.model == nullptr || state.data == nullptr || !canvas_request.visible) {
        return;
    }

    const float scale_x = framebuffer_scale.x > 0.0f ? framebuffer_scale.x : 1.0f;
    const float scale_y = framebuffer_scale.y > 0.0f ? framebuffer_scale.y : 1.0f;
    const int left = static_cast<int>(std::round(canvas_request.min.x * scale_x));
    const int top = static_cast<int>(std::round(canvas_request.min.y * scale_y));
    const int width = std::max(1, static_cast<int>(std::round((canvas_request.max.x - canvas_request.min.x) * scale_x)));
    const int height = std::max(1, static_cast<int>(std::round((canvas_request.max.y - canvas_request.min.y) * scale_y)));
    const int bottom = framebuffer_height - (top + height);
    if (height <= 0 || width <= 0 || bottom < 0) {
        return;
    }

    SyncMujocoPoseFromSnapshot(state, snapshot);
    state.option.frame = state.show_geom_frame ? mjFRAME_GEOM : mjFRAME_NONE;

    const mjrRect viewport {left, bottom, width, height};
    mjr_setBuffer(mjFB_WINDOW, &state.context);
    mjv_updateScene(state.model, state.data, &state.option, nullptr, &state.camera, mjCAT_ALL, &state.scene);
    mjr_render(viewport, &state.scene, &state.context);
}
#else
void FreeMujocoModel(MujocoViewerState&) {}
void ResetMujocoCamera(MujocoViewerState&) {}
bool LoadMujocoModel(MujocoViewerState& state, const std::string&) {
    state.loaded = false;
    state.status = "MuJoCo support is not compiled in";
    state.error = "Rebuild ui_ads_ver with MuJoCo installed";
    return false;
}
void HandleMujocoCameraInput(MujocoViewerState&, bool, const ImVec2&) {}
void RenderMujocoCanvas(MujocoViewerState&,
                        const UiSnapshot&,
                        const MujocoCanvasRequest&,
                        int,
                        const ImVec2&) {}
#endif

void ApplyImGuiStyle() {
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark(&style);

    style.WindowRounding = 10.0f;
    style.ChildRounding = 10.0f;
    style.FrameRounding = 7.0f;
    style.GrabRounding = 7.0f;
    style.ScrollbarRounding = 8.0f;
    style.TabRounding = 7.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.WindowPadding = ImVec2(18.0f, 18.0f);
    style.FramePadding = ImVec2(10.0f, 7.0f);
    style.CellPadding = ImVec2(8.0f, 8.0f);
    style.ItemSpacing = ImVec2(12.0f, 10.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);
    style.IndentSpacing = 18.0f;

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.08f, 0.11f, 1.00f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.09f, 0.11f, 0.15f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.12f, 0.16f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.24f, 0.29f, 0.36f, 0.70f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.10f, 0.13f, 0.18f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.16f, 0.21f, 0.28f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.18f, 0.23f, 0.31f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.08f, 0.10f, 0.13f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.15f, 0.20f, 0.28f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.22f, 0.29f, 0.39f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.18f, 0.25f, 0.34f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.28f, 0.35f, 0.45f, 0.65f);
    colors[ImGuiCol_Text] = ImVec4(0.92f, 0.95f, 0.98f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.56f, 0.62f, 0.70f, 1.00f);
    colors[ImGuiCol_TableBorderStrong] = ImVec4(0.22f, 0.28f, 0.36f, 1.00f);
    colors[ImGuiCol_TableBorderLight] = ImVec4(0.16f, 0.20f, 0.27f, 1.00f);
    colors[ImGuiCol_TableRowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.08f, 0.10f, 0.14f, 0.45f);
}

std::string FindFirstExistingPath(const std::vector<std::string>& candidates) {
    for (const auto& path : candidates) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }
    return {};
}

UiFonts LoadFonts() {
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();

    const std::string regular_font = FindFirstExistingPath({
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    });
    const std::string medium_font = FindFirstExistingPath({
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    });
    const std::string mono_font = FindFirstExistingPath({
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/home/drcl-rss/.local/share/fonts/DejaVu Sans Mono for Powerline.ttf",
    });

    ImFontConfig config;
    config.OversampleH = 3;
    config.OversampleV = 2;
    config.PixelSnapH = false;
    const ImWchar* korean_ranges = io.Fonts->GetGlyphRangesKorean();

    UiFonts fonts;
    if (!regular_font.empty()) {
        fonts.body = io.Fonts->AddFontFromFileTTF(regular_font.c_str(), 20.0f, &config, korean_ranges);
    }
    if (!medium_font.empty()) {
        fonts.heading = io.Fonts->AddFontFromFileTTF(medium_font.c_str(), 26.0f, &config, korean_ranges);
    }
    if (!mono_font.empty()) {
        fonts.mono = io.Fonts->AddFontFromFileTTF(mono_font.c_str(), 18.0f, &config, korean_ranges);
    }

    if (fonts.body == nullptr) {
        fonts.body = io.Fonts->AddFontDefault();
    }
    if (fonts.heading == nullptr) {
        fonts.heading = fonts.body;
    }
    if (fonts.mono == nullptr) {
        fonts.mono = fonts.body;
    }

    io.FontDefault = fonts.body;
    return fonts;
}

void DrawSingleJointPlot(const char* label,
                         const char* canvas_id,
                         const PlotBuffer& buffer,
                         const size_t joint_index,
                         const ImU32 joint_color,
                         const ImVec2 size) {
    const size_t n = buffer.Size();
    ImGui::TextUnformatted(label);
    ImGui::InvisibleButton(canvas_id, size);
    const ImVec2 p0 = ImGui::GetItemRectMin();
    const ImVec2 p1 = ImGui::GetItemRectMax();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    const float left_margin = 60.0f;
    const float top_margin = 30.0f;
    const float right_margin = 10.0f;
    const float bottom_margin = 14.0f;
    const ImVec2 plot_p0(p0.x + left_margin, p0.y + top_margin);
    const ImVec2 plot_p1(p1.x - right_margin, p1.y - bottom_margin);

    draw_list->AddRectFilled(p0, p1, IM_COL32(22, 22, 26, 255), 4.0f);
    draw_list->AddRect(p0, p1, IM_COL32(80, 80, 90, 255), 4.0f);
    if (n < 2) {
        draw_list->AddText(ImVec2(p0.x + 8.0f, p0.y + 8.0f), IM_COL32(160, 160, 170, 255),
                           "waiting for topic data");
        return;
    }

    if (plot_p1.x <= plot_p0.x || plot_p1.y <= plot_p0.y) {
        return;
    }

    auto IndexAt = [&](const size_t i) -> size_t {
        if (!buffer.full) {
            return i;
        }
        return (buffer.head + i) % buffer.time_s.size();
    };

    float min_time = buffer.time_s[IndexAt(0)];
    float max_time = buffer.time_s[IndexAt(n - 1)];
    if (max_time <= min_time) {
        max_time = min_time + 1e-3f;
    }

    float min_y = buffer.ref_deg[joint_index][IndexAt(0)];
    float max_y = min_y;
    for (size_t i = 0; i < n; ++i) {
        const size_t index = IndexAt(i);
        min_y = std::min(min_y, buffer.ref_deg[joint_index][index]);
        max_y = std::max(max_y, buffer.ref_deg[joint_index][index]);
        min_y = std::min(min_y, buffer.sim_deg[joint_index][index]);
        max_y = std::max(max_y, buffer.sim_deg[joint_index][index]);
    }
    if (max_y <= min_y) {
        min_y -= 1.0f;
        max_y += 1.0f;
    }
    const float padding = 0.1f * (max_y - min_y);
    min_y -= padding;
    max_y += padding;

    auto MapX = [&](const float ts) {
        return plot_p0.x + (ts - min_time) / (max_time - min_time) * (plot_p1.x - plot_p0.x);
    };
    auto MapY = [&](const float value) {
        return plot_p1.y - (value - min_y) / (max_y - min_y) * (plot_p1.y - plot_p0.y);
    };

    draw_list->AddRectFilled(plot_p0, plot_p1, IM_COL32(16, 18, 24, 255), 4.0f);
    draw_list->AddRect(plot_p0, plot_p1, IM_COL32(70, 76, 90, 255), 4.0f);

    constexpr int kTickCount = 5;
    for (int tick = 0; tick < kTickCount; ++tick) {
        const float t = static_cast<float>(tick) / static_cast<float>(kTickCount - 1);
        const float y = plot_p0.y + (plot_p1.y - plot_p0.y) * t;
        draw_list->AddLine(ImVec2(plot_p0.x, y), ImVec2(plot_p1.x, y), IM_COL32(50, 50, 58, 255), 1.0f);

        const double tick_value = max_y - (max_y - min_y) * static_cast<double>(t);
        char tick_label[32];
        std::snprintf(tick_label, sizeof(tick_label), "%.1f", tick_value);
        draw_list->AddText(ImVec2(p0.x + 6.0f, y - 8.0f), IM_COL32(170, 178, 190, 255), tick_label);
    }

    for (size_t i = 1; i < n; ++i) {
        const size_t prev = IndexAt(i - 1);
        const size_t curr = IndexAt(i);
        draw_list->AddLine(
            ImVec2(MapX(buffer.time_s[prev]), MapY(buffer.ref_deg[joint_index][prev])),
            ImVec2(MapX(buffer.time_s[curr]), MapY(buffer.ref_deg[joint_index][curr])),
            IM_COL32(80, 200, 120, 255), 2.0f);
        draw_list->AddLine(
            ImVec2(MapX(buffer.time_s[prev]), MapY(buffer.sim_deg[joint_index][prev])),
            ImVec2(MapX(buffer.time_s[curr]), MapY(buffer.sim_deg[joint_index][curr])),
            joint_color, 2.0f);
    }

    float legend_x = plot_p0.x;
    draw_list->AddText(ImVec2(legend_x, p0.y + 8.0f), IM_COL32(80, 200, 120, 255), "ref");
    legend_x += 42.0f;
    draw_list->AddText(ImVec2(legend_x, p0.y + 8.0f), joint_color, "actual");
}

void DrawLatestJointBlock(const char* title,
                          const std::array<double, kJointCount>& ref_values,
                          const std::array<double, kJointCount>& sim_values,
                          const char* unit,
                          const std::array<std::string, kJointCount>& item_names = kJointNames) {
    const std::string title_text = title;
    const bool show_error =
        title_text.find("Position") != std::string::npos;
    ImGui::SeparatorText(title);
    for (size_t i = 0; i < kJointCount; ++i) {
        ImGui::TextColored(ImColor(JointColor(i)), "%s", item_names[i].c_str());
        ImGui::SameLine(90.0f);
        ImGui::Text("ref=%7.2f %s", ref_values[i], unit);
        ImGui::SameLine(250.0f);
        ImGui::Text("act=%7.2f %s", sim_values[i], unit);
        if (show_error) {
            ImGui::SameLine(410.0f);
            ImGui::Text("err=%7.2f %s", ref_values[i] - sim_values[i], unit);
        }
    }
}

void DrawTipPositionBlock(const std::vector<double>& mju_state) {
    if (mju_state.size() < (kRefTipPosOffset + 3)) {
        return;
    }
    ImGui::SeparatorText("Tip Position");
    ImGui::Text("sim: x=% .3f  y=% .3f  z=% .3f",
                mju_state[kSimTipPosOffset + 0],
                mju_state[kSimTipPosOffset + 1],
                mju_state[kSimTipPosOffset + 2]);
    ImGui::Text("ref: x=% .3f  y=% .3f  z=% .3f",
                mju_state[kRefTipPosOffset + 0],
                mju_state[kRefTipPosOffset + 1],
                mju_state[kRefTipPosOffset + 2]);
}

void LoadUiTestConfigFromSnapshot(const UiSnapshot& snapshot,
                                  int& ui_joint_index,
                                  float& ui_amplitude_rad,
                                  float& ui_period_s) {
    ui_joint_index = std::clamp(snapshot.test_config.joint_index, 0, static_cast<int>(kJointCount));
    ui_amplitude_rad = static_cast<float>(std::max(0.0, snapshot.test_config.amplitude_rad));
    ui_period_s = static_cast<float>(FrequencyToPeriodSeconds(snapshot.test_config.frequency_hz));
}

}  // namespace

AdsStateUiNode::AdsStateUiNode()
    : rclcpp::Node("ads_state_ui") {
    sim_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    ref_topic_ = declare_parameter<std::string>("joint_state_ref_topic", "/joint_states_ref");
    motor_sim_topic_ = declare_parameter<std::string>("motor_state_topic", "/motor_states");
    motor_ref_topic_ = declare_parameter<std::string>("motor_state_ref_topic", "/motor_states_ref");
    mju_topic_ = declare_parameter<std::string>("mju_state_topic", "/ads/mju_state");
    test_config_state_topic_ = declare_parameter<std::string>("test_config_state_topic", "/ads/test_config_state");
    test_config_cmd_topic_ = declare_parameter<std::string>("test_config_cmd_topic", "/ads/test_config_cmd");

    sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        sim_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnSimJointState(*msg); });
    ref_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        ref_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnRefJointState(*msg); });
    motor_sim_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        motor_sim_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnMotorSimJointState(*msg); });
    motor_ref_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        motor_ref_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { OnMotorRefJointState(*msg); });
    mju_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        mju_topic_, rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnMjuState(*msg); });
    test_config_state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        test_config_state_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { OnTestConfigState(*msg); });
    test_config_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        test_config_cmd_topic_, rclcpp::QoS(10).reliable());

    RCLCPP_INFO(get_logger(),
                "Listening: joint_sim=%s joint_ref=%s motor_sim=%s motor_ref=%s mju=%s test_state=%s test_cmd=%s",
                sim_topic_.c_str(), ref_topic_.c_str(), motor_sim_topic_.c_str(),
                motor_ref_topic_.c_str(), mju_topic_.c_str(),
                test_config_state_topic_.c_str(), test_config_cmd_topic_.c_str());
}

UiSnapshot AdsStateUiNode::Snapshot() const {
    UiSnapshot snapshot;
    snapshot.sim = sim_data_;
    snapshot.ref = ref_data_;
    snapshot.motor_sim = motor_sim_data_;
    snapshot.motor_ref = motor_ref_data_;
    snapshot.test_config = test_config_;
    snapshot.mju_state = mju_state_;
    snapshot.sim_count = sim_count_;
    snapshot.ref_count = ref_count_;
    snapshot.motor_sim_count = motor_sim_count_;
    snapshot.motor_ref_count = motor_ref_count_;
    snapshot.mju_count = mju_count_;
    snapshot.test_config_count = test_config_count_;
    snapshot.joint_order_warning = joint_order_warning_;
    snapshot.sim_topic = sim_topic_;
    snapshot.ref_topic = ref_topic_;
    snapshot.motor_sim_topic = motor_sim_topic_;
    snapshot.motor_ref_topic = motor_ref_topic_;
    snapshot.mju_topic = mju_topic_;
    snapshot.test_config_state_topic = test_config_state_topic_;
    snapshot.test_config_cmd_topic = test_config_cmd_topic_;
    return snapshot;
}

void AdsStateUiNode::PublishTestConfigCommand(const int joint_index,
                                              const double amplitude_rad,
                                              const double frequency_hz) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        static_cast<double>(std::clamp(joint_index, 0, static_cast<int>(kJointCount))),
        std::max(0.0, amplitude_rad),
        std::max(0.0, frequency_hz),
    };
    test_config_cmd_pub_->publish(msg);
}

void AdsStateUiNode::FillOrderedJointData(const sensor_msgs::msg::JointState& msg,
                                          JointStateData& data,
                                          bool& joint_order_warning,
                                          const std::array<std::string, kJointCount>& expected_names) {
    std::array<int, kJointCount> order {-1, -1, -1, -1};
    bool exact_match = msg.name.size() >= kJointCount;
    for (size_t i = 0; i < kJointCount; ++i) {
        auto it = std::find(msg.name.begin(), msg.name.end(), expected_names[i]);
        if (it == msg.name.end()) {
            exact_match = false;
            continue;
        }
        order[i] = static_cast<int>(std::distance(msg.name.begin(), it));
    }

    if (!exact_match) {
        joint_order_warning = true;
    }

    for (size_t i = 0; i < kJointCount; ++i) {
        if (order[i] < 0) {
            data.position_deg[i] = 0.0;
            data.velocity_deg_s[i] = 0.0;
            data.effort[i] = 0.0;
            continue;
        }

        const size_t index = static_cast<size_t>(order[i]);
        data.position_deg[i] = index < msg.position.size() ? RadToDeg(msg.position[index]) : 0.0;
        data.velocity_deg_s[i] = index < msg.velocity.size() ? RadToDeg(msg.velocity[index]) : 0.0;
        data.effort[i] = index < msg.effort.size() ? msg.effort[index] : 0.0;
    }
    data.received = true;
}

void AdsStateUiNode::OnSimJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, sim_data_, joint_order_warning_, kJointNames);
    ++sim_count_;
}

void AdsStateUiNode::OnRefJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, ref_data_, joint_order_warning_, kJointNames);
    ++ref_count_;
}

void AdsStateUiNode::OnMotorSimJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, motor_sim_data_, joint_order_warning_, kMotorNames);
    ++motor_sim_count_;
}

void AdsStateUiNode::OnMotorRefJointState(const sensor_msgs::msg::JointState& msg) {
    FillOrderedJointData(msg, motor_ref_data_, joint_order_warning_, kMotorNames);
    ++motor_ref_count_;
}

void AdsStateUiNode::OnMjuState(const std_msgs::msg::Float64MultiArray& msg) {
    mju_state_ = msg.data;
    ++mju_count_;
}

void AdsStateUiNode::OnTestConfigState(const std_msgs::msg::Float64MultiArray& msg) {
    if (msg.data.size() < kTestConfigFieldCount) {
        return;
    }

    if (!std::isfinite(msg.data[0]) ||
        !std::isfinite(msg.data[1]) ||
        !std::isfinite(msg.data[2]) ||
        !std::isfinite(msg.data[3]) ||
        !std::isfinite(msg.data[4])) {
        return;
    }

    test_config_.joint_index = std::clamp(static_cast<int>(std::lround(msg.data[0])), 0, static_cast<int>(kJointCount));
    test_config_.amplitude_rad = std::max(0.0, msg.data[1]);
    test_config_.frequency_hz = std::max(0.0, msg.data[2]);
    test_config_.current_mode = static_cast<int>(std::lround(msg.data[3]));
    test_config_.change_counter = static_cast<unsigned long long>(std::llround(std::max(0.0, msg.data[4])));
    test_config_.received = true;
    ++test_config_count_;
}

int RunAdsStateUi(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<AdsStateUiNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    if (!glfwInit()) {
        throw std::runtime_error("glfwInit failed");
    }

    GLFWwindow* window = glfwCreateWindow(1400, 860, kWindowTitle, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        throw std::runtime_error("glfwCreateWindow failed");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ApplyImGuiStyle();
    const UiFonts fonts = LoadFonts();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();

    MujocoViewerState mujoco_viewer = CreateMujocoViewerState();
    if (std::strlen(mujoco_viewer.model_path.data()) > 0) {
        LoadMujocoModel(mujoco_viewer, mujoco_viewer.model_path.data());
    } else {
        mujoco_viewer.status = "No MuJoCo model path was found";
        mujoco_viewer.error = "Set a valid XML or URDF path and press Reload Model";
    }

    PlotBuffer joint_plot_buffer;
    PlotBuffer motor_plot_buffer;
    const auto start_time = std::chrono::steady_clock::now();
    size_t last_plotted_ref_count = 0;
    size_t last_plotted_motor_sim_count = 0;
    size_t last_plotted_motor_ref_count = 0;
    size_t last_plotted_sim_count = 0;
    bool test_config_editor_initialized = false;
    int ui_test_joint_index = 0;
    float ui_test_amplitude_rad = 0.25f;
    float ui_test_period_s = 5.0f;
    rclcpp::WallRate ui_rate(kUiRateHz);

    while (!glfwWindowShouldClose(window) && rclcpp::ok()) {
        MujocoCanvasRequest mujoco_canvas_request;

        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        executor.spin_some();

        const UiSnapshot snapshot = node->Snapshot();
        if (snapshot.test_config.received && !test_config_editor_initialized) {
            LoadUiTestConfigFromSnapshot(
                snapshot, ui_test_joint_index, ui_test_amplitude_rad, ui_test_period_s);
            test_config_editor_initialized = true;
        }
        const bool has_new_sample = snapshot.ref_count != last_plotted_ref_count ||
                                    snapshot.motor_sim_count != last_plotted_motor_sim_count ||
                                    snapshot.motor_ref_count != last_plotted_motor_ref_count ||
                                    snapshot.sim_count != last_plotted_sim_count;
        if (has_new_sample &&
            (snapshot.ref.received || snapshot.sim.received ||
             snapshot.motor_ref.received || snapshot.motor_sim.received)) {
            const float elapsed_s = static_cast<float>(
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());
            joint_plot_buffer.Push(elapsed_s, snapshot.ref.position_deg, snapshot.sim.position_deg);
            motor_plot_buffer.Push(elapsed_s, snapshot.motor_ref.position_deg, snapshot.motor_sim.position_deg);
            last_plotted_ref_count = snapshot.ref_count;
            last_plotted_motor_sim_count = snapshot.motor_sim_count;
            last_plotted_motor_ref_count = snapshot.motor_ref_count;
            last_plotted_sim_count = snapshot.sim_count;
        }

        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize, ImGuiCond_Always);
        const ImGuiWindowFlags window_flags =
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse;

        ImGui::Begin("ADS Joint State Monitor", nullptr, window_flags);
        ImGui::PushFont(fonts.heading);
        ImGui::TextUnformatted(kWindowTitle);
        ImGui::PopFont();
        ImGui::Text("ROS 2 topic subscriber view");
        ImGui::Separator();

        const ImGuiTableFlags table_flags =
            ImGuiTableFlags_Resizable |
            ImGuiTableFlags_BordersInnerV |
            ImGuiTableFlags_SizingStretchProp;

        if (ImGui::BeginTable("main_split_table", 2, table_flags)) {
            ImGui::TableSetupColumn("LeftInfo", ImGuiTableColumnFlags_WidthStretch, 0.38f);
            ImGui::TableSetupColumn("RightPlots", ImGuiTableColumnFlags_WidthStretch, 0.62f);
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::BeginChild("left_info_panel", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
            ImGui::Text("received joint(sim/ref)=%zu/%zu  motor(sim/ref)=%zu/%zu  mju=%zu  test=%zu",
                        snapshot.sim_count, snapshot.ref_count,
                        snapshot.motor_sim_count, snapshot.motor_ref_count,
                        snapshot.mju_count, snapshot.test_config_count);
            ImGui::Text("status joint(sim/ref)=%s/%s  motor(sim/ref)=%s/%s  test=%s",
                        snapshot.sim.received ? "ok" : "waiting",
                        snapshot.ref.received ? "ok" : "waiting",
                        snapshot.motor_sim.received ? "ok" : "waiting",
                        snapshot.motor_ref.received ? "ok" : "waiting",
                        snapshot.test_config.received ? "ok" : "waiting");
            if (snapshot.joint_order_warning) {
                ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.30f, 1.0f),
                                   "warning: joint name mismatch, unmatched joints are set to 0");
            }

            DrawLatestJointBlock("Joint Position", snapshot.ref.position_deg, snapshot.sim.position_deg, "deg");
            DrawLatestJointBlock("Joint Velocity", snapshot.ref.velocity_deg_s, snapshot.sim.velocity_deg_s, "deg/s");
            DrawLatestJointBlock("Joint Effort", snapshot.ref.effort, snapshot.sim.effort, "");
            DrawLatestJointBlock("Motor Position", snapshot.motor_ref.position_deg, snapshot.motor_sim.position_deg, "deg", kMotorNames);
            DrawLatestJointBlock("Motor Velocity", snapshot.motor_ref.velocity_deg_s, snapshot.motor_sim.velocity_deg_s, "deg/s", kMotorNames);
            DrawLatestJointBlock("Motor Effort", snapshot.motor_ref.effort, snapshot.motor_sim.effort, "", kMotorNames);
            DrawTipPositionBlock(snapshot.mju_state);
            ImGui::SeparatorText("테스트 설정 상태");
            ImGui::Text("상태 토픽: %s", snapshot.test_config_state_topic.c_str());
            ImGui::Text("명령 토픽: %s", snapshot.test_config_cmd_topic.c_str());
            if (snapshot.test_config.received) {
                ImGui::Text("현재 모드: %s (%d)",
                            ModeName(snapshot.test_config.current_mode),
                            snapshot.test_config.current_mode);
                ImGui::Text("적용 모터: %s",
                            kTestJointLabels[std::clamp(snapshot.test_config.joint_index, 0, static_cast<int>(kJointCount))]);
                ImGui::Text("적용 진폭: %.3f rad", snapshot.test_config.amplitude_rad);
                ImGui::Text("적용 주기: %.3f s", FrequencyToPeriodSeconds(snapshot.test_config.frequency_hz));
                ImGui::Text("적용 주파수: %.3f Hz", snapshot.test_config.frequency_hz);
                ImGui::Text("변경 카운터: %llu", snapshot.test_config.change_counter);
            } else {
                ImGui::TextDisabled("/ads/test_config_state 수신 대기 중");
            }

            ImGui::SeparatorText("테스트 명령");
            ImGui::Combo("대상 모터", &ui_test_joint_index, kTestJointLabels.data(), static_cast<int>(kTestJointLabels.size()));
            ImGui::SliderFloat("진폭 [rad]", &ui_test_amplitude_rad, 0.0f, 10.0f, "%.3f");
            ImGui::SliderFloat("주기 [s]", &ui_test_period_s, 0.0f, 120.0f, "%.3f");
            ui_test_amplitude_rad = std::max(0.0f, ui_test_amplitude_rad);
            ui_test_period_s = std::max(0.0f, ui_test_period_s);
            const double preview_frequency_hz =
                ui_test_period_s > 1e-6f ? (1.0 / static_cast<double>(ui_test_period_s)) : 0.0;
            ImGui::Text("변환 주파수: %.3f Hz", preview_frequency_hz);
            if (ImGui::Button("테스트 설정 적용")) {
                node->PublishTestConfigCommand(
                    ui_test_joint_index,
                    static_cast<double>(ui_test_amplitude_rad),
                    preview_frequency_hz);
            }
            if (snapshot.test_config.received) {
                ImGui::SameLine();
                if (ImGui::Button("현재 적용값 불러오기")) {
                    LoadUiTestConfigFromSnapshot(
                        snapshot, ui_test_joint_index, ui_test_amplitude_rad, ui_test_period_s);
                }
            }
            ImGui::TextDisabled("이 버튼은 [motor_index, amplitude_rad, frequency_hz]를 ROS bridge로 보냅니다.");
            ImGui::TextDisabled("현재 모드가 테스트가 아니어도 값은 저장되고, 테스트 모드 진입 시 그대로 사용됩니다.");
            ImGui::EndChild();

            ImGui::TableSetColumnIndex(1);
            ImGui::BeginChild("right_plot_panel", ImVec2(0.0f, 0.0f), false, ImGuiWindowFlags_NoScrollbar);
            if (ImGui::BeginTabBar("right_panel_tabs")) {
                if (ImGui::BeginTabItem("Graphs")) {
                    const ImGuiTableFlags plot_split_flags =
                        ImGuiTableFlags_BordersInnerV |
                        ImGuiTableFlags_SizingStretchProp;
                    if (ImGui::BeginTable("plot_split_table", 2, plot_split_flags)) {
                        ImGui::TableSetupColumn("JointPlots", ImGuiTableColumnFlags_WidthStretch, 0.5f);
                        ImGui::TableSetupColumn("MotorPlots", ImGuiTableColumnFlags_WidthStretch, 0.5f);
                        ImGui::TableNextRow();

                        ImGui::TableSetColumnIndex(0);
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.11f, 0.16f, 1.00f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
                        ImGui::BeginChild("joint_plot_group", ImVec2(0.0f, 0.0f), true);
                        ImGui::SeparatorText("Joint Plots");
                        for (size_t i = 0; i < kJointCount; ++i) {
                            const float remaining_h = ImGui::GetContentRegionAvail().y;
                            const float plots_left = static_cast<float>(kJointCount - i);
                            const float label_h = ImGui::GetTextLineHeightWithSpacing();
                            const float spacing_h = ImGui::GetStyle().ItemSpacing.y;
                            const float canvas_h = std::max(
                                90.0f,
                                (remaining_h - plots_left * label_h - (plots_left - 1.0f) * spacing_h) / plots_left);
                            const std::string joint_label = kJointNames[i] + " Joint Ref vs Sim";
                            const std::string joint_canvas_id = "joint_plot_canvas_" + std::to_string(i);
                            DrawSingleJointPlot(joint_label.c_str(), joint_canvas_id.c_str(), joint_plot_buffer, i,
                                                JointColor(i), ImVec2(-1.0f, canvas_h));
                        }
                        ImGui::EndChild();
                        ImGui::PopStyleVar();
                        ImGui::PopStyleColor();

                        ImGui::TableSetColumnIndex(1);
                        ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.11f, 0.16f, 1.00f));
                        ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
                        ImGui::BeginChild("motor_plot_group", ImVec2(0.0f, 0.0f), true);
                        ImGui::SeparatorText("Motor Plots");
                        for (size_t i = 0; i < kJointCount; ++i) {
                            const float remaining_h = ImGui::GetContentRegionAvail().y;
                            const float plots_left = static_cast<float>(kJointCount - i);
                            const float label_h = ImGui::GetTextLineHeightWithSpacing();
                            const float spacing_h = ImGui::GetStyle().ItemSpacing.y;
                            const float canvas_h = std::max(
                                90.0f,
                                (remaining_h - plots_left * label_h - (plots_left - 1.0f) * spacing_h) / plots_left);
                            const std::string motor_label = kMotorNames[i] + " Motor Ref vs Sim";
                            const std::string motor_canvas_id = "motor_plot_canvas_" + std::to_string(i);
                            DrawSingleJointPlot(motor_label.c_str(), motor_canvas_id.c_str(), motor_plot_buffer, i,
                                                JointColor(i), ImVec2(-1.0f, canvas_h));
                        }
                        ImGui::EndChild();
                        ImGui::PopStyleVar();
                        ImGui::PopStyleColor();

                        ImGui::EndTable();
                    }
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Extra")) {
                    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08f, 0.11f, 0.16f, 1.00f));
                    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
                    ImGui::BeginChild("extra_panel_group", ImVec2(0.0f, 0.0f), true);
                    ImGui::SeparatorText("MuJoCo Viewer");
                    ImGui::TextWrapped("Extra tab now hosts an embedded MuJoCo view. Drag left mouse to orbit, drag right mouse to pan, and use wheel to zoom.");
                    ImGui::InputText("Model Path", mujoco_viewer.model_path.data(), mujoco_viewer.model_path.size());
                    if (ImGui::Button("Reload Model")) {
                        LoadMujocoModel(mujoco_viewer, mujoco_viewer.model_path.data());
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Reset Camera")) {
                        ResetMujocoCamera(mujoco_viewer);
                    }
                    int pose_source = mujoco_viewer.follow_joint_sim ? 1 : 0;
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(180.0f);
                    if (ImGui::Combo("Pose Source", &pose_source, "Joint Ref\0Joint Sim\0")) {
                        mujoco_viewer.follow_joint_sim = (pose_source == 1);
                    }
                    ImGui::SameLine();
                    ImGui::Checkbox("Show Geom Frame", &mujoco_viewer.show_geom_frame);

                    ImGui::Text("Status: %s", mujoco_viewer.status.c_str());
                    if (!mujoco_viewer.loaded_path.empty()) {
                        ImGui::Text("Loaded: %s", mujoco_viewer.loaded_path.c_str());
                    }
                    if (!mujoco_viewer.error.empty()) {
                        ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.45f, 1.0f), "Load Error: %s", mujoco_viewer.error.c_str());
                    }
                    ImGui::Text("ROS source: %s", mujoco_viewer.follow_joint_sim ? "joint sim" : "joint ref");
                    ImGui::Text("Topics: ref=%s  sim=%s", snapshot.ref_topic.c_str(), snapshot.sim_topic.c_str());

                    ImGui::Spacing();
                    const ImVec2 available = ImGui::GetContentRegionAvail();
                    const ImVec2 canvas_size(
                        std::max(available.x, 240.0f),
                        std::max(available.y, 320.0f));
                    ImGui::InvisibleButton("mujoco_canvas", canvas_size);
                    const ImVec2 canvas_min = ImGui::GetItemRectMin();
                    const ImVec2 canvas_max = ImGui::GetItemRectMax();
                    ImDrawList* draw_list = ImGui::GetWindowDrawList();
                    draw_list->AddRectFilled(canvas_min, canvas_max, IM_COL32(16, 18, 24, 255), 8.0f);
                    draw_list->AddRect(canvas_min, canvas_max, IM_COL32(84, 98, 120, 255), 8.0f);
                    HandleMujocoCameraInput(mujoco_viewer, ImGui::IsItemHovered(), canvas_size);

                    if (mujoco_viewer.loaded) {
                        mujoco_canvas_request.visible = true;
                        mujoco_canvas_request.min = canvas_min;
                        mujoco_canvas_request.max = canvas_max;
                    } else {
                        draw_list->AddText(ImVec2(canvas_min.x + 18.0f, canvas_min.y + 18.0f),
                                           IM_COL32(205, 214, 226, 255),
                                           "MuJoCo canvas is waiting for a valid model.");
                    }
                    ImGui::EndChild();
                    ImGui::PopStyleVar();
                    ImGui::PopStyleColor();
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }
            ImGui::EndChild();

            ImGui::EndTable();
        }
        ImGui::End();

        ImGui::Render();
        int width = 0;
        int height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        RenderMujocoCanvas(
            mujoco_viewer,
            snapshot,
            mujoco_canvas_request,
            height,
            ImGui::GetIO().DisplayFramebufferScale);
        glfwSwapBuffers(window);
        ui_rate.sleep();
    }

    FreeMujocoModel(mujoco_viewer);
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    executor.remove_node(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}

}  // namespace ui_ads_ver
