#include "MjuJoy.hpp"

namespace {
std::atomic<bool> kbd_w{false};
std::atomic<bool> kbd_s{false};
std::atomic<bool> kbd_a{false};
std::atomic<bool> kbd_d{false};
std::atomic<bool> kbd_q{false};
std::atomic<bool> kbd_e{false};
std::atomic<bool> kbd_i{false};
std::atomic<bool> kbd_k{false};

std::atomic<bool> kbd_j{false};  // BTN_A
std::atomic<bool> kbd_l{false};  // BTN_B
std::atomic<bool> kbd_u{false};  // BTN_X
std::atomic<bool> kbd_y{false};  // BTN_Y
std::atomic<bool> kbd_1{false};  // BTN_LB
std::atomic<bool> kbd_2{false};  // BTN_RB
}  // namespace

MjuJoy::MjuJoy(const std::string& js_path) {
    fd = open(js_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        keyboard_fallback = true;
        std::cerr << "Failed to open joystick at " << js_path
                  << " (keyboard fallback enabled)\n";
        std::cerr << "Keyboard map: W/S=JOYdx, A/D=JOYdyaw, Q/E=JOYroll, I/K=JOYpitch, "
                     "J/L/U/Y=A/B/X/Y, 1/2=LB/RB\n";
    }

    path = js_path;
    axes.fill(0);
    buttons.fill(false);
}

MjuJoy::~MjuJoy() {
    if (fd >= 0) {
        std::cerr << "END JOYSTICK" << std::endl;
        close(fd);
    }
}

bool MjuJoy::update() {
    if (keyboard_fallback) {
        const int axis_step = 32767;

        axes[1] = (kbd_w.load() ? -axis_step : 0) + (kbd_s.load() ? axis_step : 0);
        axes[0] = (kbd_a.load() ? -axis_step : 0) + (kbd_d.load() ? axis_step : 0);
        axes[3] = (kbd_e.load() ? axis_step : 0) + (kbd_q.load() ? -axis_step : 0);
        axes[4] = (kbd_i.load() ? axis_step : 0) + (kbd_k.load() ? -axis_step : 0);

        buttons[0] = kbd_j.load();
        buttons[1] = kbd_l.load();
        buttons[2] = kbd_u.load();
        buttons[3] = kbd_y.load();
        buttons[4] = kbd_1.load();
        buttons[5] = kbd_2.load();

        mapping();
        return true;
    }

    js_event e;
    while (read(fd, &e, sizeof(e)) == sizeof(e)) {
        switch (e.type & ~JS_EVENT_INIT)
        {
        case JS_EVENT_AXIS:
            if (e.number < axes.size()) {
                axes[e.number] = e.value;
            }
            break;
        
        case JS_EVENT_BUTTON:
            if (e.number < buttons.size()) {
                buttons[e.number] = (e.value != 0);
            }
            break;        
        }
    }

    mapping();
    return true;
}

void MjuJoy::mapping() {

    JOYdyaw = (axes[0]/(-32767))*max_dx;
    JOYdx = (axes[1]/(-32767))*max_dx;

    JOYroll = axes[3]/(32767)*max_dx;
    JOYpitch = axes[4]/(32767)*max_dx;

    JOYBtnA = buttons[0];
    JOYBtnB = buttons[1];
    JOYBtnX = buttons[2];
    JOYBtnY = buttons[3];

    JOYBtnLB = buttons[4];
    JOYBtnRB = buttons[5];
}

void SetKeyboardJoystickKey(int key, int act) {
    const bool pressed = (act != kKeyboardRelease);

    switch (key) {
        case kKeyW: kbd_w = pressed; break;
        case kKeyS: kbd_s = pressed; break;
        case kKeyA: kbd_a = pressed; break;
        case kKeyD: kbd_d = pressed; break;
        case kKeyQ: kbd_q = pressed; break;
        case kKeyE: kbd_e = pressed; break;
        case kKeyI: kbd_i = pressed; break;
        case kKeyK: kbd_k = pressed; break;

        case kKeyJ: kbd_j = pressed; break;
        case kKeyL: kbd_l = pressed; break;
        case kKeyU: kbd_u = pressed; break;
        case kKeyY: kbd_y = pressed; break;
        case kKey1: kbd_1 = pressed; break;
        case kKey2: kbd_2 = pressed; break;
        default: break;
    }
}
