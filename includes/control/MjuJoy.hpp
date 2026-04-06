#ifndef MJUJOY_HPP
#define MJUJOY_HPP

#include "header.hpp"
#include "MjuRobotState.hpp"

class MjuJoy
{
private:
    int fd;
    bool keyboard_fallback = false;
    std::array<int, 8> axes{};
    std::array<bool, 11> buttons{};
    std::string path;  // 디버깅용

public:
    MjuJoy(const std::string& js_path);
    ~MjuJoy();

    bool update();
    void mapping();

    double max_dx   = 1.0;
    double JOYdx    = 0.0;
    double JOYdyaw  = 0.0;
    double JOYpitch = 0.0;
    double JOYroll  = 0.0;

    double JOYBtnA  = 0.0;
    double JOYBtnB  = 0.0;
    double JOYBtnX  = 0.0;
    double JOYBtnY  = 0.0;
    double JOYBtnRB = 0.0;
    double JOYBtnLB = 0.0;
    double JoyOut   = 0.0;

};

// GLFW 없이도 keyboard fallback을 유지하기 위해 필요한 최소 키/액션 상수만 로컬로 정의한다.
// 문자 키는 ASCII 값을 그대로 사용하면 대부분의 UI/터미널 입력과 맞는다.
constexpr int kKeyboardRelease = 0;
constexpr int kKeyW = 'W';
constexpr int kKeyS = 'S';
constexpr int kKeyA = 'A';
constexpr int kKeyD = 'D';
constexpr int kKeyQ = 'Q';
constexpr int kKeyE = 'E';
constexpr int kKeyI = 'I';
constexpr int kKeyK = 'K';
constexpr int kKeyJ = 'J';
constexpr int kKeyL = 'L';
constexpr int kKeyU = 'U';
constexpr int kKeyY = 'Y';
constexpr int kKey1 = '1';
constexpr int kKey2 = '2';

// 외부 입력 콜백에서 호출: 키보드 상태를 가상 조이스틱 입력으로 저장한다.
void SetKeyboardJoystickKey(int key, int act);


#endif
