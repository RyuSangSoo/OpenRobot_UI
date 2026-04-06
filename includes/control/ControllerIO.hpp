#ifndef CONTROLLER_IO_HPP
#define CONTROLLER_IO_HPP

#include "MjuRobotState.hpp"

// Hardware/simulator adapter boundary for the controller loop.
// Sim builds can use SharedStateIO (no-op), while hardware builds can provide
// an implementation that reads sensors and writes torque commands.
class ControllerIO {
public:
    virtual ~ControllerIO() = default;
    virtual void ReadState(MjuState& state) = 0;
    virtual void WriteCommand(const MjuState& state) = 0;
};

class SharedStateIO : public ControllerIO {
public:
    void ReadState(MjuState& state) override;
    void WriteCommand(const MjuState& state) override;
};

#endif
