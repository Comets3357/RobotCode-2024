#pragma once


#define COMMAND_START(name) \
class name##Command : public frc2::CommandHelper<frc2::CommandBase, name##Command> { \
public: \
    void Initialize() override; \
    void Execute() override; \
    bool IsFinished() override; \
    void End(bool interrupted) override; 

#define COMMAND_END };