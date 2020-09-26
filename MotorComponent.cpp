#include "MotorComponent.h"

namespace MotorComponent
{

    bool Motor::_initSuccess = false;

    Motor::Motor(int axisChannel) :
        _axisChannel(axisChannel),
        _moveMode(static_cast<enum MoveMode>(MotorControlState::UNDEFINED)),
        _originDetectionMode(static_cast<enum OriginDetectionMode>(MotorControlState::UNDEFINED)),
        _moveSpeed(static_cast<double>(MotorControlState::UNDEFINED)),
        _fastMoveSpeed{static_cast<double>(MotorControlState::UNDEFINED), 
                                            static_cast<double>(MotorControlState::UNDEFINED), 
                                            static_cast<double>(MotorControlState::UNDEFINED)},
        _interpolationMoveSpeed(static_cast<double>(MotorControlState::UNDEFINED)),
        _interpolationFastMoveSpeed{static_cast<double>(MotorControlState::UNDEFINED), 
                                            static_cast<double>(MotorControlState::UNDEFINED), 
                                            static_cast<double>(MotorControlState::UNDEFINED)},
        _maxSpeed(static_cast<double>(MotorControlState::UNDEFINED))
    {

    }

    Motor::~Motor()
    {

    }

    bool Motor::Init()
    {
        if (!_initSuccess)
        {
            if (auto_set() > 0)
            {
                if (init_board() > 0)
                {
                    _initSuccess = true;
                }
            }
        }
        return _initSuccess;
    }

    int Motor::Axis()
    {
        return get_max_axe();
    }

    int Motor::Board()
    {
        return get_board_num();
    }

    MotorControlState Motor::InitDefaultSetting()
    {

    }

    MotorControlState Motor::SetMoveMode(enum MoveMode mode)
    {
        if (_initSuccess)
        {
            if (mode != MoveMode::PULSE_AND_DIRECTION || mode != MoveMode::DOUBLE_PULSE)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_outmode(_axisChannel, static_cast<int>(mode), 1) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::MoveMode(enum MoveMode *mode)
    {
        if (mode == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (static_cast<MotorControlState>(_moveMode) == MotorControlState::UNDEFINED)
        {
            return MotorControlState::UNDEFINED;
        }
        *mode = _moveMode;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetOriginDetectionMode(enum OriginDetectionMode mode)
    {
        if (_initSuccess)
        {
            if (mode != OriginDetectionMode::SWITCH_SIGNAL || mode != OriginDetectionMode::SWITCH_SIGNAL_AND_ENCODER_SIGNAL)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_home_mode(_axisChannel, static_cast<int>(mode)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::OriginDetectionMode(enum OriginDetectionMode *mode)
    {
        if (mode == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (static_cast<MotorControlState>(_originDetectionMode) == MotorControlState::UNDEFINED)
        {
            return MotorControlState::UNDEFINED;
        }
        *mode = _originDetectionMode;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetMoveSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (speed <= 0)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_conspeed(_axisChannel, speed) == 0)
            {
                _moveSpeed = speed;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::MoveSpeed(double* speed)
    {
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_moveSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *speed = _moveSpeed;
        return MotorControlState::DEFINED;
    }
    
    MotorControlState Motor::SetFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (startingSpeed <= 0 || targetSpeed <= 0 || accelerationSpeed <= 0 || startingSpeed >= targetSpeed)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_profile(_axisChannel, startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _fastMoveSpeed.startingSpeed = startingSpeed;
                _fastMoveSpeed.targetSpeed = targetSpeed;
                _fastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetFastMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (ts == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        return SetFastMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    MotorControlState Motor::FastMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (ts == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_fastMoveSpeed.startingSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _fastMoveSpeed.targetSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _fastMoveSpeed.accelerationSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        ts->startingSpeed = _fastMoveSpeed.startingSpeed;
        ts->targetSpeed = _fastMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _fastMoveSpeed.accelerationSpeed;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetInterpolationMoveSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (speed <= 0)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_vector_conspeed(speed) == 0)
            {
                _interpolationMoveSpeed = speed;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::InterpolationMoveSpeed(double* speed)
    {
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_interpolationMoveSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *speed = _interpolationMoveSpeed;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetInterpolationFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (startingSpeed <= 0 || targetSpeed <= 0 || accelerationSpeed <= 0 || startingSpeed >= targetSpeed)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_vector_profile(startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _interpolationFastMoveSpeed.startingSpeed = startingSpeed;
                _interpolationFastMoveSpeed.targetSpeed = targetSpeed;
                _interpolationFastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetInterpolationFastMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (ts == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        return SetInterpolationFastMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    MotorControlState Motor::InterpolationFastMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (ts == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_interpolationFastMoveSpeed.startingSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _interpolationFastMoveSpeed.targetSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _interpolationFastMoveSpeed.accelerationSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        ts->startingSpeed = _interpolationFastMoveSpeed.startingSpeed;
        ts->targetSpeed = _interpolationFastMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _interpolationFastMoveSpeed.accelerationSpeed;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetMaxSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (speed <= 0)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (set_maxspeed(_axisChannel, speed) == 0)
            {
                _maxSpeed = speed;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::MaxSpeed(double* speed)
    {
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_maxSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *speed = _maxSpeed;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::RunningSpeed(double* speed)
    {
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        *speed = get_rate(_axisChannel);
        if (*speed == -1)
        {
            return MotorControlState::UNDEFINED;
        }
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::Move(long distance, MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (distance <= 0)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            switch (direction)
            {
                case MoveDirection::POSITIVE_DIRECTION:
                {
                    if (con_pmove(_axisChannel, distance) == 0)
                    {
                        return MotorControlState::SUCCESS;
                    }
                    break;
                }

                case MoveDirection::NEGATIVE_DIRECTION:
                {
                    if (con_pmove(_axisChannel, ~distance + 1) == 0)
                    {
                        return MotorControlState::SUCCESS;
                    }
                    break;
                }
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::ContinuousMove(MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (con_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::FastMove(long distance, MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (distance <= 0)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            switch (direction)
            {
                case MoveDirection::POSITIVE_DIRECTION:
                {
                    if (fast_pmove(_axisChannel, distance) == 0)
                    {
                        return MotorControlState::SUCCESS;
                    }
                    break;
                }

                case MoveDirection::NEGATIVE_DIRECTION:
                {
                    if (fast_pmove(_axisChannel, ~distance + 1) == 0)
                    {
                        return MotorControlState::SUCCESS;
                    }
                    break;
                }
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::FastContinuousMove(MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (fast_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::OriginMove(MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (con_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::FastOriginMove(MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (fast_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

}
