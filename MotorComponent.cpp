#include "MotorComponent.h"

namespace MotorComponent
{

    bool Motor::_initSuccess = false;

    Motor::Motor(int axisChannel) :
        _axisChannel(axisChannel),
        _moveMode(static_cast<enum MoveMode>(MotorDataState::UNDEFINED)),
        _originDetectionMode(static_cast<enum OriginDetectionMode>(MotorDataState::UNDEFINED)),
        _moveAndContinuousMoveSpeed(static_cast<double>(MotorDataState::UNDEFINED)),
        _continuousMoveTrapezaidalSpeed{static_cast<double>(MotorDataState::UNDEFINED), static_cast<double>(MotorDataState::UNDEFINED), static_cast<double>(MotorDataState::UNDEFINED)},
        _interpolationMoveSpeed(static_cast<double>(MotorDataState::UNDEFINED)),
        _interpolationContinuousMoveSpeed{static_cast<double>(MotorDataState::UNDEFINED), static_cast<double>(MotorDataState::UNDEFINED), static_cast<double>(MotorDataState::UNDEFINED)},
        _maxSpeed(static_cast<double>(MotorDataState::UNDEFINED))
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

    MotorDataState Motor::SetMoveMode(enum MoveMode mode)
    {
        if (_initSuccess)
        {
            if (set_outmode(_axisChannel, static_cast<int>(mode), 1) == 0)
            {
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::MoveMode(enum MoveMode *mode)
    {
        if (static_cast<MotorDataState>(_moveMode) == MotorDataState::UNDEFINED)
        {
            return MotorDataState::UNDEFINED;
        }
        *mode = _moveMode;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::SetOriginDetectionMode(enum OriginDetectionMode mode)
    {
        if (_initSuccess)
        {
            if (set_home_mode(_axisChannel, static_cast<int>(mode)) == 0)
            {
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::OriginDetectionMode(enum OriginDetectionMode *mode)
    {
        if (static_cast<MotorDataState>(_originDetectionMode) == MotorDataState::UNDEFINED)
        {
            return MotorDataState::UNDEFINED;
        }
        *mode = _originDetectionMode;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::SetMoveAndContinuousMoveSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (set_conspeed(_axisChannel, speed) == 0)
            {
                _moveAndContinuousMoveSpeed = speed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::MoveAndContinuousMoveSpeed(double* speed)
    {
        if (_moveAndContinuousMoveSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        *speed = _moveAndContinuousMoveSpeed;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::SetContinuousMoveTrapezaidalSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (set_profile(_axisChannel, startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _continuousMoveTrapezaidalSpeed.startingSpeed = startingSpeed;
                _continuousMoveTrapezaidalSpeed.targetSpeed = targetSpeed;
                _continuousMoveTrapezaidalSpeed.accelerationSpeed = accelerationSpeed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::SetContinuousMoveTrapezaidalSpeed(TrapezaidalSpeed* ts)
    {
        return SetContinuousMoveTrapezaidalSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    MotorDataState Motor::ContinuousMoveTrapezaidalSpeed(TrapezaidalSpeed* ts)
    {
        if (_continuousMoveTrapezaidalSpeed.startingSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _continuousMoveTrapezaidalSpeed.targetSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _continuousMoveTrapezaidalSpeed.accelerationSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        ts->startingSpeed = _continuousMoveTrapezaidalSpeed.startingSpeed;
        ts->targetSpeed = _continuousMoveTrapezaidalSpeed.targetSpeed;
        ts->accelerationSpeed = _continuousMoveTrapezaidalSpeed.accelerationSpeed;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::SetInterpolationMoveSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (set_vector_conspeed(speed) == 0)
            {
                _interpolationMoveSpeed = speed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::InterpolationMoveSpeed(double* speed)
    {
        if (_interpolationMoveSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        *speed = _interpolationMoveSpeed;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::SetInterpolationContinuousMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (set_vector_profile(startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _interpolationContinuousMoveSpeed.startingSpeed = startingSpeed;
                _interpolationContinuousMoveSpeed.targetSpeed = targetSpeed;
                _interpolationContinuousMoveSpeed.accelerationSpeed = accelerationSpeed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::SetInterpolationContinuousMoveSpeed(TrapezaidalSpeed* ts)
    {
        return SetInterpolationContinuousMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    MotorDataState Motor::InterpolationContinuousMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (_interpolationContinuousMoveSpeed.startingSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _interpolationContinuousMoveSpeed.targetSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _interpolationContinuousMoveSpeed.accelerationSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        ts->startingSpeed = _interpolationContinuousMoveSpeed.startingSpeed;
        ts->targetSpeed = _interpolationContinuousMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _interpolationContinuousMoveSpeed.accelerationSpeed;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::SetMaxSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (set_maxspeed(_axisChannel, speed) == 0)
            {
                _maxSpeed = speed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::MaxSpeed(double* speed)
    {
        if (_maxSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        *speed = _maxSpeed;
        return MotorDataState::DEFINED;
    }

    MotorDataState Motor::RunningSpeed(double* speed)
    {
        *speed = get_rate(_axisChannel);
        if (*speed == -1)
        {
            return MotorDataState::UNDEFINED;
        }
        return MotorDataState::DEFINED;
    }

}
