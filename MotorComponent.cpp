#include "MotorComponent.h"

namespace MotorComponent
{

    bool Motor::_initSuccess = false;

    Motor::Motor(int axisChannel) :
        _axisChannel(axisChannel),
        _moveMode(static_cast<enum MoveMode>(MotorDataState::UNDEFINED)),
        _originDetectionMode(static_cast<enum OriginDetectionMode>(MotorDataState::UNDEFINED)),
        _moveSpeed(static_cast<double>(MotorDataState::UNDEFINED)),
        _fastMoveSpeed{static_cast<double>(MotorDataState::UNDEFINED), 
                                            static_cast<double>(MotorDataState::UNDEFINED), 
                                            static_cast<double>(MotorDataState::UNDEFINED)},
        _interpolationMoveSpeed(static_cast<double>(MotorDataState::UNDEFINED)),
        _interpolationFastMoveSpeed{static_cast<double>(MotorDataState::UNDEFINED), 
                                            static_cast<double>(MotorDataState::UNDEFINED), 
                                            static_cast<double>(MotorDataState::UNDEFINED)},
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

    MotorDataState Motor::InitDefaultSetting()
    {

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

    MotorDataState Motor::SetMoveSpeed(double speed)
    {
        if (_initSuccess)
        {
            if (set_conspeed(_axisChannel, speed) == 0)
            {
                _moveSpeed = speed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::MoveSpeed(double* speed)
    {
        if (_moveSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        *speed = _moveSpeed;
        return MotorDataState::DEFINED;
    }
    
    MotorDataState Motor::SetFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (set_profile(_axisChannel, startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _fastMoveSpeed.startingSpeed = startingSpeed;
                _fastMoveSpeed.targetSpeed = targetSpeed;
                _fastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::SetFastMoveSpeed(TrapezaidalSpeed* ts)
    {
        return SetFastMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    MotorDataState Motor::FastMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (_fastMoveSpeed.startingSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _fastMoveSpeed.targetSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _fastMoveSpeed.accelerationSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        ts->startingSpeed = _fastMoveSpeed.startingSpeed;
        ts->targetSpeed = _fastMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _fastMoveSpeed.accelerationSpeed;
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

    MotorDataState Motor::SetInterpolationFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (set_vector_profile(startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _interpolationFastMoveSpeed.startingSpeed = startingSpeed;
                _interpolationFastMoveSpeed.targetSpeed = targetSpeed;
                _interpolationFastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return MotorDataState::SUCCESS;
            }
        }
        return MotorDataState::FAILURE;
    }

    MotorDataState Motor::SetInterpolationFastMoveSpeed(TrapezaidalSpeed* ts)
    {
        return SetInterpolationFastMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    MotorDataState Motor::InterpolationFastMoveSpeed(TrapezaidalSpeed* ts)
    {
        if (_interpolationFastMoveSpeed.startingSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _interpolationFastMoveSpeed.targetSpeed == static_cast<double>(MotorDataState::UNDEFINED) ||
                _interpolationFastMoveSpeed.accelerationSpeed == static_cast<double>(MotorDataState::UNDEFINED))
        {
            return MotorDataState::UNDEFINED;
        }
        ts->startingSpeed = _interpolationFastMoveSpeed.startingSpeed;
        ts->targetSpeed = _interpolationFastMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _interpolationFastMoveSpeed.accelerationSpeed;
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
