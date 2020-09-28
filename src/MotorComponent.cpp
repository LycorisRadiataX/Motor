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
        _maxSpeed(static_cast<double>(MotorControlState::UNDEFINED)),
        _positionTriggerFlag(static_cast<enum PositionTriggerFlag>(MotorControlState::UNDEFINED)),
        _decelerationSignalFlag(static_cast<enum DecelerationSignalFlag>(MotorControlState::UNDEFINED)),
        _limitSignalFlag(static_cast<enum LimitSignalFlag>(MotorControlState::UNDEFINED)),
        _originSignalFlag(static_cast<enum OriginSignalFlag>(MotorControlState::UNDEFINED)),
        _decelerationSignalMode(static_cast<enum DecelerationSignalMode>(MotorControlState::UNDEFINED)),
        _limitSignalMode(static_cast<enum LimitSignalMode>(MotorControlState::UNDEFINED)),
        _originSignalMode(static_cast<enum OriginSignalMode>(MotorControlState::UNDEFINED)),
        _alarmSignalMode(static_cast<enum AlarmSignalMode>(MotorControlState::UNDEFINED))
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
        if (static_cast<MotorControlState>(_moveMode) == MotorControlState::UNDEFINED)
        {
            return MotorControlState::UNDEFINED;
        }
        if (mode == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
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
        if (static_cast<MotorControlState>(_originDetectionMode) == MotorControlState::UNDEFINED)
        {
            return MotorControlState::UNDEFINED;
        }
        if (mode == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
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
        if (_moveSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
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
        if (_fastMoveSpeed.startingSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _fastMoveSpeed.targetSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _fastMoveSpeed.accelerationSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        if (ts == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
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
        if (_interpolationMoveSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
         if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
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
        if (_interpolationFastMoveSpeed.startingSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _interpolationFastMoveSpeed.targetSpeed == static_cast<double>(MotorControlState::UNDEFINED) ||
                _interpolationFastMoveSpeed.accelerationSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        if (ts == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
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
        if (_maxSpeed == static_cast<double>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        *speed = _maxSpeed;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::RunningSpeed(double* speed)
    {
        double runSpeed = get_rate(_axisChannel);
        if (runSpeed == -1)
        {
            return MotorControlState::UNDEFINED;
        }
        if (speed == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        *speed = runSpeed;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::Move(long distance)
    {
        if (_initSuccess)
        {
            if (MoveSpeed(nullptr) == MotorControlState::UNDEFINED)
            {
                return MotorControlState::FAILURE;
            }
            if (con_pmove(_axisChannel, distance) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::ContinuousMove(MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (MoveSpeed(nullptr) == MotorControlState::UNDEFINED)
            {
                return MotorControlState::FAILURE;
            }
            if (con_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::FastMove(long distance)
    {
        if (_initSuccess)
        {
            if (FastMoveSpeed(nullptr) == MotorControlState::UNDEFINED)
            {
                return MotorControlState::FAILURE;
            }
            if (fast_pmove(_axisChannel, distance) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::FastContinuousMove(MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (FastMoveSpeed(nullptr) == MotorControlState::UNDEFINED)
            {
                return MotorControlState::FAILURE;
            }
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
            if (MoveSpeed(nullptr) == MotorControlState::UNDEFINED)
            {
                return MotorControlState::FAILURE;
            }
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
            if (FastMoveSpeed(nullptr) == MotorControlState::UNDEFINED)
            {
                return MotorControlState::FAILURE;
            }
            if (fast_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetAbsolutePosition(long position)
    {
        if (_initSuccess)
        {
            if (set_abs_pos(_axisChannel, position) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::AbsolutePosition(long* position)
    {
        if (_initSuccess)
        {
            if (position == nullptr)
            {
                return MotorControlState::PARAMETER_ERROR;
            }
            if (get_encoder(_axisChannel, &_position) == -1)
            {
                return MotorControlState::UNDEFINED;
            }
        }
        *position = _position;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::RelativelyPosition(long* position)
    {
        long oldPosition = _position;
        long newPosition = static_cast<long>(MotorControlState::UNDEFINED);
        if (position == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (AbsolutePosition(&newPosition) == MotorControlState::UNDEFINED)
        {
            return MotorControlState::UNDEFINED;
        }
        *position = newPosition - oldPosition;
        return MotorControlState::DEFINED;
        
    }

    MotorControlState Motor::ResetPosition()
    {
        if (_initSuccess)
        {
            if (reset_pos(_axisChannel) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetPositionTriggerPoint(long startingPosition, long targetPosition)
    {
        if (_initSuccess)
        {
            if (set_io_pos(_axisChannel, startingPosition, targetPosition) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetPositionTriggerIsValid(int card, PositionTriggerFlag flag)
    {
        if (_initSuccess)
        {
            if (enable_io_pos(card, static_cast<int>(flag)) == 0)
            {
                _positionTriggerFlag = flag;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetDecelerationSignalIsValid(DecelerationSignalFlag flag)
    {
        if (_initSuccess)
        {
            if (enable_sd(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _decelerationSignalFlag = flag;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetLimitSignalIsValid(LimitSignalFlag flag)
    {
        if (_initSuccess)
        {
            if (enable_el(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _limitSignalFlag = flag;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetOriginSignalIsValid(OriginSignalFlag flag)
    {
        if (_initSuccess)
        {
            if (enable_org(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _originSignalFlag = flag;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetDecelerationSignalMode(DecelerationSignalMode mode)
    {
        if (_initSuccess)
        {
            if (set_sd_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _decelerationSignalMode = mode;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetLimitSignalMode(LimitSignalMode mode)
    {
        if (_initSuccess)
        {
            if (set_el_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _limitSignalMode = mode;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetOriginSignalMode(OriginSignalMode mode)
    {
        if (_initSuccess)
        {
            if (set_org_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _originSignalMode = mode;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetAlarmSignalMode(AlarmSignalMode mode)
    {
        if (_initSuccess)
        {
            if (set_alm_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _alarmSignalMode = mode;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

}
