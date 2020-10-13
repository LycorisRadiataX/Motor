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
        _alarmSignalMode(static_cast<enum AlarmSignalMode>(MotorControlState::UNDEFINED)),
        _startingPosition(static_cast<long>(MotorControlState::UNDEFINED)),
        _targetPosition(static_cast<long>(MotorControlState::UNDEFINED))
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

    MotorControlState Motor::SetMoveMode(const enum MoveMode mode)
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

    MotorControlState Motor::SetOriginDetectionMode(const enum OriginDetectionMode mode)
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

    MotorControlState Motor::SetMoveSpeed(const double speed)
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
    
    MotorControlState Motor::SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed)
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

    MotorControlState Motor::SetFastMoveSpeed(const TrapezaidalSpeed* const ts)
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

    MotorControlState Motor::SetInterpolationMoveSpeed(const double speed)
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

    MotorControlState Motor::SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed)
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

    MotorControlState Motor::SetInterpolationFastMoveSpeed(const TrapezaidalSpeed* const ts)
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

    MotorControlState Motor::SetMaxSpeed(const double speed)
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

    MotorControlState Motor::Move(const long distance)
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

    MotorControlState Motor::ContinuousMove(const MoveDirection direction)
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

    MotorControlState Motor::FastMove(const long distance)
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

    MotorControlState Motor::FastContinuousMove(const MoveDirection direction)
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

    MotorControlState Motor::OriginMove(const MoveDirection direction)
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

    MotorControlState Motor::FastOriginMove(const MoveDirection direction)
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

    MotorControlState Motor::StopMove()
    {
        if (_initSuccess)
        {
            if (sudden_stop(_axisChannel) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::StopFastMove()
    {
        if (_initSuccess)
        {
            if (decel_stop(_axisChannel) == 0)
            {
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

    MotorControlState Motor::SetAbsolutePosition(const long position)
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

    MotorControlState Motor::SetPositionTriggerPoint(const long startingPosition, const long targetPosition)
    {
        if (_initSuccess)
        {
            if (set_io_pos(_axisChannel, startingPosition, targetPosition) == 0)
            {
                _startingPosition = startingPosition;
                _targetPosition = targetPosition;
                return MotorControlState::SUCCESS;
            }
        }
        return MotorControlState::FAILURE;
    }

     MotorControlState Motor::PositionTriggerPoint(long* startingPosition, long* targetPosition)
     {
         if (startingPosition == nullptr || targetPosition == nullptr)
         {
             return MotorControlState::PARAMETER_ERROR;
         }
         if (_startingPosition == static_cast<long>(MotorControlState::UNDEFINED) || _targetPosition == static_cast<long>(MotorControlState::UNDEFINED))
         {
             return MotorControlState::UNDEFINED;
         }
         *startingPosition = _startingPosition;
         *targetPosition = _targetPosition;
         return MotorControlState::DEFINED;
     }

    MotorControlState Motor::SetPositionTriggerIsValid(const int card, const PositionTriggerFlag flag)
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

    MotorControlState Motor::PositionTriggerIsValid(PositionTriggerFlag* flag)
    {
        if (flag == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_positionTriggerFlag == static_cast<PositionTriggerFlag>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *flag = _positionTriggerFlag;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetDecelerationSignalIsValid(const DecelerationSignalFlag flag)
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

    MotorControlState Motor::DecelerationSignalIsValid(DecelerationSignalFlag* flag)
    {
        if (flag == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_decelerationSignalFlag == static_cast<DecelerationSignalFlag>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *flag = _decelerationSignalFlag;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetLimitSignalIsValid(const LimitSignalFlag flag)
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

    MotorControlState Motor::LimitSignalIsValid(LimitSignalFlag* flag)
    {
        if (flag == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_limitSignalFlag == static_cast<LimitSignalFlag>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *flag = _limitSignalFlag;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetOriginSignalIsValid(const OriginSignalFlag flag)
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

    MotorControlState Motor::OriginSignalIsValid(OriginSignalFlag* flag)
    {
        if (flag == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_originSignalFlag == static_cast<OriginSignalFlag>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *flag = _originSignalFlag;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetDecelerationSignalMode(const enum DecelerationSignalMode mode)
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

    MotorControlState Motor::DecelerationSignalMode(enum DecelerationSignalMode* mode)
    {
        if (mode == nullptr)
        {
            return MotorControlState::PARAMETER_ERROR;
        }
        if (_decelerationSignalMode == static_cast<enum DecelerationSignalMode>(MotorControlState::UNDEFINED))
        {
            return MotorControlState::UNDEFINED;
        }
        *mode = _decelerationSignalMode;
        return MotorControlState::DEFINED;
    }

    MotorControlState Motor::SetLimitSignalMode(const enum LimitSignalMode mode)
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

     MotorControlState Motor::LimitSignalMode(enum LimitSignalMode* mode)
     {
         if (mode == nullptr)
         {
             return MotorControlState::PARAMETER_ERROR;
         }
         if (_limitSignalMode == static_cast<enum LimitSignalMode>(MotorControlState::UNDEFINED))
         {
             return MotorControlState::UNDEFINED;
         }
         *mode = _limitSignalMode;
         return MotorControlState::DEFINED;
     }

    MotorControlState Motor::SetOriginSignalMode(const enum OriginSignalMode mode)
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

    MotorControlState Motor::OriginSignalMode(enum OriginSignalMode* mode)
    {

    }

    MotorControlState Motor::SetAlarmSignalMode(const enum AlarmSignalMode mode)
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

    MotorControlState Motor::AlarmSignalMode(enum AlarmSignalMode* mode)
    {

    }

    SingleAxisFlag Motor::AxisFlag(Motor_Control_State& mcs)
    {
        SingleAxisFlag saf = {false, false, false, false, false, false, false, false, false, false};
        if (_initSuccess)
        {
            int flag = check_status(_axisChannel);
            if (flag)
            {
                saf.origin = flag & 0x04000000;
                saf.positiveLimit = flag & 0x02000000;
                saf.negativeLimit = flag & 0x01000000;
                saf.alarm = flag & 0x00010000;
                saf.run = flag & 0x00000080;
                saf.deceleration = flag & 0x00000008;;
                saf.stopDueToOrigin = flag & 0x00000400;
                saf.stopDueToPositiveLimit = flag & 0x00000200;
                saf.stopDueToNegativeLimit = flag & 0x00000100;
                saf.stopDueToAlarm = flag & 0x00002000;
                if (mcs != nullptr)
                {
                    *mcs = Motor_Control_State::Success;
                }
                return saf;
            }
        }
        if (mcs != nullptr)
        {
            *mcs = Motor_Control_State::Failure;
        }
        return saf;
    }

    bool Motor::AxisStop(Motor_Control_State& mcs)
    {
        if (check_done(_axisChannel) == 0)
        {
            mcs = Motor_Control_State::Success;
            return true;
        }
        return false;
    }

    bool Motor::DecelerationSignal(Motor_Control_State& mcs)
    {
        if(check_SD(_axisChannel) == 1)
        {
            mcs = Motor_Control_State::Success;
            return true;
        }
        return false;
    }

    bool Motor::PositiveLimitSignal(Motor_Control_State& mcs)
    {
        if (check_limit(_axisChannel) == 1 || check_limit(_axisChannel) == 2)
        {
            mcs = Motor_Control_State::Success;
            return true;
        }
        return false;
    }

    bool Motor::NegativeLimitSignal(Motor_Control_State& mcs)
    {
        if (check_limit(_axisChannel) == -1 || check_limit(_axisChannel) == 2)
        {
            mcs = Motor_Control_State::Success;
            return true;
        }
        return false;
    }

    bool Motor::OriginSignal(Motor_Control_State& mcs)
    {
        if (check_home(_axisChannel) == 1)
        {
            mcs = Motor_Control_State::Success;
            return true;
        }
        return false;
    }

    bool Motor::AlarmSignal(Motor_Control_State& mcs)
    {
        if (check_alarm(_axisChannel) == 1)
        {
            mcs = Motor_Control_State::Success;
            return true;
        }
        return false;
    }

    MotorControlState Motor::DecelerationAndLimitAndOriginSignal(int card, Motor_Control_State& mcs)
    {

    }

}
