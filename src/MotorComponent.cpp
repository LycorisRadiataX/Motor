#include "MotorComponent.h"

namespace MotorComponent
{

    bool Motor::_initSuccess = false;

    Motor::Motor(int axisChannel) :
        _axisChannel(axisChannel),
        _moveMode(static_cast<enum Move_Mode>(Motor_Control_State::UNDEFINED)),
        _originDetectionMode(static_cast<enum OriginDetectionMode>(Motor_Control_State::UNDEFINED)),
        _moveSpeed(static_cast<double>(Motor_Control_State::UNDEFINED)),
        _fastMoveSpeed{static_cast<double>(Motor_Control_State::UNDEFINED), 
                                            static_cast<double>(Motor_Control_State::UNDEFINED), 
                                            static_cast<double>(Motor_Control_State::UNDEFINED)},
        _interpolationMoveSpeed(static_cast<double>(Motor_Control_State::UNDEFINED)),
        _interpolationFastMoveSpeed{static_cast<double>(Motor_Control_State::UNDEFINED), 
                                            static_cast<double>(Motor_Control_State::UNDEFINED), 
                                            static_cast<double>(Motor_Control_State::UNDEFINED)},
        _maxSpeed(static_cast<double>(Motor_Control_State::UNDEFINED)),
        _positionTriggerFlag(static_cast<enum PositionTriggerFlag>(Motor_Control_State::UNDEFINED)),
        _decelerationSignalFlag(static_cast<enum DecelerationSignalFlag>(Motor_Control_State::UNDEFINED)),
        _limitSignalFlag(static_cast<enum LimitSignalFlag>(Motor_Control_State::UNDEFINED)),
        _originSignalFlag(static_cast<enum OriginSignalFlag>(Motor_Control_State::UNDEFINED)),
        _decelerationSignalMode(static_cast<enum DecelerationSignalMode>(Motor_Control_State::UNDEFINED)),
        _limitSignalMode(static_cast<enum LimitSignalMode>(Motor_Control_State::UNDEFINED)),
        _originSignalMode(static_cast<enum OriginSignalMode>(Motor_Control_State::UNDEFINED)),
        _alarmSignalMode(static_cast<enum AlarmSignalMode>(Motor_Control_State::UNDEFINED)),
        _startingPosition(static_cast<long>(Motor_Control_State::UNDEFINED)),
        _targetPosition(static_cast<long>(Motor_Control_State::UNDEFINED))
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

    void Motor::InitDefaultSetting()
    {

    }

    Motor_Control_State Motor::SetMoveMode(const Move_Mode mode)
    {
        if (_initSuccess)
        {
            if (mode != Move_Mode::PULSE_AND_DIRECTION || mode != Move_Mode::DOUBLE_PULSE)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_outmode(_axisChannel, static_cast<int>(mode), 1) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Move_Mode Motor::MoveMode()
    {
        if (static_cast<Motor_Control_State>(_moveMode) == Motor_Control_State::UNDEFINED)
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (mode == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        *mode = _moveMode;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetOriginDetectionMode(const OriginDetectionMode mode)
    {
        if (_initSuccess)
        {
            if (mode != OriginDetectionMode::SWITCH_SIGNAL || mode != OriginDetectionMode::SWITCH_SIGNAL_AND_ENCODER_SIGNAL)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_home_mode(_axisChannel, static_cast<int>(mode)) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Origin_Detection_Mode Motor::OriginDetectionMode()
    {
        if (static_cast<Motor_Control_State>(_originDetectionMode) == Motor_Control_State::UNDEFINED)
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (mode == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        *mode = _originDetectionMode;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetMoveSpeed(const double speed)
    {
        if (_initSuccess)
        {
            if (speed <= 0)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_conspeed(_axisChannel, speed) == 0)
            {
                _moveSpeed = speed;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    double Motor::MoveSpeed()
    {
        if (_moveSpeed == static_cast<double>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (speed == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        *speed = _moveSpeed;
        return Motor_Control_State::DEFINED;
    }
    
    bool Motor::SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (startingSpeed <= 0 || targetSpeed <= 0 || accelerationSpeed <= 0 || startingSpeed >= targetSpeed)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_profile(_axisChannel, startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _fastMoveSpeed.startingSpeed = startingSpeed;
                _fastMoveSpeed.targetSpeed = targetSpeed;
                _fastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::SetFastMoveSpeed(const TrapezaidalSpeed* const ts)
    {
        if (ts == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        return SetFastMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    TrapezaidalSpeed Motor::FastMoveSpeed()
    {
        if (_fastMoveSpeed.startingSpeed == static_cast<double>(Motor_Control_State::UNDEFINED) ||
                _fastMoveSpeed.targetSpeed == static_cast<double>(Motor_Control_State::UNDEFINED) ||
                _fastMoveSpeed.accelerationSpeed == static_cast<double>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (ts == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        ts->startingSpeed = _fastMoveSpeed.startingSpeed;
        ts->targetSpeed = _fastMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _fastMoveSpeed.accelerationSpeed;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetInterpolationMoveSpeed(const double speed)
    {
        if (_initSuccess)
        {
            if (speed <= 0)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_vector_conspeed(speed) == 0)
            {
                _interpolationMoveSpeed = speed;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    double Motor::InterpolationMoveSpeed()
    {
        if (_interpolationMoveSpeed == static_cast<double>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
         if (speed == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        *speed = _interpolationMoveSpeed;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (startingSpeed <= 0 || targetSpeed <= 0 || accelerationSpeed <= 0 || startingSpeed >= targetSpeed)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_vector_profile(startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _interpolationFastMoveSpeed.startingSpeed = startingSpeed;
                _interpolationFastMoveSpeed.targetSpeed = targetSpeed;
                _interpolationFastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::SetInterpolationFastMoveSpeed(const TrapezaidalSpeed* const ts)
    {
        if (ts == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        return SetInterpolationFastMoveSpeed(ts->startingSpeed, ts->targetSpeed, ts->accelerationSpeed);
    }

    TrapezaidalSpeed Motor::InterpolationFastMoveSpeed()
    {
        if (_interpolationFastMoveSpeed.startingSpeed == static_cast<double>(Motor_Control_State::UNDEFINED) ||
                _interpolationFastMoveSpeed.targetSpeed == static_cast<double>(Motor_Control_State::UNDEFINED) ||
                _interpolationFastMoveSpeed.accelerationSpeed == static_cast<double>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (ts == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        ts->startingSpeed = _interpolationFastMoveSpeed.startingSpeed;
        ts->targetSpeed = _interpolationFastMoveSpeed.targetSpeed;
        ts->accelerationSpeed = _interpolationFastMoveSpeed.accelerationSpeed;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetMaxSpeed(const double speed)
    {
        if (_initSuccess)
        {
            if (speed <= 0)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (set_maxspeed(_axisChannel, speed) == 0)
            {
                _maxSpeed = speed;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    double Motor::MaxSpeed()
    {
        if (_maxSpeed == static_cast<double>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (speed == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        *speed = _maxSpeed;
        return Motor_Control_State::DEFINED;
    }

    double Motor::RunningSpeed()
    {
        double runSpeed = get_rate(_axisChannel);
        if (runSpeed == -1)
        {
            return Motor_Control_State::UNDEFINED;
        }
        if (speed == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        *speed = runSpeed;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::Move(const long distance)
    {
        if (_initSuccess)
        {
            if (MoveSpeed(nullptr) == Motor_Control_State::UNDEFINED)
            {
                return Motor_Control_State::FAILURE;
            }
            if (con_pmove(_axisChannel, distance) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::ContinuousMove(const MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (MoveSpeed(nullptr) == Motor_Control_State::UNDEFINED)
            {
                return Motor_Control_State::FAILURE;
            }
            if (con_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::FastMove(const long distance)
    {
        if (_initSuccess)
        {
            if (FastMoveSpeed(nullptr) == Motor_Control_State::UNDEFINED)
            {
                return Motor_Control_State::FAILURE;
            }
            if (fast_pmove(_axisChannel, distance) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::FastContinuousMove(const MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (FastMoveSpeed(nullptr) == Motor_Control_State::UNDEFINED)
            {
                return Motor_Control_State::FAILURE;
            }
            if (fast_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::OriginMove(const MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (MoveSpeed(nullptr) == Motor_Control_State::UNDEFINED)
            {
                return Motor_Control_State::FAILURE;
            }
            if (con_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::FastOriginMove(const MoveDirection direction)
    {
        if (_initSuccess)
        {
            if (FastMoveSpeed(nullptr) == Motor_Control_State::UNDEFINED)
            {
                return Motor_Control_State::FAILURE;
            }
            if (fast_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::StopMove()
    {
        if (_initSuccess)
        {
            if (sudden_stop(_axisChannel) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::StopFastMove()
    {
        if (_initSuccess)
        {
            if (decel_stop(_axisChannel) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::SetAbsolutePosition(const long position)
    {
        if (_initSuccess)
        {
            if (set_abs_pos(_axisChannel, position) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    long Motor::AbsolutePosition()
    {
        if (_initSuccess)
        {
            if (position == nullptr)
            {
                return Motor_Control_State::PARAMETER_ERROR;
            }
            if (get_encoder(_axisChannel, &_position) == -1)
            {
                return Motor_Control_State::UNDEFINED;
            }
        }
        *position = _position;
        return Motor_Control_State::DEFINED;
    }

    long Motor::RelativelyPosition()
    {
        long oldPosition = _position;
        long newPosition = static_cast<long>(Motor_Control_State::UNDEFINED);
        if (position == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        if (AbsolutePosition(&newPosition) == Motor_Control_State::UNDEFINED)
        {
            return Motor_Control_State::UNDEFINED;
        }
        *position = newPosition - oldPosition;
        return Motor_Control_State::DEFINED;
        
    }

    bool Motor::ResetPosition()
    {
        if (_initSuccess)
        {
            if (reset_pos(_axisChannel) == 0)
            {
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::SetPositionTriggerPoint(const long startingPosition, const long targetPosition)
    {
        if (_initSuccess)
        {
            if (set_io_pos(_axisChannel, startingPosition, targetPosition) == 0)
            {
                _startingPosition = startingPosition;
                _targetPosition = targetPosition;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    bool Motor::SetPositionTriggerPoint(const PositionTrigger* pt)
    {

    }

    PositionTrigger Motor::PositionTriggerPoint()
     {
         if (startingPosition == nullptr || targetPosition == nullptr)
         {
             return Motor_Control_State::PARAMETER_ERROR;
         }
         if (_startingPosition == static_cast<long>(Motor_Control_State::UNDEFINED) || _targetPosition == static_cast<long>(Motor_Control_State::UNDEFINED))
         {
             return Motor_Control_State::UNDEFINED;
         }
         *startingPosition = _startingPosition;
         *targetPosition = _targetPosition;
         return Motor_Control_State::DEFINED;
     }

    bool Motor::SetPositionTriggerIsValid(const int card, const Position_Trigger_Flag flag)
    {
        if (_initSuccess)
        {
            if (enable_io_pos(card, static_cast<int>(flag)) == 0)
            {
                _positionTriggerFlag = flag;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Position_Trigger_Flag Motor::PositionTriggerIsValid()
    {
        if (flag == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        if (_positionTriggerFlag == static_cast<PositionTriggerFlag>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        *flag = _positionTriggerFlag;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetDecelerationSignalIsValid(const Deceleration_Signal_Flag flag)
    {
        if (_initSuccess)
        {
            if (enable_sd(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _decelerationSignalFlag = flag;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Deceleration_Signal_Flag Motor::DecelerationSignalIsValid()
    {
        if (flag == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        if (_decelerationSignalFlag == static_cast<DecelerationSignalFlag>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        *flag = _decelerationSignalFlag;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetLimitSignalIsValid(const Limit_Signal_Flag flag)
    {
        if (_initSuccess)
        {
            if (enable_el(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _limitSignalFlag = flag;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Limit_Signal_Flag Motor::LimitSignalIsValid()
    {
        if (flag == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        if (_limitSignalFlag == static_cast<LimitSignalFlag>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        *flag = _limitSignalFlag;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetOriginSignalIsValid(const Origin_Signal_Flag flag)
    {
        if (_initSuccess)
        {
            if (enable_org(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _originSignalFlag = flag;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Origin_Signal_Flag Motor::OriginSignalIsValid()
    {
        if (flag == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        if (_originSignalFlag == static_cast<OriginSignalFlag>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        *flag = _originSignalFlag;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetDecelerationSignalMode(const Deceleration_Signal_Mode mode)
    {
        if (_initSuccess)
        {
            if (set_sd_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _decelerationSignalMode = mode;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Deceleration_Signal_Mode Motor::DecelerationSignalMode()
    {
        if (mode == nullptr)
        {
            return Motor_Control_State::PARAMETER_ERROR;
        }
        if (_decelerationSignalMode == static_cast<enum DecelerationSignalMode>(Motor_Control_State::UNDEFINED))
        {
            return Motor_Control_State::UNDEFINED;
        }
        *mode = _decelerationSignalMode;
        return Motor_Control_State::DEFINED;
    }

    bool Motor::SetLimitSignalMode(const Limit_Signal_Mode mode)
    {
        if (_initSuccess)
        {
            if (set_el_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _limitSignalMode = mode;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

     Limit_Signal_Mode Motor::LimitSignalMode()
     {
         if (mode == nullptr)
         {
             return Motor_Control_State::PARAMETER_ERROR;
         }
         if (_limitSignalMode == static_cast<enum LimitSignalMode>(Motor_Control_State::UNDEFINED))
         {
             return Motor_Control_State::UNDEFINED;
         }
         *mode = _limitSignalMode;
         return Motor_Control_State::DEFINED;
     }

    bool Motor::SetOriginSignalMode(const enum OriginSignalMode mode)
    {
        if (_initSuccess)
        {
            if (set_org_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _originSignalMode = mode;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    OriginSignalMode Motor::OriginSignalMode()
    {

    }

    bool Motor::SetAlarmSignalMode(const enum AlarmSignalMode mode)
    {
        if (_initSuccess)
        {
            if (set_alm_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _alarmSignalMode = mode;
                return Motor_Control_State::SUCCESS;
            }
        }
        return Motor_Control_State::FAILURE;
    }

    Alarm_Signal_Mode Motor::AlarmSignalMode()
    {

    }

    SingleAxisFlag Motor::AxisFlag()
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
                return saf;
            }
        }
        return saf;
    }

    bool Motor::AxisStop()
    {
        if (check_done(_axisChannel) == 0)
        {
            return true;
        }
        return false;
    }

    bool Motor::DecelerationSignal()
    {
        if(check_SD(_axisChannel) == 1)
        {
            return true;
        }
        return false;
    }

    bool Motor::PositiveLimitSignal()
    {
        if (check_limit(_axisChannel) == 1 || check_limit(_axisChannel) == 2)
        {
            return true;
        }
        return false;
    }

    bool Motor::NegativeLimitSignal()
    {
        if (check_limit(_axisChannel) == -1 || check_limit(_axisChannel) == 2)
        {
            return true;
        }
        return false;
    }

    bool Motor::OriginSignal()
    {
        if (check_home(_axisChannel) == 1)
        {
            return true;
        }
        return false;
    }

    bool Motor::AlarmSignal()
    {
        if (check_alarm(_axisChannel) == 1)
        {
            return true;
        }
        return false;
    }

    AllAxisSignalFlag Motor::DecelerationAndLimitAndOriginSignal(int card)
    {
        AllAxisSignalFlag aasf = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false};
        int flag = check_SFR(card);
        if (flag != -1)
        {
            aasf.alarm = flag & 0x00010000;
            aasf.originOne = flag & 0x00000008;
            aasf.positiveLimitOne = flag & 0x00000004;
            aasf.negativeLimitOne = flag & 0x00000002;
            aasf.decelerationOne = flag & 0x00000001;
            aasf.originTwo = flag & 0x00000080;
            aasf.positiveLimitTwo = flag & 0x00000040;
            aasf.negativeLimitTwo = flag & 0x00000020;
            aasf.decelerationTwo = flag & 0x00000010;
            aasf.originThree = flag & 0x00000800;
            aasf.positiveLimitThree = flag & 0x00000400;
            aasf.negativeLimitThree = flag & 0x00000200;
            aasf.decelerationThree = flag & 0x00000100;
            aasf.originFour = flag & 0x00008000;
            aasf.positiveLimitFour = flag & 0x00004000;
            aasf.negativeLimitFour = flag & 0x00002000;
            aasf.decelerationFour = flag & 0x00001000;
            return aasf;
        }
        return aasf;
    }

}
