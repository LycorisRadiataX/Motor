#include "MotorComponent.h"

namespace MotorComponent
{

    bool Motor::_initSuccess = false;
    AllAxisSignalFlag Motor::_aasf = {false, false, false, false, false, false, false, false, 
        false, false, false, false, false, false, false, false, false};

    Motor::Motor(int axisChannel) :
        _axisChannel(axisChannel),
        _position(0),
        _moveSpeed(2000),
        _fastMoveSpeed{2000, 8000, 80000},
        _interpolationMoveSpeed(2000),
        _interpolationFastMoveSpeed{2000, 8000, 80000},
        _maxSpeed(0),
        _positionTriggerPoint{0, 0},
        _moveMode(MOVE_MODE::PULSE_AND_DIRECTION),
        _originDetectionMode(ORIGIN_DETECTION_MODE::SWITCH_SIGNAL),
        _originSignalFlag(ORIGIN_SIGNAL_FLAG::SIGNAL_VALID),
        _decelerationSignalFlag(DECELERATION_SIGNAL_FLAG::SIGNAL_VALID),
        _limitSignalFlag(LILMIT_SIGNAL_FLAG::SIGNAL_VALID),
        _positionTriggerFlag(POSITION_TRIGGER_FLAG::SIGNAL_INVALID),
        _originSignalMode(ORIGIN_SIGNAL_MODE::LOW_LEVEL),
        _decelerationSignalMode(DECELERATION_SIGNAL_MODE::LOW_LEVEL),
        _limitSignalMode(LIMIT_SIGNAL_MODE::HIGH_LEVEL),
        _alarmSignalMode(ALARM_SIGNAL_MODE::LOW_LEVEL),
        _saf{false, false, false, false, false, false, false, false, false, false}
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

    const int Motor::Axis()
    {
        return get_max_axe();
    }

    const int Motor::Board()
    {
        return get_board_num();
    }

    void Motor::InitDefaultSetting()
    {
        _position = 0;
        _moveSpeed = 2000;
        _fastMoveSpeed = {2000, 8000, 80000};
        _interpolationMoveSpeed = 2000;
        _interpolationFastMoveSpeed = {2000, 8000, 80000};
        _maxSpeed = 0;
        _positionTriggerPoint = {0, 0};
        _moveMode = MOVE_MODE::PULSE_AND_DIRECTION;
        _originDetectionMode = ORIGIN_DETECTION_MODE::SWITCH_SIGNAL;
        _originSignalFlag = ORIGIN_SIGNAL_FLAG::SIGNAL_VALID;
        _decelerationSignalFlag = DECELERATION_SIGNAL_FLAG::SIGNAL_VALID;
        _limitSignalFlag = LILMIT_SIGNAL_FLAG::SIGNAL_VALID;
        _positionTriggerFlag = POSITION_TRIGGER_FLAG::SIGNAL_INVALID;
        _originSignalMode = ORIGIN_SIGNAL_MODE::LOW_LEVEL;
        _decelerationSignalMode = DECELERATION_SIGNAL_MODE::LOW_LEVEL;
        _limitSignalMode = LIMIT_SIGNAL_MODE::HIGH_LEVEL;
        _alarmSignalMode = ALARM_SIGNAL_MODE::LOW_LEVEL;
    }

    bool Motor::SetMoveSpeed(const double speed)
    {
        if (_initSuccess)
        {
            if (set_conspeed(_axisChannel, speed) == 0)
            {
                _moveSpeed = speed;
                return true;
            }
        }
        return false;
    }

    const double& Motor::MoveSpeed()
    {
        return _moveSpeed;
    }
    
    bool Motor::SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (set_profile(_axisChannel, startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _fastMoveSpeed.startingSpeed = startingSpeed;
                _fastMoveSpeed.targetSpeed = targetSpeed;
                _fastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return true;
            }
        }
        return false;
    }

    bool Motor::SetFastMoveSpeed(const TrapezaidalSpeed& ts)
    {
        return SetFastMoveSpeed(ts.startingSpeed, ts.targetSpeed, ts.accelerationSpeed);
    }

    const TrapezaidalSpeed& Motor::FastMoveSpeed()
    {
        return _fastMoveSpeed;
    }

    bool Motor::SetInterpolationMoveSpeed(const double speed)
    {
        if (_initSuccess)
        {
            if (set_vector_conspeed(speed) == 0)
            {
                _interpolationMoveSpeed = speed;
                return true;
            }
        }
        return false;
    }

    const double& Motor::InterpolationMoveSpeed()
    {
        return _interpolationMoveSpeed;
    }

    bool Motor::SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed)
    {
        if (_initSuccess)
        {
            if (startingSpeed <= 0 || targetSpeed <= 0 || accelerationSpeed <= 0 || startingSpeed >= targetSpeed)
            {
                return false;
            }
            if (set_vector_profile(startingSpeed, targetSpeed, accelerationSpeed) == 0)
            {
                _interpolationFastMoveSpeed.startingSpeed = startingSpeed;
                _interpolationFastMoveSpeed.targetSpeed = targetSpeed;
                _interpolationFastMoveSpeed.accelerationSpeed = accelerationSpeed;
                return true;
            }
        }
        return false;
    }

    bool Motor::SetInterpolationFastMoveSpeed(const TrapezaidalSpeed& ts)
    {
        return SetInterpolationFastMoveSpeed(ts.startingSpeed, ts.targetSpeed, ts.accelerationSpeed);
    }

    const TrapezaidalSpeed& Motor::InterpolationFastMoveSpeed()
    {
        return _interpolationFastMoveSpeed;
    }

    bool Motor::SetMaxSpeed(const double speed)
    {
        if (_initSuccess)
        {
            if (set_maxspeed(_axisChannel, speed) == 0)
            {
                _maxSpeed = speed;
                return true;
            }
        }
        return false;
    }

    const double& Motor::MaxSpeed()
    {
        return _maxSpeed;
    }

    const double Motor::RunningSpeed()
    {
        return get_rate(_axisChannel);
    }

    bool Motor::Move(const long distance)
    {
        if (_initSuccess)
        {
            if (con_pmove(_axisChannel, distance) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::ContinuousMove(const MOVE_DIRECTION direction)
    {
        if (_initSuccess)
        {
            if (con_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::FastMove(const long distance)
    {
        if (_initSuccess)
        {
            if (fast_pmove(_axisChannel, distance) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::FastContinuousMove(const MOVE_DIRECTION direction)
    {
        if (_initSuccess)
        {
            if (fast_vmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::OriginMove(const MOVE_DIRECTION direction)
    {
        if (_initSuccess)
        {
            if (con_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::FastOriginMove(const MOVE_DIRECTION direction)
    {
        if (_initSuccess)
        {
            if (fast_hmove(_axisChannel, static_cast<int>(direction)) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::StopMove()
    {
        if (_initSuccess)
        {
            if (sudden_stop(_axisChannel) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::StopFastMove()
    {
        if (_initSuccess)
        {
            if (decel_stop(_axisChannel) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::SetAbsolutePosition(const long position)
    {
        if (_initSuccess)
        {
            if (set_abs_pos(_axisChannel, position) == 0)
            {
                return true;
            }
        }
        return false;
    }

    const long& Motor::AbsolutePosition()
    {
        get_encoder(_axisChannel, &_position);
        return _position;
    }

    long Motor::RelativelyPosition()
    {
        long oldPosition = _position;
        long newPosition = AbsolutePosition();
        long position = newPosition - oldPosition;
        return position;
        
    }

    bool Motor::ResetPosition()
    {
        if (_initSuccess)
        {
            if (reset_pos(_axisChannel) == 0)
            {
                return true;
            }
        }
        return false;
    }

    bool Motor::SetPositionTriggerPoint(const long startingPosition, const long targetPosition)
    {
        if (_initSuccess)
        {
            if (set_io_pos(_axisChannel, startingPosition, targetPosition) == 0)
            {
                _positionTriggerPoint.startingPosition = startingPosition;
                _positionTriggerPoint.targetPosition = targetPosition;
                return true;
            }
        }
        return false;
    }

    bool Motor::SetPositionTriggerPoint(const PositionTrigger* pt)
    {
        return SetPositionTriggerPoint(pt->startingPosition, pt->targetPosition);
    }

    const PositionTrigger& Motor::PositionTriggerPoint()
     {
         return _positionTriggerPoint;
     }

    bool Motor::SetMoveMode(const MOVE_MODE mode)
    {
        if (_initSuccess)
        {
            if (set_outmode(_axisChannel, static_cast<int>(mode), 1) == 0)
            {
                return true;
            }
        }
        return false;
    }

    const MOVE_MODE& Motor::MoveMode()
    {
        return _moveMode;
    }

    bool Motor::SetOriginDetectionMode(const ORIGIN_DETECTION_MODE mode)
    {
        if (_initSuccess)
        {
            if (set_home_mode(_axisChannel, static_cast<int>(mode)) == 0)
            {
                return true;
            }
        }
        return false;
    }

    const ORIGIN_DETECTION_MODE& Motor::OriginDetectionMode()
    {
        return _originDetectionMode;
    }

    bool Motor::SetOriginSignalIsValid(const ORIGIN_SIGNAL_FLAG flag)
    {
        if (_initSuccess)
        {
            if (enable_org(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _originSignalFlag = flag;
                return true;
            }
        }
        return false;
    }

    const ORIGIN_SIGNAL_FLAG& Motor::OriginSignalIsValid()
    {
        return _originSignalFlag;
    }

    bool Motor::SetDecelerationSignalIsValid(const DECELERATION_SIGNAL_FLAG flag)
    {
        if (_initSuccess)
        {
            if (enable_sd(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _decelerationSignalFlag = flag;
                return true;
            }
        }
        return false;
    }

    const DECELERATION_SIGNAL_FLAG& Motor::DecelerationSignalIsValid()
    {
        return _decelerationSignalFlag;
    }

    bool Motor::SetLimitSignalIsValid(const LILMIT_SIGNAL_FLAG flag)
    {
        if (_initSuccess)
        {
            if (enable_el(_axisChannel, static_cast<int>(flag)) == 0)
            {
                _limitSignalFlag = flag;
                return true;
            }
        }
        return false;
    }

    const LILMIT_SIGNAL_FLAG& Motor::LimitSignalIsValid()
    {
        return _limitSignalFlag;
    }

    bool Motor::SetPositionTriggerIsValid(const int card, const POSITION_TRIGGER_FLAG flag)
    {
        if (_initSuccess)
        {
            if (enable_io_pos(card, static_cast<int>(flag)) == 0)
            {
                _positionTriggerFlag = flag;
                return true;
            }
        }
        return false;
    }

    const POSITION_TRIGGER_FLAG& Motor::PositionTriggerIsValid()
    {
        return _positionTriggerFlag;
    }

    bool Motor::SetOriginSignalMode(const ORIGIN_SIGNAL_MODE mode)
    {
        if (_initSuccess)
        {
            if (set_org_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _originSignalMode = mode;
                return true;
            }
        }
        return false;
    }

    const ORIGIN_SIGNAL_MODE& Motor::OriginSignalMode()
    {
        return _originSignalMode;
    }

    bool Motor::SetDecelerationSignalMode(const DECELERATION_SIGNAL_MODE mode)
    {
        if (_initSuccess)
        {
            if (set_sd_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _decelerationSignalMode = mode;
                return true;
            }
        }
        return false;
    }

    const DECELERATION_SIGNAL_MODE& Motor::DecelerationSignalMode()
    {
        return _decelerationSignalMode;
    }

    bool Motor::SetLimitSignalMode(const LIMIT_SIGNAL_MODE mode)
    {
        if (_initSuccess)
        {
            if (set_el_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _limitSignalMode = mode;
                return true;
            }
        }
        return false;
    }

    const LIMIT_SIGNAL_MODE& Motor::LimitSignalMode()
    {
        return _limitSignalMode;
    }

    bool Motor::SetAlarmSignalMode(const ALARM_SIGNAL_MODE mode)
    {
        if (_initSuccess)
        {
            if (set_alm_logic(_axisChannel, static_cast<int>(mode)) == 0)
            {
                _alarmSignalMode = mode;
                return true;
            }
        }
        return false;
    }

    const ALARM_SIGNAL_MODE& Motor::AlarmSignalMode()
    {
        return _alarmSignalMode;
    }

    const SingleAxisFlag& Motor::AxisFlag()
    {
        if (_initSuccess)
        {
            int flag = check_status(_axisChannel);
            if (flag)
            {
                _saf.origin = flag & 0x04000000;
                _saf.positiveLimit = flag & 0x02000000;
                _saf.negativeLimit = flag & 0x01000000;
                _saf.alarm = flag & 0x00010000;
                _saf.run = flag & 0x00000080;
                _saf.deceleration = flag & 0x00000008;;
                _saf.stopDueToOrigin = flag & 0x00000400;
                _saf.stopDueToPositiveLimit = flag & 0x00000200;
                _saf.stopDueToNegativeLimit = flag & 0x00000100;
                _saf.stopDueToAlarm = flag & 0x00002000;
                return _saf;
            }
        }
        return _saf;
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

    const AllAxisSignalFlag& Motor::DecelerationAndLimitAndOriginSignal(int card)
    {
        int flag = check_SFR(card);
        if (flag != -1)
        {
            _aasf.alarm = flag & 0x00010000;
            _aasf.originOne = flag & 0x00000008;
            _aasf.positiveLimitOne = flag & 0x00000004;
            _aasf.negativeLimitOne = flag & 0x00000002;
            _aasf.decelerationOne = flag & 0x00000001;
            _aasf.originTwo = flag & 0x00000080;
            _aasf.positiveLimitTwo = flag & 0x00000040;
            _aasf.negativeLimitTwo = flag & 0x00000020;
            _aasf.decelerationTwo = flag & 0x00000010;
            _aasf.originThree = flag & 0x00000800;
            _aasf.positiveLimitThree = flag & 0x00000400;
            _aasf.negativeLimitThree = flag & 0x00000200;
            _aasf.decelerationThree = flag & 0x00000100;
            _aasf.originFour = flag & 0x00008000;
            _aasf.positiveLimitFour = flag & 0x00004000;
            _aasf.negativeLimitFour = flag & 0x00002000;
            _aasf.decelerationFour = flag & 0x00001000;
            return _aasf;
        }
        return _aasf;
    }
}
