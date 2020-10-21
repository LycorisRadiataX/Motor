/*
 * Detail : MPC08运动控制卡C++封装, 编译时最低C++标准为C++11
 * Author : LycorisRadiata
 * Date   : 2020.9.15
 * Ver    : v4.3.2.d.145656
*/

#ifndef MOTORCOMPONENT_H_
#define MOTORCOMPONENT_H_

#include "../third_party/Mpc.h"

namespace MotorComponent
{
    struct TrapezaidalSpeed
    {
        double startingSpeed;
        double targetSpeed;
        double accelerationSpeed;
    };

    struct SingleAxisFlag
    {
        bool origin;
        bool positiveLimit;
        bool negativeLimit;
        bool alarm;
        bool run;
        bool deceleration;
        bool stopDueToOrigin;
        bool stopDueToPositiveLimit;
        bool stopDueToNegativeLimit;
        bool stopDueToAlarm;
    };

    struct AllAxisSignalFlag
    {
        bool alarm;
        bool originOne;
        bool positiveLimitOne;
        bool negativeLimitOne;
        bool decelerationOne;
        bool originTwo;
        bool positiveLimitTwo;
        bool negativeLimitTwo;
        bool decelerationTwo;
        bool originThree;
        bool positiveLimitThree;
        bool negativeLimitThree;
        bool decelerationThree;
        bool originFour;
        bool positiveLimitFour;
        bool negativeLimitFour;
        bool decelerationFour;
    };

    struct PositionTrigger
    {
        long startingPosition;
        long targetPosition;
    };

    enum class MOVE_DIRECTION : int
    {
        POSITIVE_DIRECTION = 1,
        NEGATIVE_DIRECTION = -1,
    };

    enum class MOVE_MODE : int
    {
        PULSE_AND_DIRECTION = 1,
        DOUBLE_PULSE = 0,
    };

    enum class ORIGIN_DETECTION_MODE : int
    {
        SWITCH_SIGNAL = 0,
        SWITCH_SIGNAL_AND_ENCODER_SIGNAL = 1,
    };

    enum class ORIGIN_SIGNAL_FLAG : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0,
    };

    enum class DECELERATION_SIGNAL_FLAG : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0,
    };

    enum class LILMIT_SIGNAL_FLAG : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0,
    };

    enum class POSITION_TRIGGER_FLAG : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0,
    };

    enum class ORIGIN_SIGNAL_MODE : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0,
    };

    enum class DECELERATION_SIGNAL_MODE : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0,
    };

    enum class LIMIT_SIGNAL_MODE : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0,
    };

    enum class ALARM_SIGNAL_MODE : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0,
    };

    class Motor
    {
    public:
        explicit Motor(int axisChannel);
        ~Motor();
        Motor(Motor&) = delete;
        Motor(Motor&&) = delete;
        Motor& operator = (Motor&) = delete;
        Motor& operator = (Motor&&) = delete;

        //硬件初始化, 全局调用一次即可
        static bool Init();

        //获取总轴数
        int Axis();

        //获取总板卡数
        int Board();

        //初始化为默认设置
        void InitDefaultSetting();

        //设置和获取运动和连续运动速度
        bool SetMoveSpeed(const double speed);
        const double& MoveSpeed();

        //设置和获取连续运动梯形速度
        bool SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        bool SetFastMoveSpeed(const TrapezaidalSpeed& ts);
        const TrapezaidalSpeed& FastMoveSpeed();

        //设置和获取插补运动速度
        bool SetInterpolationMoveSpeed(const double Speed);
        const double& InterpolationMoveSpeed();

        //设置和获取插补连续运动速度
        bool SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        bool SetInterpolationFastMoveSpeed(const TrapezaidalSpeed& ts);
        const TrapezaidalSpeed& InterpolationFastMoveSpeed();

        //设置和获取最大速度
        bool SetMaxSpeed(const double speed);
        const double& MaxSpeed();

        //获取运行时实际速度
        double RunningSpeed();

        //移动
        bool Move(const long distance);

        //连续移动
        bool ContinuousMove(const MOVE_DIRECTION direction);

        //快速移动
        bool FastMove(const long distance);

        //快速连续移动
        bool FastContinuousMove(const MOVE_DIRECTION direction);

        //回原点移动
        bool OriginMove(const MOVE_DIRECTION direction);

        //快速回原点移动
        bool FastOriginMove(const MOVE_DIRECTION direction);

        //停止运动
        bool StopMove();

        //停止快速运动
        bool StopFastMove();

        //设置当前绝对位置, 但设置后点击不会进行实际运动
        bool SetAbsolutePosition(const long position);

        //获取当前绝对位置
        const long& AbsolutePosition();

        //获取当前相对位置值
        long RelativelyPosition();

        //重置位置值
        bool ResetPosition();

        //设置/获取位置触发点, 位置进入起始点时，自动触发输出 IO 信号（低电平）；当位置走出比较终止点时，自动触发输出高电平
        bool SetPositionTriggerPoint(const long startingPosition, const long targetPosition);
        bool SetPositionTriggerPoint(const PositionTrigger* pt);
        const PositionTrigger& PositionTriggerPoint();

        //设置运动模式
        bool SetMoveMode(const MOVE_MODE mode);
        const MOVE_MODE& MoveMode();

        //设置回原点时的检测模式
        bool SetOriginDetectionMode(const ORIGIN_DETECTION_MODE mode);
        const ORIGIN_DETECTION_MODE& OriginDetectionMode();

        //设置/获取原点信号是否有效
        bool SetOriginSignalIsValid(const ORIGIN_SIGNAL_FLAG flag);
        const ORIGIN_SIGNAL_FLAG& OriginSignalIsValid();

        //设置/获取减速信号是否有效
        bool SetDecelerationSignalIsValid(const DECELERATION_SIGNAL_FLAG flag);
        const DECELERATION_SIGNAL_FLAG& DecelerationSignalIsValid();

        //设置/获取限位信号是否有效
        bool SetLimitSignalIsValid(const LILMIT_SIGNAL_FLAG flag);
        const LILMIT_SIGNAL_FLAG& LimitSignalIsValid();

        //设置/获取位置触发是否有效
        bool SetPositionTriggerIsValid(const int card, const POSITION_TRIGGER_FLAG flag);
        const POSITION_TRIGGER_FLAG& PositionTriggerIsValid();

        //设置/获取原点信号触发模式
        bool SetOriginSignalMode(const ORIGIN_SIGNAL_MODE mode);
        const ORIGIN_SIGNAL_MODE& OriginSignalMode();

        //设置/获取减速信号触发模式
        bool SetDecelerationSignalMode(const DECELERATION_SIGNAL_MODE mode);
        const DECELERATION_SIGNAL_MODE& DecelerationSignalMode();

        //设置/获取限位信号触发模式
        bool SetLimitSignalMode(const LIMIT_SIGNAL_MODE mode);
        const LIMIT_SIGNAL_MODE& LimitSignalMode();

        //设置/获取报警信号触发模式
        bool SetAlarmSignalMode(const ALARM_SIGNAL_MODE mode);
        const ALARM_SIGNAL_MODE& AlarmSignalMode();

        const SingleAxisFlag& AxisFlag();

        bool AxisStop();

        bool DecelerationSignal();

        bool PositiveLimitSignal();

        bool NegativeLimitSignal();

        bool OriginSignal();

        bool AlarmSignal();

        static const AllAxisSignalFlag& DecelerationAndLimitAndOriginSignal(const int card);

    private:
        static bool _initSuccess;
        int _axisChannel;

        long _position;

        double _moveSpeed;
        TrapezaidalSpeed _fastMoveSpeed;

        double _interpolationMoveSpeed;
        TrapezaidalSpeed _interpolationFastMoveSpeed;

        double _maxSpeed;

        PositionTrigger _positionTriggerPoint;

        MOVE_MODE _moveMode;
        ORIGIN_DETECTION_MODE _originDetectionMode;

        ORIGIN_SIGNAL_FLAG _originSignalFlag;
        DECELERATION_SIGNAL_FLAG _decelerationSignalFlag;
        LILMIT_SIGNAL_FLAG _limitSignalFlag;

        POSITION_TRIGGER_FLAG _positionTriggerFlag;

        ORIGIN_SIGNAL_MODE _originSignalMode;
        DECELERATION_SIGNAL_MODE _decelerationSignalMode;
        LIMIT_SIGNAL_MODE _limitSignalMode;
        
        ALARM_SIGNAL_MODE _alarmSignalMode;

        SingleAxisFlag _saf;

        static AllAxisSignalFlag _aasf;
    };

}

#endif // MOTORCOMPONENT_H
