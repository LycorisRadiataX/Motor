/*
 * Detail : MPC08运动控制卡C++封装, 编译时最低C++标准为C++11
 * Author : LycorisRadiata
 * Date   : 2020.9.15
 * Ver    : v2.3.2.D.170201
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

    enum class Motor_Control_State : int
    {
        Undefine = 0x7FFFFFFF,
        Define = 0x7FFFFFFE,
        Success = 0x7FFFFFFD,
        Failure = 0x7FFFFFFC,
        Paramter_Error = 0x7FFFFFFB
    };

    enum class Move_Direction : int
    {
        Positive_Direction = 1,
        Negative_Direction = -1
    };

    enum class Move_Mode : int
    {
        Pulse_And_Direction = 1,
        Double_Pulse = 0
    };

    enum class Origin_Detection_Mode : int
    {
        Switch_Signal = 0,
        Switch_Signal_And_Encoder_Signal = 1
    };

    enum class Position_Trigger_Flag : int
    {
        Signal_Valid = 1,
        Signal_Invalid = 0
    };

    enum class Deceleration_Signal_Flag : int
    {
        Signal_Valid = 1,
        Signal_Invalid = 0
    };

    enum class Limit_Signal_Flag : int
    {
        Signal_Valid = 1,
        Signal_Invalid = 0
    };

    enum class Origin_Signal_Flag : int
    {
        Signal_Valid = 1,
        Signal_Invalid = 0
    };

    enum class Deceleration_Signal_Mode : int
    {
        High_Level = 1,
        Low_Level = 0
    };

    enum class Limit_Signal_Mode : int
    {
        High_Level = 1,
        Low_Level = 0
    };

    enum class Origin_Signal_Mode : int
    {
        High_Level = 1,
        Low_Level = 0
    };

    enum class Alarm_Signal_Mode : int
    {
        High_Level = 1,
        Low_Level = 0
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

    class Motor
    {
    public:
        explicit Motor(int axisChannel);
        ~Motor();
        Motor(Motor&) = delete;
        Motor(Motor&&) = delete;
        Motor& operator = (Motor&) = delete;
        Motor& operator = (Motor&) = delete;

        //硬件初始化, 全局调用一次即可
        static bool Init();

        //获取总轴数
        int Axis();

        //获取总板卡数
        int Board();

        //初始化为默认设置
        void InitDefaultSetting();
        
        //设置运动模式
        Motor_Control_State SetMoveMode(const Move_Mode mode);
        Move_Mode MoveMode();

        //设置回原点时的检测模式
        bool SetOriginDetectionMode(const Origin_Detection_Mode mode);
        Origin_Detection_Mode OriginDetectionMode();

        //设置和获取运动和连续运动速度
        bool SetMoveSpeed(const double speed);
        double MoveSpeed();

        //设置和获取连续运动梯形速度
        bool SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        bool SetFastMoveSpeed(const TrapezaidalSpeed* const ts);
        TrapezaidalSpeed FastMoveSpeed();

        //设置和获取插补运动速度
        bool SetInterpolationMoveSpeed(const double Speed);
        double InterpolationMoveSpeed();

        //设置和获取插补连续运动速度
        bool SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        bool SetInterpolationFastMoveSpeed(const TrapezaidalSpeed* const ts);
        TrapezaidalSpeed InterpolationFastMoveSpeed();

        //设置和获取最大速度
        bool SetMaxSpeed(const double speed);
        double MaxSpeed();

        //获取运行时实际速度
        double RunningSpeed();

        //移动
        bool Move(const long distance);

        //连续移动
        bool ContinuousMove(const Move_Direction direction);

        //快速移动
        bool FastMove(const long distance);

        //快速连续移动
        bool FastContinuousMove(const Move_Direction direction);

        //回原点移动
        bool OriginMove(const Move_Direction direction);

        //快速回原点移动
        bool FastOriginMove(const Move_Direction direction);

        //停止运动
        bool StopMove();

        //停止快速运动
        bool StopFastMove();

        //设置当前绝对位置, 但设置后点击不会进行实际运动
        bool SetAbsolutePosition(const long position);

        //获取当前绝对位置
        long AbsolutePosition();

        //获取当前相对位置值
        long RelativelyPosition();

        //重置位置值
        bool ResetPosition();

        //设置/获取位置触发点, 位置进入起始点时，自动触发输出 IO 信号（低电平）；当位置走出比较终止点时，自动触发输出高电平
        bool SetPositionTriggerPoint(const long startingPosition, const long targetPosition);
        bool SetPositionTriggerPoint(const PositionTrigger* pt);
        PositionTrigger PositionTriggerPoint();

        //设置/获取位置触发是否有效
        bool SetPositionTriggerIsValid(const int card, const Position_Trigger_Flag flag);
        Position_Trigger_Flag PositionTriggerIsValid();

        //设置/获取减速信号是否有效
        bool SetDecelerationSignalIsValid(const Deceleration_Signal_Flag flag);
        Deceleration_Signal_Flag DecelerationSignalIsValid();

        //设置/获取限位信号是否有效
        bool SetLimitSignalIsValid(const Limit_Signal_Flag flag);
        Limit_Signal_Flag LimitSignalIsValid();

        //设置/获取原点信号是否有效
        bool SetOriginSignalIsValid(const Origin_Signal_Flag flag);
        Origin_Signal_Flag OriginSignalIsValid();

        //设置/获取减速信号触发模式
        bool SetDecelerationSignalMode(const Deceleration_Signal_Mode mode);
        Deceleration_Signal_Mode DecelerationSignalMode();

        //设置/获取限位信号触发模式
        bool SetLimitSignalMode(const Limit_Signal_Mode mode);
        Limit_Signal_Mode LimitSignalMode();

        //设置/获取原点信号触发模式
        bool SetOriginSignalMode(const Origin_Signal_Mode mode);
        Origin_Signal_Mode OriginSignalMode();

        //设置/获取报警信号触发模式
        bool SetAlarmSignalMode(const Alarm_Signal_Mode mode);
        Alarm_Signal_Mode AlarmSignalMode();

        SingleAxisFlag AxisFlag();

        bool AxisStop();

        bool DecelerationSignal();

        bool PositiveLimitSignal();

        bool NegativeLimitSignal();

        bool OriginSignal();

        bool AlarmSignal();

        static AllAxisSignalFlag DecelerationAndLimitAndOriginSignal(const int card);

    private:
        static bool _initSuccess;
        int _axisChannel;
        long _position;
        Move_Mode _moveMode;
        Origin_Detection_Mode _originDetectionMode;
        double _moveSpeed;
        TrapezaidalSpeed _fastMoveSpeed;
        double _interpolationMoveSpeed;
        TrapezaidalSpeed _interpolationFastMoveSpeed;
        double _maxSpeed;
        Position_Trigger_Flag _positionTriggerFlag;
        Deceleration_Signal_Flag _decelerationSignalFlag;
        Limit_Signal_Flag _limitSignalFlag;
        Origin_Signal_Flag _originSignalFlag;
        Deceleration_Signal_Mode _decelerationSignalMode;
        Limit_Signal_Mode _limitSignalMode;
        Origin_Signal_Mode _originSignalMode;
        Alarm_Signal_Mode _alarmSignalMode;
        long _startingPosition;
        long _targetPosition;
    };

}

#endif // MOTORCOMPONENT_H
