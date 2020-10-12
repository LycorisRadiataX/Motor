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
        Motor_Control_State InitDefaultSetting();
        
        //设置运动模式
        Motor_Control_State SetMoveMode(const MoveMode mode);
        Motor_Control_State MoveMode(MoveMode* mode);

        //设置回原点时的检测模式
        Motor_Control_State SetOriginDetectionMode(const OriginDetectionMode mode);
        Motor_Control_State OriginDetectionMode(OriginDetectionMode* mode);

        //设置和获取运动和连续运动速度
        Motor_Control_State SetMoveSpeed(const double speed);
        Motor_Control_State MoveSpeed(double* speed);

        //设置和获取连续运动梯形速度
        Motor_Control_State SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        Motor_Control_State SetFastMoveSpeed(const TrapezaidalSpeed* const ts);
        Motor_Control_State FastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取插补运动速度
        Motor_Control_State SetInterpolationMoveSpeed(const double Speed);
        Motor_Control_State InterpolationMoveSpeed(double* speed);

        //设置和获取插补连续运动速度
        Motor_Control_State SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        Motor_Control_State SetInterpolationFastMoveSpeed(const TrapezaidalSpeed* const ts);
        Motor_Control_State InterpolationFastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取最大速度
        Motor_Control_State SetMaxSpeed(const double speed);
        Motor_Control_State MaxSpeed(double* speed);

        //获取运行时实际速度
        Motor_Control_State RunningSpeed(double* speed);

        //移动
        Motor_Control_State Move(const long distance);

        //连续移动
        Motor_Control_State ContinuousMove(const Move_Direction direction);

        //快速移动
        Motor_Control_State FastMove(const long distance);

        //快速连续移动
        Motor_Control_State FastContinuousMove(const Move_Direction direction);

        //回原点移动
        Motor_Control_State OriginMove(const Move_Direction direction);

        //快速回原点移动
        Motor_Control_State FastOriginMove(const Move_Direction direction);

        //停止运动
        Motor_Control_State StopMove();

        //停止快速运动
        Motor_Control_State StopFastMove();

        //设置当前绝对位置, 但设置后点击不会进行实际运动
        Motor_Control_State SetAbsolutePosition(const long position);

        //获取当前绝对位置
        Motor_Control_State AbsolutePosition(long* position);

        //获取当前相对位置值
        Motor_Control_State RelativelyPosition(long* position);

        //重置位置值
        Motor_Control_State ResetPosition();

        //设置/获取位置触发点, 位置进入起始点时，自动触发输出 IO 信号（低电平）；当位置走出比较终止点时，自动触发输出高电平
        Motor_Control_State SetPositionTriggerPoint(const long startingPosition, const long targetPosition);
        Motor_Control_State PositionTriggerPoint(long* startingPosition, long* targetPosition);

        //设置/获取位置触发是否有效
        Motor_Control_State SetPositionTriggerIsValid(const int card, const Position_Trigger_Flag flag);
        Motor_Control_State PositionTriggerIsValid(Position_Trigger_Flag* flag);

        //设置/获取减速信号是否有效
        Motor_Control_State SetDecelerationSignalIsValid(const Deceleration_Signal_Flag flag);
        Motor_Control_State DecelerationSignalIsValid(Deceleration_Signal_Flag* flag);

        //设置/获取限位信号是否有效
        Motor_Control_State SetLimitSignalIsValid(const Limit_Signal_Flag flag);
        Motor_Control_State LimitSignalIsValid(Limit_Signal_Flag* flag);

        //设置/获取原点信号是否有效
        Motor_Control_State SetOriginSignalIsValid(const Origin_Signal_Flag flag);
        Motor_Control_State OriginSignalIsValid(Origin_Signal_Flag* flag);

        //设置/获取减速信号触发模式
        Motor_Control_State SetDecelerationSignalMode(const Deceleration_Signal_Mode mode);
        Motor_Control_State DecelerationSignalMode(Deceleration_Signal_Mode* mode);

        //设置/获取限位信号触发模式
        Motor_Control_State SetLimitSignalMode(const Limit_Signal_Mode mode);
        Motor_Control_State LimitSignalMode(Limit_Signal_Mode* mode);

        //设置/获取原点信号触发模式
        Motor_Control_State SetOriginSignalMode(const Origin_Signal_Mode mode);
        Motor_Control_State OriginSignalMode(Origin_Signal_Mode* mode);

        //设置/获取报警信号触发模式
        Motor_Control_State SetAlarmSignalMode(const Alarm_Signal_Mode mode);
        Motor_Control_State AlarmSignalMode(Alarm_Signal_Mode* mode);

        SingleAxisFlag AxisFlag(int* flag);

        Motor_Control_State AxisStop(bool* stop);

        Motor_Control_State DecelerationSignal(bool* signal);

        Motor_Control_State LimitSignal(bool* signal);

        Motor_Control_State OriginSignal(bool* signal);

        Motor_Control_State AlarmSignal(bool* signal);

        static Motor_Control_State DecelerationAndLimitAndOriginSignal(const int card, int* flag);

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
