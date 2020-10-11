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

    enum class MotorControlState : int
    {
        UNDEFINED = 0x7FFFFFFF,
        DEFINED = 0x7FFFFFFE,
        SUCCESS = 0x7FFFFFFD,
        FAILURE = 0x7FFFFFFC,
        PARAMETER_ERROR = 0x7FFFFFFB
    };

    enum class MoveDirection : int
    {
        POSITIVE_DIRECTION = 1,
        NEGATIVE_DIRECTION = -1
    };

    enum class MoveMode : int
    {
        PULSE_AND_DIRECTION = 1,
        DOUBLE_PULSE = 0
    };

    enum class OriginDetectionMode : int
    {
        SWITCH_SIGNAL = 0,
        SWITCH_SIGNAL_AND_ENCODER_SIGNAL = 1
    };

    enum class PositionTriggerFlag : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0
    };

    enum class DecelerationSignalFlag : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0
    };

    enum class LimitSignalFlag : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0
    };

    enum class OriginSignalFlag : int
    {
        SIGNAL_VALID = 1,
        SIGNAL_INVALID = 0
    };

    enum class DecelerationSignalMode : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0
    };

    enum class LimitSignalMode : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0
    };

    enum class OriginSignalMode : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0
    };

    enum class AlarmSignalMode : int
    {
        HIGH_LEVEL = 1,
        LOW_LEVEL = 0
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
        MotorControlState InitDefaultSetting();
        
        //设置运动模式
        MotorControlState SetMoveMode(const enum MoveMode mode);
        MotorControlState MoveMode(enum MoveMode* mode);

        //设置回原点时的检测模式
        MotorControlState SetOriginDetectionMode(const enum OriginDetectionMode mode);
        MotorControlState OriginDetectionMode(enum OriginDetectionMode* mode);

        //设置和获取运动和连续运动速度
        MotorControlState SetMoveSpeed(const double speed);
        MotorControlState MoveSpeed(double* speed);

        //设置和获取连续运动梯形速度
        MotorControlState SetFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        MotorControlState SetFastMoveSpeed(const TrapezaidalSpeed* const ts);
        MotorControlState FastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取插补运动速度
        MotorControlState SetInterpolationMoveSpeed(const double Speed);
        MotorControlState InterpolationMoveSpeed(double* speed);

        //设置和获取插补连续运动速度
        MotorControlState SetInterpolationFastMoveSpeed(const double startingSpeed, const double targetSpeed, const double accelerationSpeed);
        MotorControlState SetInterpolationFastMoveSpeed(const TrapezaidalSpeed* const ts);
        MotorControlState InterpolationFastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取最大速度
        MotorControlState SetMaxSpeed(const double speed);
        MotorControlState MaxSpeed(double* speed);

        //获取运行时实际速度
        MotorControlState RunningSpeed(double* speed);

        //移动
        MotorControlState Move(const long distance);

        //连续移动
        MotorControlState ContinuousMove(const MoveDirection direction);

        //快速移动
        MotorControlState FastMove(const long distance);

        //快速连续移动
        MotorControlState FastContinuousMove(const MoveDirection direction);

        //回原点移动
        MotorControlState OriginMove(const MoveDirection direction);

        //快速回原点移动
        MotorControlState FastOriginMove(const MoveDirection direction);

        //停止运动
        MotorControlState StopMove();

        //停止快速运动
        MotorControlState StopFastMove();

        //设置当前绝对位置, 但设置后点击不会进行实际运动
        MotorControlState SetAbsolutePosition(const long position);

        //获取当前绝对位置
        MotorControlState AbsolutePosition(long* position);

        //获取当前相对位置值
        MotorControlState RelativelyPosition(long* position);

        //重置位置值
        MotorControlState ResetPosition();

        //设置/获取位置触发点, 位置进入起始点时，自动触发输出 IO 信号（低电平）；当位置走出比较终止点时，自动触发输出高电平
        MotorControlState SetPositionTriggerPoint(const long startingPosition, const long targetPosition);
        MotorControlState PositionTriggerPoint(long* startingPosition, long* targetPosition);

        //设置/获取位置触发是否有效
        MotorControlState SetPositionTriggerIsValid(const int card, const PositionTriggerFlag flag);
        MotorControlState PositionTriggerIsValid(PositionTriggerFlag* flag);

        //设置/获取减速信号是否有效
        MotorControlState SetDecelerationSignalIsValid(const DecelerationSignalFlag flag);
        MotorControlState DecelerationSignalIsValid(DecelerationSignalFlag* flag);

        //设置/获取限位信号是否有效
        MotorControlState SetLimitSignalIsValid(const LimitSignalFlag flag);
        MotorControlState LimitSignalIsValid(LimitSignalFlag* flag);

        //设置/获取原点信号是否有效
        MotorControlState SetOriginSignalIsValid(const OriginSignalFlag flag);
        MotorControlState OriginSignalIsValid(OriginSignalFlag* flag);

        //设置/获取减速信号触发模式
        MotorControlState SetDecelerationSignalMode(const enum DecelerationSignalMode mode);
        MotorControlState DecelerationSignalMode(enum DecelerationSignalMode* mode);

        //设置/获取限位信号触发模式
        MotorControlState SetLimitSignalMode(const enum LimitSignalMode mode);
        MotorControlState LimitSignalMode(enum LimitSignalMode* mode);

        //设置/获取原点信号触发模式
        MotorControlState SetOriginSignalMode(const enum OriginSignalMode mode);
        MotorControlState OriginSignalMode(enum OriginSignalMode* mode);

        //设置/获取报警信号触发模式
        MotorControlState SetAlarmSignalMode(const enum AlarmSignalMode mode);
        MotorControlState AlarmSignalMode(enum AlarmSignalMode* mode);

        MotorControlState AxisFlag(int* flag);

        MotorControlState AxisStop(bool* stop);

        MotorControlState DecelerationSignal(bool* signal);

        MotorControlState LimitSignal(bool* signal);

        MotorControlState OriginSignal(bool* signal);

        MotorControlState AlarmSignal(bool* signal);

        static MotorControlState DecelerationAndLimitAndOriginSignal(const int card, int* flag);

    private:
        static bool _initSuccess;
        int _axisChannel;
        long _position;
        enum MoveMode _moveMode;
        enum OriginDetectionMode _originDetectionMode;
        double _moveSpeed;
        TrapezaidalSpeed _fastMoveSpeed;
        double _interpolationMoveSpeed;
        TrapezaidalSpeed _interpolationFastMoveSpeed;
        double _maxSpeed;
        enum PositionTriggerFlag _positionTriggerFlag;
        enum DecelerationSignalFlag _decelerationSignalFlag;
        enum LimitSignalFlag _limitSignalFlag;
        enum OriginSignalFlag _originSignalFlag;
        enum DecelerationSignalMode _decelerationSignalMode;
        enum LimitSignalMode _limitSignalMode;
        enum OriginSignalMode _originSignalMode;
        enum AlarmSignalMode _alarmSignalMode;
        long _startingPosition;
        long _targetPosition;
    };

}

#endif // MOTORCOMPONENT_H
