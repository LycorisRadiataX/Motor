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
        MotorControlState SetMoveMode(MoveMode mode);
        MotorControlState MoveMode(MoveMode* mode);

        //设置回原点时的检测模式
        MotorControlState SetOriginDetectionMode(OriginDetectionMode mode);
        MotorControlState OriginDetectionMode(OriginDetectionMode* mode);

        //设置和获取运动和连续运动速度
        MotorControlState SetMoveSpeed(double speed);
        MotorControlState MoveSpeed(double* speed);

        //设置和获取连续运动梯形速度
        MotorControlState SetFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed);
        MotorControlState SetFastMoveSpeed(TrapezaidalSpeed* ts);
        MotorControlState FastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取插补运动速度
        MotorControlState SetInterpolationMoveSpeed(double Speed);
        MotorControlState InterpolationMoveSpeed(double* speed);

        //设置和获取插补连续运动速度
        MotorControlState SetInterpolationFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed);
        MotorControlState SetInterpolationFastMoveSpeed(TrapezaidalSpeed* ts);
        MotorControlState InterpolationFastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取最大速度
        MotorControlState SetMaxSpeed(double speed);
        MotorControlState MaxSpeed(double* speed);

        //获取运行时实际速度
        MotorControlState RunningSpeed(double* speed);

        MotorControlState Move(long distance);

        MotorControlState ContinuousMove(MoveDirection direction);

        MotorControlState FastMove(long distance);

        MotorControlState FastContinuousMove(MoveDirection direction);

        MotorControlState OriginMove(MoveDirection direction);

        MotorControlState FastOriginMove(MoveDirection direction);

        MotorControlState SetAbsolutePosition(long position);

        MotorControlState AbsolutePosition(long* position);

        MotorControlState RelativelyPosition(long* position);

        MotorControlState ResetPosition();

        MotorControlState SetPositionTriggerPoint(long startingPosition, long targetPosition);

        MotorControlState SetPositionTriggerIsValid(int card, PositionTriggerFlag flag);

        MotorControlState SetDecelerationSignalIsValid(DecelerationSignalFlag flag);

        MotorControlState SetLimitSignalIsValid(LimitSignalFlag flag);

        MotorControlState SetOriginSignalIsValid(OriginSignalFlag flag);

        MotorControlState SetDecelerationSignalMode(DecelerationSignalMode mode);

        MotorControlState SetLimitSignalMode(LimitSignalMode mode);

        MotorControlState SetOriginSignalMode(OriginSignalMode mode);

        MotorControlState SetAlarmSignalMode(AlarmSignalMode mode);
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
    };

}

#endif // MOTORCOMPONENT_H
