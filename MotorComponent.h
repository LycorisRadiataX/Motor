/*
 * Detail : MPC08运动控制卡C++封装, 编译时最低C++标准为C++11
 * Author : LycorisRadiata
 * Date   : 2020.9.15
 * Ver    : v2.1.1.D.1503
*/

#ifndef MOTORCOMPONENT_H_
#define MOTORCOMPONENT_H_

#include "Mpc.h"

namespace MotorComponent
{

    enum class MotorControlState : int
    {
        UNDEFINED = -1,
        DEFINED = 1,
        SUCCESS = 2,
        FAILURE = 3,
        PARAMETER_ERROR = 4
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

    struct TrapezaidalSpeed
    {
        double startingSpeed;
        double targetSpeed;
        double accelerationSpeed;
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

        MotorControlState Move(long distance, MoveDirection direction);

        MotorControlState ContinuousMove(MoveDirection direction);

        MotorControlState FastMove(long distance, MoveDirection direction);

        MotorControlState FastContinuousMove(MoveDirection direction);

        MotorControlState OriginMove(MoveDirection direction);

        MotorControlState FastOriginMove(MoveDirection direction);

        MotorControlState SetAbsolutePosition(long position);

        MotorControlState ResetPosition();

        MotorControlState SetPositionTriggerPoint(long startingPosition, long targetPosition);
    private:
        static bool _initSuccess;
        int _axisChannel;
        enum MoveMode _moveMode;
        enum OriginDetectionMode _originDetectionMode;
        double _moveSpeed;
        TrapezaidalSpeed _fastMoveSpeed;
        double _interpolationMoveSpeed;
        TrapezaidalSpeed _interpolationFastMoveSpeed;
        double _maxSpeed;
    };

}

#endif // MOTORCOMPONENT_H
