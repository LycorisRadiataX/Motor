/*
 * Detail : MPC08运动控制卡C++封装, 编译时最低C++标准为C++11
 * Author : LycorisRadiata
 * Date   : 2020.9.15
 * Ver    : v2.0.0.D.1119
*/

#ifndef MOTORCOMPONENT_H
#define MOTORCOMPONENT_H

#include "Mpc.h"

namespace MotorComponent
{

    enum class MotorDataState : int
    {
        UNDEFINED = -1,
        DEFINED = 1,
        SUCCESS = 2,
        FAILURE = 3,
        PARAMETER_ERROR = 4
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
        MotorDataState InitDefaultSetting();
        
        //设置运动模式
        MotorDataState SetMoveMode(MoveMode mode);
        MotorDataState MoveMode(MoveMode* mode);

        //设置回原点时的检测模式
        MotorDataState SetOriginDetectionMode(OriginDetectionMode mode);
        MotorDataState OriginDetectionMode(OriginDetectionMode* mode);

        //设置和获取运动和连续运动速度
        MotorDataState SetMoveSpeed(double speed);
        MotorDataState MoveSpeed(double* speed);

        //设置和获取连续运动梯形速度
        MotorDataState SetFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed);
        MotorDataState SetFastMoveSpeed(TrapezaidalSpeed* ts);
        MotorDataState FastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取插补运动速度
        MotorDataState SetInterpolationMoveSpeed(double Speed);
        MotorDataState InterpolationMoveSpeed(double* speed);

        //设置和获取插补连续运动速度
        MotorDataState SetInterpolationFastMoveSpeed(double startingSpeed, double targetSpeed, double accelerationSpeed);
        MotorDataState SetInterpolationFastMoveSpeed(TrapezaidalSpeed* ts);
        MotorDataState InterpolationFastMoveSpeed(TrapezaidalSpeed* ts);

        //设置和获取最大速度
        MotorDataState SetMaxSpeed(double speed);
        MotorDataState MaxSpeed(double* speed);

        //获取运行时实际速度
        MotorDataState RunningSpeed(double* speed);

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
