#ifndef VEXARM_H
#define VEXARM_H

#include "vexMotor.h"

typedef enum{
    off,
    manual,
    on
} motorStates;

class vexArm
{
public:
    vexArm();
    void attachPin(int pin);
    void setTheta(int theta);
    void setProcessVariable(int input);
    void setMaxSpeed(int maxSpeed);
    void setGains(float uinKp, float uinKi, float uinKd,
                  float dinKp, float dinKi, float dinKd,
                  float inKv);
    void writeMotorPublic(int value);
    void setMotorState(motorStates state);
    int retrieveError();
    float getP();
    motorStates getMotorState();
private:
    vexMotor myMotor;
    /*PID and feedforward variables*/
    float ukp, uki, ukd,
    dkp, dki, dkd,
    kp, ki, kd,
    kv;
    int setpoint, lastSetpoint, output, P, I, D, V,
    error, lastError, processVariable;
    const int integralCap = 95; //was 75
    int outputCap, finalOutput, publicOutput;
    motorStates currentState;
    /*Motion profile variables*/
    float mpVel, mpPos, oldTarget, accelDist, mpError;
    bool mpForward, targetChange = true;
    typedef enum{
        accel,
        cruise,
        deccel,
        rest
    }mpStates;
    mpStates mpCurrState, lastMpState;
    /*Private function prototypes*/
    void pid();
    void motionProfileGen(float maxA, float maxV, float target);
    void pidFF();
    void writeMotor();
};

#endif // VEXARM_H
