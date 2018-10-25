#include "vexArm.h"

vexArm::vexArm()
{
    outputCap = 255;
    output = finalOutput = publicOutput = 0;
    currentState = off;
}

void vexArm::attachPin(int pin)
{
    myMotor.attach(pin);
}

void vexArm::setTheta(int theta)
{
    setpoint = theta;
}

void vexArm::setProcessVariable(int input)
{
    processVariable = input;
}

void vexArm::setMaxSpeed(int maxSpeed)
{
    outputCap = maxSpeed;
}

void vexArm::setGains(float uinKp, float uinKi, float uinKd,
                      float dinKp, float dinKi, float dinKd,
                      float inKv)
{
    //gains going up
    ukp = uinKp;
    uki = uinKi;
    ukd = uinKd;
    //gains going down
    dkp = dinKp;
    dki = dinKi;
    dkd = dinKd;
    //feedforward gain
    kv = inKv;
    //control loop
    pid();
}

void vexArm::writeMotorPublic(int value)
{
    publicOutput = value;
    writeMotor();
}

void vexArm::setMotorState(motorStates state)
{
    currentState = state;
}

int vexArm::retrieveError()
{
    return error;
}

float vexArm::getP()
{
    return kp;
}

motorStates vexArm::getMotorState()
{
    return currentState;
}

/*Private functions defined below*/

void vexArm::pid()
{
    error = setpoint - processVariable;
    //gain scheduler
    if(error >= 0)
    {
        kp = ukp;
        ki = uki;
        kd = ukd;
    }
    if(error < 0)
    {
        kp = dkp;
        ki = dki;
        kd = dkd;
    }
    //anti windup for integral
    if(setpoint != lastSetpoint || abs(error) > 20)
    {
        I = lastError = 0;
    }
    //P, I, and D calculations with integral cap
    P = kp * error;
    I += (ki * error);
    if(I > integralCap)
    {
        I = integralCap;
    }
    if(I < -integralCap)
    {
        I = -integralCap;
    }
    D = (error - lastError) * kd;
    lastSetpoint = setpoint;
    lastError = error;
    //prevent byte overflow
    if(output > 255)
    {
        output = 255;
    }
    if(output < -255)
    {
        output = -255;
    }
    //sum terms and write to motor
    output = P+I+D;
    writeMotor();
}

void vexArm::motionProfileGen(float maxA, float maxV, float target)
{
    mpError = target - mpPos;

    if(target != oldTarget)
    {
        mpCurrState = accel;
        mpPos = mpVel = 0;
        targetChange = true;
    }
    if(target == 0)
    {
        maxA = 0;
    }
    if(target > 0)
    {
        mpForward = true;
        maxA = maxA;
    }
    if(target < 0)
    {
        mpForward = false;
        maxA = maxA;
    }
    if(abs(mpVel) >= maxV && mpCurrState != deccel)
    {
        mpCurrState = cruise;
    }
    if(mpCurrState == cruise && lastMpState == accel)
    {
        accelDist = mpPos;
    }

    switch(mpCurrState)
    {
    default:
        if(abs((target/2)) > abs(mpPos))
        {
            mpCurrState = deccel;
        }
        if(mpForward)
        {
            maxA = maxA;
        }
        if(!mpForward)
        {
            maxA = (-1 * maxA);
        }

        mpVel += ((maxA/1000) * 25);
        mpPos += ((mpVel/1000) * 25);

        break;

    case accel:
        if(abs(mpPos) > abs((target/2)))
        {
            mpCurrState = deccel;
        }
        if(mpForward)
        {
            maxA = maxA;
        }
        if(!mpForward)
        {
            maxA = (-1 * maxA);
        }

        mpVel += ((maxA/1000) * 25);
        mpPos += ((mpVel/1000) * 25);
        break;

    case cruise:
        if(mpForward)
        {
            if(mpError <= accelDist)
            {
                mpCurrState = deccel;
            }
        }
        if(!mpForward)
        {
            if(mpError >= accelDist)
            {
                mpCurrState = deccel;
            }
        }

        mpPos += ((mpVel/1000) * 25);
        break;

    case deccel:
        if(mpForward)
        {
            maxA = (-1 * maxA);
            if(mpVel < 0)
            {
                mpCurrState = rest;
            }
        }
        if(!mpForward)
        {
            maxA = maxA;

            if(mpVel > 0)
            {
                mpCurrState = rest;
            }
        }

        mpVel += (maxA/1000) * 25;
        mpPos += (maxV/1000) * 25;
        break;

    case rest:
        mpVel = 0;
        mpPos = mpPos;
        break;
    } //end of switch-case stucture

    oldTarget = target;
    lastMpState = mpCurrState;
} //end of function

void vexArm::pidFF()
{
    error = mpPos - processVariable;
    if(targetChange)
    {
        I = lastError = 0;
    }
    P = kp * error;
    I += (ki * error);
    if(I > integralCap)
    {
        I = integralCap;
    }
    if(I < -integralCap)
    {
        I = -integralCap;
    }
    D = (error - lastError) * kd;
    V = kv * mpVel;
    lastSetpoint = setpoint;
    lastError = error;

    if(output > 255)
    {
        output = 255;
    }
    if(output < -255)
    {
        output = -255;
    }

    output = P+I+D+V;
    writeMotor();
}

void vexArm::writeMotor()
{
    if(currentState == off)
    {
        finalOutput = 0;
    }
    if(currentState == manual)
    {
        finalOutput = publicOutput;
    }
    if(currentState == on)
    {
        finalOutput = output;
    }
    myMotor.write(finalOutput);
}
