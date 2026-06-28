#include "RobotController.h"

RobotController::RobotController(Motor *motors[4], const RobotParams &params,
                                 Mcp23017Bus *mcp, uint8_t redLedPin, uint8_t greenLedPin)
    : _motors(motors), _params(params), _mcp(mcp), _redLedPin(redLedPin), _greenLedPin(greenLedPin), _command{0, 0}, _enabled(false), _wheelCircumference(0), _pidConfig{0.2f, 0.01f, 0.00f, 0.3f, 0.5f}, _pidEnabled(true), _lastUpdateTime(0), _targetLeftSpeed(0), _targetRightSpeed(0)
{
    updateWheelCircumference();

    // Initialize PID controllers with default gains
    // PID outputs corrections, not absolute duty, so limit output range
    PIDGains gains;
    gains.Kp = _pidConfig.Kp;
    gains.Ki = _pidConfig.Ki;
    gains.Kd = _pidConfig.Kd;
    gains.maxIntegral = _pidConfig.maxIntegral;
    gains.maxOutput = _pidConfig.maxCorrection;
    gains.minOutput = -_pidConfig.maxCorrection;

    _frontLeftPID.setGains(gains);
    _rearLeftPID.setGains(gains);
    _frontRightPID.setGains(gains);
    _rearRightPID.setGains(gains);
    if (isEnabled())
    {
        setLEDs(false, true);
    }
    else
    {
        setLEDs(true, false);
    }
}

void RobotController::begin()
{
    _command = {0, 0};
    _enabled = false;
    _targetLeftSpeed = 0;
    _targetRightSpeed = 0;
    _lastUpdateTime = micros();
    updateWheelCircumference();
    resetPID();
}

void RobotController::setParams(const RobotParams &params)
{
    _params = params;
    updateWheelCircumference();
}

void RobotController::updateWheelCircumference()
{
    _wheelCircumference = _params.wheelDiameterM * PI;
}

void RobotController::setPIDGains(const SpeedPIDConfig &config)
{
    _pidConfig = config;

    PIDGains gains;
    gains.Kp = config.Kp;
    gains.Ki = config.Ki;
    gains.Kd = config.Kd;
    gains.maxIntegral = config.maxIntegral;
    gains.maxOutput = config.maxCorrection;
    gains.minOutput = -config.maxCorrection;

    _frontLeftPID.setGains(gains);
    _rearLeftPID.setGains(gains);
    _frontRightPID.setGains(gains);
    _rearRightPID.setGains(gains);
}

void RobotController::setPIDGains(float Kp, float Ki, float Kd)
{
    _pidConfig.Kp = Kp;
    _pidConfig.Ki = Ki;
    _pidConfig.Kd = Kd;

    _frontLeftPID.setGains(Kp, Ki, Kd);
    _rearLeftPID.setGains(Kp, Ki, Kd);
    _frontRightPID.setGains(Kp, Ki, Kd);
    _rearRightPID.setGains(Kp, Ki, Kd);
}

void RobotController::resetPID()
{
    _frontLeftPID.reset();
    _rearLeftPID.reset();
    _frontRightPID.reset();
    _rearRightPID.reset();

    _leftFrontDuty  = 0;
    _leftBackDuty   = 0;
    _rightFrontDuty = 0;
    _rightBackDuty  = 0;
}

float RobotController::mpsToEncoderCPS(float mps) const
{
    // Convert m/s to encoder counts per second
    // wheel rotations per second = mps / circumference
    // encoder counts per second = rotations per second * CPR
    float rotationsPerSec = mps / _wheelCircumference;
    return rotationsPerSec * _params.encoderCPR;
}

float RobotController::encoderCPSToMPS(float cps) const
{
    // Convert encoder counts per second to m/s
    float rotationsPerSec = cps / _params.encoderCPR;
    return rotationsPerSec * _wheelCircumference;
}

float RobotController::mpsToDuty(float mps) const
{
    // Convert m/s to duty cycle (-1 to 1)
    // Assumes max speed corresponds to duty = 1.0
    if (_params.maxSpeedMPS <= 0)
        return 0;
    float duty = mps / _params.maxSpeedMPS;

    // Clamp to valid range
    if (duty > 1.0f)
        duty = 1.0f;
    if (duty < -1.0f)
        duty = -1.0f;

    return duty;
}

void RobotController::setSpeed(float linearMPS, float angularRadPS)
{
    // Clamp linear speed to max
    if (linearMPS > _params.maxSpeedMPS)
        linearMPS = _params.maxSpeedMPS;
    if (linearMPS < -_params.maxSpeedMPS)
        linearMPS = -_params.maxSpeedMPS;

    _command.linearSpeed = linearMPS;
    _command.angularSpeed = angularRadPS;

    // Calculate target wheel speeds using differential drive kinematics
    float halfWheelBase = _params.wheelBaseM / 2.0f;
    _targetLeftSpeed = linearMPS - (angularRadPS * halfWheelBase);
    _targetRightSpeed = linearMPS + (angularRadPS * halfWheelBase);
}

void RobotController::setWheelSpeeds(float leftMPS, float rightMPS)
{
    _targetLeftSpeed  = leftMPS;
    _targetRightSpeed = rightMPS;
    // Keep _command in sync for telemetry/dashboard
    _command.linearSpeed  = (leftMPS + rightMPS) / 2.0f;
    _command.angularSpeed = (rightMPS - leftMPS) / _params.wheelBaseM;
}

void RobotController::forward(float speedMPS)
{
    setSpeed(speedMPS, 0);
}

void RobotController::backward(float speedMPS)
{
    setSpeed(-speedMPS, 0);
}

void RobotController::turnLeft(float speedMPS)
{
    // Turn in place: left wheels backward, right wheels forward
    setSpeed(0, speedMPS / (_params.wheelBaseM / 2.0f));
}

void RobotController::turnRight(float speedMPS)
{
    // Turn in place: left wheels forward, right wheels backward
    setSpeed(0, -speedMPS / (_params.wheelBaseM / 2.0f));
}

void RobotController::stop()
{
    _command = {0, 0};
    _targetLeftSpeed = 0;
    _targetRightSpeed = 0;
    resetPID();

    for (int i = 0; i < 4; i++)
    {
        _motors[i]->setDuty(0);
    }
}

void RobotController::brake()
{
    _command = {0, 0};
    _targetLeftSpeed = 0;
    _targetRightSpeed = 0;
    resetPID();

    for (int i = 0; i < 4; i++)
    {
        _motors[i]->brake();
    }
}

void RobotController::update()
{
    // Calculate dt from last update
    uint32_t now = micros();
    float dt = (now - _lastUpdateTime) / 1000000.0f; // Convert to seconds
    _lastUpdateTime = now;

    // Clamp dt to reasonable range (avoid issues on first call or timing glitches)
    if (dt <= 0 || dt > 0.1f)
    {
        dt = 0.01f; // Default to 10ms
    }

    update(dt);
}

void RobotController::update(float dt)
{
    if (!_enabled)
    {
        return;
    }

    if (_pidEnabled)
    {
        applyPIDControl(dt);
    }
    else
    {
        applyOpenLoopControl();
    }
}

void RobotController::applyPIDControl(float dt)
{
    // float scale_factor = 1.0f;
    // Feedforward initialisation — seed duty from target speed on first non-zero cycle
    if (_leftFrontDuty == 0)  _leftFrontDuty  = mpsToDuty(_targetLeftSpeed);
    if (_leftBackDuty == 0)   _leftBackDuty   = mpsToDuty(_targetLeftSpeed);
    if (_rightFrontDuty == 0) _rightFrontDuty = mpsToDuty(_targetRightSpeed);
    if (_rightBackDuty == 0)  _rightBackDuty  = mpsToDuty(_targetRightSpeed);

    // PID corrections — each wheel tracked independently
    _leftFrontDuty  += _frontLeftPID.compute(_targetLeftSpeed,  getFrontLeftWheelSpeed(),  dt);
    _leftBackDuty   += _rearLeftPID.compute(_targetLeftSpeed,   getRearLeftWheelSpeed(),   dt);
    _rightFrontDuty += _frontRightPID.compute(_targetRightSpeed, getFrontRightWheelSpeed(), dt);
    _rightBackDuty  += _rearRightPID.compute(_targetRightSpeed,  getRearRightWheelSpeed(),  dt);

    // Zero duty immediately when target is zero
    if (_targetLeftSpeed == 0)  { _leftFrontDuty  = 0; _leftBackDuty  = 0; }
    if (_targetRightSpeed == 0) { _rightFrontDuty = 0; _rightBackDuty = 0; }

    // Clamp all to valid range
    if (_leftFrontDuty  > 1.0f) _leftFrontDuty  = 1.0f;
    if (_leftFrontDuty  < -1.0f) _leftFrontDuty = -1.0f;
    if (_leftBackDuty   > 1.0f) _leftBackDuty   = 1.0f;
    if (_leftBackDuty   < -1.0f) _leftBackDuty  = -1.0f;
    if (_rightFrontDuty > 1.0f) _rightFrontDuty = 1.0f;
    if (_rightFrontDuty < -1.0f) _rightFrontDuty = -1.0f;
    if (_rightBackDuty  > 1.0f) _rightBackDuty  = 1.0f;
    if (_rightBackDuty  < -1.0f) _rightBackDuty = -1.0f;

    // Apply to motors
    _motors[FRONT_LEFT]->setDuty(_leftFrontDuty);
    _motors[REAR_LEFT]->setDuty(_leftBackDuty);
    _motors[FRONT_RIGHT]->setDuty(_rightFrontDuty);
    _motors[REAR_RIGHT]->setDuty(_rightBackDuty);
}

void RobotController::applyOpenLoopControl()
{
    // Simple open-loop: convert target speed directly to duty
    float leftDuty = mpsToDuty(_targetLeftSpeed);
    float rightDuty = mpsToDuty(_targetRightSpeed);

    _motors[FRONT_LEFT]->setDuty(leftDuty);
    _motors[REAR_LEFT]->setDuty(leftDuty);
    _motors[FRONT_RIGHT]->setDuty(rightDuty);
    _motors[REAR_RIGHT]->setDuty(rightDuty);
}

float RobotController::getFrontLeftWheelSpeed() const
{
    return encoderCPSToMPS(_motors[FRONT_LEFT]->getEncoderVelocity());
}

float RobotController::getRearLeftWheelSpeed() const
{
    return encoderCPSToMPS(_motors[REAR_LEFT]->getEncoderVelocity());
}

float RobotController::getFrontRightWheelSpeed() const
{
    return encoderCPSToMPS(_motors[FRONT_RIGHT]->getEncoderVelocity());
}

float RobotController::getRearRightWheelSpeed() const
{
    return encoderCPSToMPS(_motors[REAR_RIGHT]->getEncoderVelocity());
}

float RobotController::getLeftWheelSpeed() const
{
    return (getFrontLeftWheelSpeed() + getRearLeftWheelSpeed()) / 2.0f;
}

float RobotController::getRightWheelSpeed() const
{
    return (getFrontRightWheelSpeed() + getRearRightWheelSpeed()) / 2.0f;
}

float RobotController::getActualLinearSpeed() const
{
    // Average of left and right wheel speeds
    return (getLeftWheelSpeed() + getRightWheelSpeed()) / 2.0f;
}

float RobotController::getActualAngularSpeed() const
{
    float leftMPS = getLeftWheelSpeed();
    float rightMPS = getRightWheelSpeed();

    // Angular velocity = (rightSpeed - leftSpeed) / wheelBase
    return (rightMPS - leftMPS) / _params.wheelBaseM;
}

int64_t RobotController::getEncoderCount(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex > 3)
        return 0;
    return _motors[motorIndex]->getEncoderCount();
}

float RobotController::getMotorCurrent(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex > 3)
        return 0;
    return _motors[motorIndex]->readCurrentAmps();
}

uint16_t RobotController::getMotorPWM(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex > 3)
        return 0;
    return _motors[motorIndex]->getPWM();
}

float RobotController::getMotorDuty(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex > 3)
        return 0;
    return _motors[motorIndex]->getDuty();
}

void RobotController::setLEDs(bool red, bool green)
{
    if (_mcp)
    {
        _mcp->writePin(_redLedPin, red);
        _mcp->writePin(_greenLedPin, green);
    }
}

void RobotController::enable(bool en)
{
    _enabled = en;
    for (int i = 0; i < 4; i++)
    {
        _motors[i]->enable(en);
    }

    if (en)
    {
        // Reset PID when enabling
        resetPID();
        _lastUpdateTime = micros();
        setLEDs(false, true); // green on, red off
    }
    else
    {
        _command = {0, 0};
        _targetLeftSpeed = 0;
        _targetRightSpeed = 0;
        setLEDs(true, false); // red on, green off
    }
}
