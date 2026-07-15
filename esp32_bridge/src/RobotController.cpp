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
    clearWheelFaults();  // also resets PID
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
    // Clamp to the physical max so PID targets stay achievable — an
    // unreachable target winds the duty up to the clamp just like a dead
    // encoder would
    leftMPS  = constrain(leftMPS,  -_params.maxSpeedMPS, _params.maxSpeedMPS);
    rightMPS = constrain(rightMPS, -_params.maxSpeedMPS, _params.maxSpeedMPS);

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
    updateStatusLEDs();

    if (!_enabled)
    {
        return;
    }

    checkWheelFaults(dt);
    if (faultLatched())
    {
        // A wheel just stall-latched — don't drive this cycle; the motor
        // task sees faultLatched() on its next iteration and brakes.
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

void RobotController::checkWheelFaults(float dt)
{
    float speed[4];
    speed[FRONT_LEFT]  = getFrontLeftWheelSpeed();
    speed[REAR_LEFT]   = getRearLeftWheelSpeed();
    speed[FRONT_RIGHT] = getFrontRightWheelSpeed();
    speed[REAR_RIGHT]  = getRearRightWheelSpeed();

    for (int i = 0; i < 4; i++)
    {
        float duty = fabsf(_motors[i]->getDuty());

        switch (_wheelFault[i])
        {
        case WHEEL_OK:
            // Plausibility: driven hard but no measured motion. A hill can
            // slow a wheel, not hold it at zero against 30% duty.
            if (duty > FAULT_DUTY_THRESHOLD && fabsf(speed[i]) < FAULT_SPEED_FLOOR_MPS)
            {
                _faultTimer[i] += dt;
                if (_faultTimer[i] >= FAULT_DEBOUNCE_S)
                {
                    // Classify by current: a locked rotor pulls stall
                    // current; a dead encoder leaves the motor spinning
                    // at normal load current.
                    _wheelFault[i] = (_motors[i]->readCurrentAmps() >= STALL_CURRENT_A)
                                         ? WHEEL_STALL
                                         : WHEEL_LIMP;
                    _faultTimer[i] = 0;
                }
            }
            else
            {
                _faultTimer[i] = 0;
            }
            break;

        case WHEEL_LIMP:
            // Encoder untrusted, so stall protection for this wheel falls
            // back to current only.
            if (duty > FAULT_DUTY_THRESHOLD && _motors[i]->readCurrentAmps() >= STALL_CURRENT_A)
            {
                _stallTimer[i] += dt;
                if (_stallTimer[i] >= STALL_DEBOUNCE_S)
                {
                    _wheelFault[i] = WHEEL_STALL;
                    _stallTimer[i] = 0;
                }
            }
            else
            {
                _stallTimer[i] = 0;
            }
            break;

        case WHEEL_STALL:
        default:
            break;  // latched until clearWheelFaults()
        }
    }
}

bool RobotController::faultLatched() const
{
    for (int i = 0; i < 4; i++)
    {
        if (_wheelFault[i] == WHEEL_STALL)
            return true;
    }
    return false;
}

bool RobotController::anyWheelLimp() const
{
    for (int i = 0; i < 4; i++)
    {
        if (_wheelFault[i] == WHEEL_LIMP)
            return true;
    }
    return false;
}

bool RobotController::wheelOk(int motorIndex) const
{
    if (motorIndex < 0 || motorIndex > 3)
        return false;
    return _wheelFault[motorIndex] == WHEEL_OK;
}

void RobotController::clearWheelFaults()
{
    for (int i = 0; i < 4; i++)
    {
        _wheelFault[i] = WHEEL_OK;
        _faultTimer[i] = 0;
        _stallTimer[i] = 0;
    }
    resetPID();
}

void RobotController::updateStatusLEDs()
{
    bool red, green;
    if (faultLatched())
    {
        red = true;  green = false;    // stall latched — robot braked
    }
    else if (anyWheelLimp())
    {
        red = true;  green = true;     // limp mode — degraded but driving
    }
    else if (_enabled)
    {
        red = false; green = true;
    }
    else
    {
        red = true;  green = false;
    }

    // Only write the MCP on a change — this runs every control cycle
    uint8_t state = (red ? 1 : 0) | (green ? 2 : 0);
    if (state != _ledState)
    {
        _ledState = state;
        setLEDs(red, green);
    }
}

void RobotController::capTargetsForLimp(float& leftMPS, float& rightMPS) const
{
    if (!anyWheelLimp())
        return;

    float cap = _params.maxSpeedMPS * LIMP_SPEED_SCALE;
    float maxMag = fmaxf(fabsf(leftMPS), fabsf(rightMPS));
    if (maxMag > cap)
    {
        // Scale both sides together so turn geometry is preserved
        float scale = cap / maxMag;
        leftMPS  *= scale;
        rightMPS *= scale;
    }
}

void RobotController::applyPIDControl(float dt)
{
    // Targets are capped robot-wide while any wheel is limping
    float targetLeft  = _targetLeftSpeed;
    float targetRight = _targetRightSpeed;
    capTargetsForLimp(targetLeft, targetRight);

    // Feedforward initialisation — seed duty from target speed on first non-zero cycle
    if (_leftFrontDuty == 0)  _leftFrontDuty  = mpsToDuty(targetLeft);
    if (_leftBackDuty == 0)   _leftBackDuty   = mpsToDuty(targetLeft);
    if (_rightFrontDuty == 0) _rightFrontDuty = mpsToDuty(targetRight);
    if (_rightBackDuty == 0)  _rightBackDuty  = mpsToDuty(targetRight);

    // PID corrections — each wheel tracked independently. A limping wheel
    // (encoder untrusted) gets feedforward duty only: correcting against a
    // dead encoder is what causes the full-throttle runaway.
    if (_wheelFault[FRONT_LEFT] == WHEEL_LIMP)
        _leftFrontDuty = mpsToDuty(targetLeft);
    else
        _leftFrontDuty += _frontLeftPID.compute(targetLeft, getFrontLeftWheelSpeed(), dt);

    if (_wheelFault[REAR_LEFT] == WHEEL_LIMP)
        _leftBackDuty = mpsToDuty(targetLeft);
    else
        _leftBackDuty += _rearLeftPID.compute(targetLeft, getRearLeftWheelSpeed(), dt);

    if (_wheelFault[FRONT_RIGHT] == WHEEL_LIMP)
        _rightFrontDuty = mpsToDuty(targetRight);
    else
        _rightFrontDuty += _frontRightPID.compute(targetRight, getFrontRightWheelSpeed(), dt);

    if (_wheelFault[REAR_RIGHT] == WHEEL_LIMP)
        _rightBackDuty = mpsToDuty(targetRight);
    else
        _rightBackDuty += _rearRightPID.compute(targetRight, getRearRightWheelSpeed(), dt);

    // Zero duty immediately when target is zero
    if (targetLeft == 0)  { _leftFrontDuty  = 0; _leftBackDuty  = 0; }
    if (targetRight == 0) { _rightFrontDuty = 0; _rightBackDuty = 0; }

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
    float targetLeft  = _targetLeftSpeed;
    float targetRight = _targetRightSpeed;
    capTargetsForLimp(targetLeft, targetRight);

    // Simple open-loop: convert target speed directly to duty
    float leftDuty = mpsToDuty(targetLeft);
    float rightDuty = mpsToDuty(targetRight);

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
