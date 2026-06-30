
#include "./DCMotor.h"


DCMotor::DCMotor() : FOCMotor() {
    torque_controller = TorqueControlType::voltage;
    phase_resistance = NOT_SET;
    KV_rating = NOT_SET;
    pole_pairs = 1; // DC motors considered to have 1PP for purposes of shaft-angle calculations
};


void DCMotor::linkDriver(DCDriver* driver) {
    this->driver = driver;
};


int DCMotor::init() {
    if (!driver || !driver->initialized) {
        motor_status = FOCMotorStatus::motor_init_failed;
        SIMPLEFOC_DEBUG("MOT: Init not possible, driver not initialized");
        return 0;
    }
    if (!sensor) {
        motor_status = FOCMotorStatus::motor_init_failed;
        SIMPLEFOC_DEBUG("MOT: Init not possible, sensor not initialized");
        return 0;
    }

    motor_status = FOCMotorStatus::motor_initializing;
    SIMPLEFOC_DEBUG("MOT: Init");

    // sanity check for the voltage limit configuration
    if(voltage_limit > driver->voltage_limit) voltage_limit =  driver->voltage_limit;
    // constrain voltage for sensor alignment
    if(voltage_sensor_align > voltage_limit) voltage_sensor_align = voltage_limit;

    // update the controller limits
    if(current_sense){
        // current control loop controls voltage
        PID_current_q.limit = voltage_limit;
        PID_current_d.limit = voltage_limit;
    }
    if(_isset(phase_resistance) || torque_controller != TorqueControlType::voltage){
        // velocity control loop controls current
        PID_velocity.limit = current_limit;
    }else{
        // velocity control loop controls the voltage
        PID_velocity.limit = voltage_limit;
    }
    P_angle.limit = velocity_limit;

    _delay(50);
    // enable motor
    SIMPLEFOC_DEBUG("MOT: Enable driver.");
    enable();
    _delay(50);

    if (sensor_direction==Direction::UNKNOWN) {
        sensor_direction = Direction::CW;
        SIMPLEFOC_DEBUG("MOT: Sensor Dir: CW");
    }

    motor_status = FOCMotorStatus::motor_ready;
    return 1;
};


void DCMotor::disable() {
    // set zero to PWM
    driver->setPwm(0);
    // disable the driver
    driver->disable();
    // motor status update
    enabled = 0;
};


void DCMotor::enable() {
    // enable the driver
    driver->enable();
    // set zero to PWM
    driver->setPwm(0);
    // motor status update
    enabled = 1;
};


void DCMotor::move(float new_target) {
    // get angular velocity
    if (sensor) sensor->update();
    shaft_angle = shaftAngle();
    shaft_velocity = shaftVelocity();

    // update motor target
    if(_isset(new_target)) target = new_target;

    // open-loop not supported
    if( controller==MotionControlType::angle_openloop || controller==MotionControlType::velocity_openloop ) return;
    
    // if disabled do nothing
    if(!enabled) return;

    // get angle
    voltage.d = 0.0f; // not used for DC motors
    switch (controller) {
        case MotionControlType::torque:
            if (torque_controller == TorqueControlType::voltage)
                voltage.q = target;
            else
                voltage.q = 0.0f; // TODO implement the other torque modes, and current control loop
            break;
        case MotionControlType::angle_nocascade:
            // non-cascaded angle control: P_angle output goes directly to voltage
            // (no inner velocity loop — mirrors FOCMotor implementation)
            shaft_angle_sp = target;
            voltage.q = P_angle(shaft_angle_sp - LPF_angle(shaft_angle));
            voltage.q = _constrain(voltage.q, -voltage_limit, voltage_limit);
            break;
        case MotionControlType::angle:
            // angle set point
            // include angle loop
            shaft_angle_sp = target;
            shaft_velocity_sp = P_angle( shaft_angle_sp - shaft_angle );
            voltage.q = PID_velocity(shaft_velocity_sp - shaft_velocity);
            break;
        case MotionControlType::velocity:
            // velocity set point
            shaft_velocity_sp = target;
            voltage.q = PID_velocity(shaft_velocity_sp - shaft_velocity);
            break;
        default:
            voltage.q = 0.0f;
            disable();
            motor_status = FOCMotorStatus::motor_error; // we don't support this mode
            return;
    }
    // set the voltage to the motor
    setPhaseVoltage(voltage.q, 0.0f, 0.0f);
};


void DCMotor::setPhaseVoltage(float Uq, float Ud, float angle_el) {
    if (enabled==1)
        driver->setPwm(Uq);
    _UNUSED(Ud);
    _UNUSED(angle_el);
};


int DCMotor::initFOC() {
    // nothing to do for DC motors
    return 0; // always return failure
}; 


void DCMotor::loopFOC() {
    // nothing to do for DC motors
};