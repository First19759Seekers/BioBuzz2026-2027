package org.firstinspires.ftc.teamcode.util;

/**
 * Small PID controller with anti-windup and an output clamp.
 * Used by the pose follower for heading, and available for any tuning OpMode.
 */
public class PIDController {

    private double kp;
    private double ki;
    private double kd;

    private double integral;
    private double lastError;
    private double lastMeasurement;
    private boolean firstUpdate = true;

    private double integralLimit = 1.0;
    private double outputLimit = Double.MAX_VALUE;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void setIntegralLimit(double limit) {
        this.integralLimit = limit;
    }

    public void setOutputLimit(double limit) {
        this.outputLimit = limit;
    }

    public double calculate(double error, double dt) {
        if (dt <= 0.0) {
            dt = 0.001;
        }

        integral += error * dt;
        if (integral > integralLimit) {
            integral = integralLimit;
        } else if (integral < -integralLimit) {
            integral = -integralLimit;
        }

        double derivative = firstUpdate ? 0.0 : (error - lastError) / dt;
        lastError = error;
        lastMeasurement = error;
        firstUpdate = false;

        double output = kp * error + ki * integral + kd * derivative;
        if (output > outputLimit) {
            output = outputLimit;
        } else if (output < -outputLimit) {
            output = -outputLimit;
        }
        return output;
    }

    public double getLastError() {
        return lastMeasurement;
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastMeasurement = 0.0;
        firstUpdate = true;
    }
}
