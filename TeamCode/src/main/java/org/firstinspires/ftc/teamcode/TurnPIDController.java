package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController {
    //////////////////////
    //VARIABLES
    //////////////////////
    private double targetAngle;
    private double kP, kI, kD;
    private double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = -1;
    private double lastSlope = 0;
    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public TurnPIDController(double targetAngle, double p, double i, double d){
        this.targetAngle = targetAngle;
        kP = p;
        kI = i;
        kD = d;

    }

    //////////////////////
    //METHODS
    //////////////////////

    // Returns the optimal motorPower that will prevent overshooting, oscillation, and stagnation
    public double update(double currentAngle){
        // Proportional Coefficient
        double error = targetAngle - currentAngle;
        // error is within ±360
        error %= 360;
        // error is positive
        error += 360;
        // positive error is within ±360
        error %= 360;
        // error is within ±180
        if (error > 180){
            error -= 360;
        }

        // Integral Coefficient
        // Adds more power to the motor to overcome stagnation

        // If there is a collision, the desired direction may change
        // accumulatedError becomes the same sign as the error
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        // error is within 2 degree of target angle
        if (Math.abs(error) < 2){
            accumulatedError = 0;
        }

        // Derivative Coefficient
        // Measures the (change in error) / (change in time)
        double slope = 0;
        // If the updates function was called a 2nd time
        if(lastTime > 0){
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastSlope = slope;
        lastTime = timer.milliseconds();
        lastError = error;

        // Motor power calculation (CRAZY MATH STUFF)
        // The first part makes sure it goes in the direction of the error
        // motorPower is within ±1
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * slope);
        return motorPower;

    }

    // Returns the (change in error) / (change in time)
    public double getLastSlope() {
        return lastSlope;
    }
}
