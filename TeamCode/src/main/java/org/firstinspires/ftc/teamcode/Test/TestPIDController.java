package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TestPIDController {
    //////////////////////
    //VARIABLES
    //////////////////////
    private double kP;
    private double kI;
    private double kD;
    private double accumulatedError = 0;
    private double lastError = 0;
    private double lastTime = -1;
    private ElapsedTime timer = new ElapsedTime();

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public TestPIDController(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    //////////////////////
    //METHODS
    //////////////////////

    public double PIDControl(double error){
        // Calculates distance from target
        accumulatedError += error * timer.seconds();
        double slope = 0;
        if(lastTime > 0){
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }

        lastError = error;
        lastTime = timer.seconds();

        timer.reset();

        return (error * kP) + (accumulatedError * kI) + (slope * kD);
    }


}
