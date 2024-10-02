package org.firstinspires.ftc.teamcode.Test;

// The hardware object is referenced

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Configuration.HardwareBot;
import org.firstinspires.ftc.teamcode.MainOpModes.CameraPipeline;
import org.firstinspires.ftc.teamcode.MainOpModes.TurnPIDController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "TutorialAuto")

public class TutorialAuto extends LinearOpMode{
    //////////////////////
    //VARIABLES
    //////////////////////

    // The configuration of all robot components
    public HardwareBot robot = new HardwareBot();
    // used to record time
    public ElapsedTime runtime;

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public TutorialAuto(){}

    //////////////////////
    //MAIN METHOD
    //////////////////////
    /**
     * This method must be implemented. It is overridden to do the stuff we want.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // INITIALIZES ROBOT POSITIONS
        robot.init(hardwareMap);
        robot.grab_left.setPosition(0.35);
        robot.grab_right.setPosition(0.68);
        robot.drone.setPosition(0);
        // INITIALIZED
        this.telemetry.addData("Status", "Initialized");
        this.telemetry.update();
        // WAITS FOR AUTO TO BE STARTED
        waitForStart();
        sleep(500);
        // READY
        telemetry.addData("Status", "Ready");
        telemetry.update();
        sleep(2000);

        ///////////////////////////////////////////////////////
        // EXECUTE YOUR COMMANDS HERE
        ///////////////////////////////////////////////////////

        sleep(2000);
        // FINISHED
        telemetry.addData("Status", "Finished Auto");
        telemetry.update();

    }

    //////////////////////
    //METHODS
    //////////////////////

    // IMPLEMENT THESE METHODS!

    public void moveForward(double power, double time){

    }

    public void turn(double power, double time){

    }

    public void climb(double climbTime, double climbPower){

    }

    public void grab(double leftPos, double rightPos){

    }




}

