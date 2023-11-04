package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class SimpleAuto extends LinearOpMode {
    NewHardwareBot robot;
    public void runOpMode() throws InterruptedException {
        robot = new NewHardwareBot();
        robot.init(hardwareMap);
        robot.claw.setPosition(0.5);

        waitForStart();

        robot.claw.setPosition(1);
        sleep(500);
        robot.setMotorPower(0.5);
        sleep(1500);
        robot.setMotorPower(0);
        robot.claw.setPosition(0.5);
        robot.setMotorPower(-0.2);
        sleep(500);
        robot.setMotorPower(0);
        sleep(100);
    }

}
