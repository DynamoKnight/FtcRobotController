package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MainOpModes.CameraPipeline;
import org.firstinspires.ftc.teamcode.MainOpModes.NakulAuto;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Detection", group = "")
//@Autonomous
public class TestCameraAuto extends LinearOpMode {

    OpenCvWebcam camera;
    CameraPipeline pipeline;

    @Override
    public void runOpMode(){
        // Gets the camera object
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new CameraPipeline(telemetry, NakulAuto.Side.BLUE_BACK);

        camera.setPipeline(pipeline);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            // Sets the Camera Resolution and Orientation
            public void onOpened() {
                camera.startStreaming(320, 240 , OpenCvCameraRotation.UPRIGHT);
            }
            // Reports error
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });
        // Displays on FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(camera, 0);
        waitForStart();
        CameraPipeline.Position position = pipeline.getPos();
    }

}
