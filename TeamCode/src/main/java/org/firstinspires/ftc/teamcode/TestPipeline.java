package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TestPipeline extends OpenCvPipeline {

    //////////////////////
    //CONSTRUCTOR
    //////////////////////
    public TestPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    //////////////////////
    //VARIABLES
    //////////////////////
    Telemetry telemetry;
    // Used for Configuration in FTC Dashboard
    public static boolean returnInput = true;
    // Represents the different positions of the object
    enum Position {
        LEFT,
        CENTER,
        RIGHT
    }
    Position pos = Position.CENTER;
    // A Matrix stores image data
    Mat color = new Mat();
    // Rectangles
    public static Rect box1 = new Rect(0,100,30,30);
    public static Rect box2 = new Rect(100,100,30,30);
    public static Rect box3 = new Rect(200,100,30,30);

    //////////////////////
    //METHODS
    //////////////////////

    // Shows what the camera is seeing
    @Override
    public Mat processFrame(Mat input){
        // Based on the color channel, it evaluates a greyscale channel where
        // the darkest color is the least of the color, and
        // the brightest color is the most of the color.

        /*
        // Turns from RGB into HSV
        Imgproc.cvtColor(input, color, Imgproc.COLOR_RGB2HSV);
        // Defines the range of red color in HSV
        Scalar lowerHSV = new Scalar(0, 100, 100);
        Scalar upperHSV = new Scalar(10, 255, 255);
        Core.inRange(input, lowerHSV, upperHSV, input);
        */
        // rgb - 012
        Core.extractChannel(input, color, 1);
        // Creates a submat based on the area of the box
        Mat area1 = color.submat(box1);
        Mat area2 = color.submat(box2);
        Mat area3 = color.submat(box3);
        // Finds the average color value of the channel in the rectangle
        double avg1 = Core.mean(area1).val[0];
        double avg2 = Core.mean(area2).val[0];
        double avg3 = Core.mean(area3).val[0];
        // Creates a visual representation to be seen on the camera
        Imgproc.rectangle(input, box1, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, box2, new Scalar(0,0,255), 2);
        Imgproc.rectangle(input, box3, new Scalar(0,0,255), 2);
        // Finds the darkest color value, and makes a green border
        double min = Math.min(avg3, Math.min(avg1, avg2));
        if (min == avg1){
            pos = Position.LEFT;
            Imgproc.rectangle(input, box1, new Scalar(0,255,0), 2);
        }
        else if (min == avg2){
            pos = Position.CENTER;
            Imgproc.rectangle(input, box2, new Scalar(0,255,0), 2);
        }
        else if (min == avg3){
            pos = Position.RIGHT;
            Imgproc.rectangle(input, box3, new Scalar(0,255,0), 2);
        }
        telemetry.addData("Position", pos);
        telemetry.update();
        // Shows what the camera sees with rectangles
        if (returnInput){
            return input;
        }
        // Shows the greyscale channel
        else{
            return color;
        }
    }
    // Returns the current position where the desired object is
    public Position getPos(){
        return pos;
    }
}
