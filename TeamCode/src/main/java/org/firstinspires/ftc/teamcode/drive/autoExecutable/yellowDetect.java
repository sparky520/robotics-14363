package org.firstinspires.ftc.teamcode.drive.autoExecutable;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class yellowDetect extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    private String location = "RIGHT";

    static final Rect LEFT = new Rect(
            new Point(160, 90),
            new Point(220, 220));
    static final Rect RIGHT = new Rect(
            new Point(0, 100),
            new Point(60, 230));
    static double MIDDLE_PERCENT_COLOR_THRESHOLD = 0.1;
    static double LEFT_PERCENT_COLOR_THRESHOLD = 0.2;
    public yellowDetect(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(20, 100, 100);
        Scalar highHSV = new Scalar(30, 255,255);


        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT);
        Mat left = mat.submat(LEFT);

        double rightValue = Core.sumElems(right).val[0] / RIGHT.area() / 255;
        double leftValue = Core.sumElems(left).val[0] / LEFT.area() / 255;

        right.release();
        left.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Left percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(leftValue * 100) + "%");

        boolean TSERight = rightValue > LEFT_PERCENT_COLOR_THRESHOLD;
        boolean TSELeft = leftValue > MIDDLE_PERCENT_COLOR_THRESHOLD;

        if(TSELeft){
            location = "LEFT";
            telemetry.addData("TSE Location", "LEFT");
            // TSE = team scoring element
        }
        else if (TSERight){
            location = "RIGHT";
            telemetry.addData("TSE Location", "RIGHT");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar noTSE = new Scalar(255, 0, 0);
        Scalar yesTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, RIGHT, location.equals("RIGHT")? yesTSE:noTSE);
        Imgproc.rectangle(mat, LEFT, location.equals("LEFT")? yesTSE:noTSE);

        return mat;
    }

    public String getLocation() {
        return location;
    }
}
