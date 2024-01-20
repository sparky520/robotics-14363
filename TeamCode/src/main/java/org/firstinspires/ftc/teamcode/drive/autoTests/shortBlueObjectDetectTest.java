package org.firstinspires.ftc.teamcode.drive.autoTests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class shortBlueObjectDetectTest extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    private String location = "LEFT";

    static final Rect MIDDLE_ROI = new Rect(
            new Point(260, 30),
            new Point(320, 70));

    /*static final Rect MIDDLE_ROI = new Rect(
            new Point(0, 110),
            new Point(60, 150));*/
    static final Rect RIGHT_ROI = new Rect(
            new Point(40, 0),
            new Point(100, 40));
    static double MIDDLE_PERCENT_COLOR_THRESHOLD = 0.1;
    static double LEFT_PERCENT_COLOR_THRESHOLD = 0.2;
    public shortBlueObjectDetectTest(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(100, 150, 0);
        Scalar highHSV = new Scalar(140, 255,255);


        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        right.release();
        middle.release();

        telemetry.addData("Left raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Left percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

        boolean TSERight = rightValue > LEFT_PERCENT_COLOR_THRESHOLD;
        boolean TSEMiddle = middleValue > MIDDLE_PERCENT_COLOR_THRESHOLD;

        if(TSEMiddle){
            location = "MIDDLE";
            telemetry.addData("TSE Location", "MIDDLE");
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

        Imgproc.rectangle(mat, RIGHT_ROI, location.equals("RIGHT")? yesTSE:noTSE);
        Imgproc.rectangle(mat, MIDDLE_ROI, location.equals("MIDDLE")? yesTSE:noTSE);

        return mat;
    }

    public String getLocation() {
        return location;
    }
}
