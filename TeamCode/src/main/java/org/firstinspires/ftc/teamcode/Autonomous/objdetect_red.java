package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class objdetect_red extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location = Location.LEFT;

    static final Rect RIGHT_ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(140, 35),
            new Point(200, 75));
    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public objdetect_red(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(10, 255, 255);
        Scalar highHSV = new Scalar(179, 255,255);


        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        right.release();
        middle.release();

        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

        boolean TSERight = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean TSEMiddle = middleValue > PERCENT_COLOR_THRESHOLD;

        if(TSEMiddle){
            location = Location.MIDDLE;
            telemetry.addData("TSE Location", "MIDDLE");
            // TSE = team scoring element
        }
        else if (TSERight){
            location = Location.RIGHT;
            telemetry.addData("TSE Location", "RIGHT");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar noTSE = new Scalar(255, 0, 0);
        Scalar yesTSE = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? yesTSE:noTSE);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? yesTSE:noTSE);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
