package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 0;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    private List<Integer> lastEncPositions, lastEncVels;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        int leftPos = leftEncoder.getCurrentPosition();
        int rightPos = rightEncoder.getCurrentPosition();
        int frontPos = frontEncoder.getCurrentPosition();

        lastEncPositions.clear();
        lastEncPositions.add(leftPos);
        lastEncPositions.add(rightPos);
        lastEncPositions.add(frontPos);

        return Arrays.asList(
                encoderTicksToInches(leftPos),
                encoderTicksToInches(rightPos),
                encoderTicksToInches(frontPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int leftVel = (int) leftEncoder.getCorrectedVelocity();
        int rightVel = (int) rightEncoder.getCorrectedVelocity();
        int frontVel = (int) frontEncoder.getCorrectedVelocity();

        lastEncVels.clear();
        lastEncVels.add(leftVel);
        lastEncVels.add(rightVel);
        lastEncVels.add(frontVel);

        return Arrays.asList(
                encoderTicksToInches(leftVel),
                encoderTicksToInches(rightVel),
                encoderTicksToInches(frontVel)
        );
    }

    /*
     * Sample tracking wheel localizer implementation assuming the standard configuration:
     *
     *    ^
     *    |
     *    | ( x direction)
     *    |
     *    v
     *    <----( y direction )---->

     *        (forward)
     *    /--------------\
     *    |     ____     |
     *    |     ----     |    <- Perpendicular Wheel
     *    |           || |
     *    |           || |    <- Parallel Wheel
     *    |              |
     *    |              |
     *    \--------------/
     *
     */
    public static class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
        public static double TICKS_PER_REV = 0;
        public static double WHEEL_RADIUS = 2; // in
        public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

        public static double PARALLEL_X = 0; // X is the up and down direction
        public static double PARALLEL_Y = 0; // Y is the strafe direction

        public static double PERPENDICULAR_X = 0;
        public static double PERPENDICULAR_Y = 0;

        // Parallel/Perpendicular to the forward axis
        // Parallel wheel is parallel to the forward axis
        // Perpendicular is perpendicular to the forward axis
        private Encoder parallelEncoder, perpendicularEncoder;

        private SampleMecanumDrive drive;

        public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
            super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
            ));

            this.drive = drive;

            parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
            perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

            // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        }

        public static double encoderTicksToInches(double ticks) {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
        }

        @Override
        public double getHeading() {
            return drive.getRawExternalHeading();
        }

        @Override
        public Double getHeadingVelocity() {
            return drive.getExternalHeadingVelocity();
        }

        @NonNull
        @Override
        public List<Double> getWheelPositions() {
            return Arrays.asList(
                    encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                    encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
            );
        }

        @NonNull
        @Override
        public List<Double> getWheelVelocities() {
            // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
            //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
            //  compensation method

            return Arrays.asList(
                    encoderTicksToInches(parallelEncoder.getRawVelocity()),
                    encoderTicksToInches(perpendicularEncoder.getRawVelocity())
            );
        }
    }
}
