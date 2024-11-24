package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lancers.util.UnitUtil;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.LancersBotConfig;

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
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1.37795276d / 2d; // in // was originally diameter, hence /2
    public static double GEAR_RATIO = StandardTrackingWheelLocalizer.GEAR_RATIO; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 93.11; // X is the up and down direction
    public static double PARALLEL_Y = 110.16; // Y is the strafe direction

    public static double PERPENDICULAR_X = 72.216;
    public static double PERPENDICULAR_Y = 33.496;

    // Multipliers to adjust for dimensional inaccuracy in both direction
    // Added https://learnroadrunner.com/dead-wheels.html#adjusting-the-wheel-radius

    // NOTE: only one trial was done for each of these, so it is theoretically possible that the multipliers are off
    public static double CALIBRATION_X_TRAVELLED_INCHES = 104.5d;
    public static double CALIBRATION_X_MEASURED_INCHES = 104.027d; // dont take more than 3 decimal places, unreliable

    public static double CALIBRATION_Y_TRAVELLED_INCHES = -103.25d; // did this backwards
    public static double CALIBRATION_Y_MEASURED_INCHES = -102.891d;

    // TODO: ADJUST CALIBRATIONS (alan)

    public static double X_MULTIPLIER = CALIBRATION_X_TRAVELLED_INCHES / CALIBRATION_X_MEASURED_INCHES;
    public static double Y_MULTIPLIER = CALIBRATION_Y_TRAVELLED_INCHES / CALIBRATION_Y_MEASURED_INCHES;

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

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_RIGHT_MOTOR));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_LEFT_MOTOR));

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
                encoderTicksToInches(parallelEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition()) * Y_MULTIPLIER);
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
        //fix used, we are using REV bore encoders
    }
}
