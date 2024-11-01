package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp()
public class LancersTeleOp extends LinearOpMode {
    public static final String TAG = "LancerTeleOp";

    public double trackedExtensionRadians = 0.0d;
    public double trackedRotationRadians = 0.0d;

    // do not make final in order to edit from dashboard
    public static double MINIMUM_EXTENSION_RADIANS = 0.0d;
    public static double MAXIMUM_EXTENSION_RADIANS = Double.MAX_VALUE;
    public static double MINIMUM_ROTATION_RADIANS = 0.0d;
    public static double MAXIMUM_ROTATION_RADIANS = Double.MAX_VALUE;

    public long currentRunTimeStamp = -1;
    public long timeStampAtLastOpModeRun = -1;


    @Override
    public void runOpMode() throws InterruptedException  {
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            currentRunTimeStamp = System.currentTimeMillis();

            // make sure you run these each time
            final DcMotor leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
            final DcMotor leftRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
            final DcMotor rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
            final DcMotor rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);

            final DcMotorEx clockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.CLOCKWISE_EXPAND_MOTOR);
            final DcMotorEx counterclockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTERCLOCKWISE_EXPAND_MOTOR);

            final DcMotorEx rotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.ROTATION_MOTOR);

            final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);


            final double speedMultiplier = (gamepad1.a ? 1.0d : 0.8d)/1.5;

            // Gamepad positions; Motors are swapped
            final double ly = -respectDeadZones(gamepad1.left_stick_y) * speedMultiplier; // Remember, Y stick value is reversed
            final double lx = -respectDeadZones(gamepad1.left_stick_x) * speedMultiplier; // Counteract imperfect strafing
            final double rx = respectDeadZones(gamepad1.right_stick_x) * speedMultiplier;

            double currentServoPosition = hookServo.getPosition();
            double openServoPosition = 0.15;
            double closeServoPosition = 0.515;
            final double servoSpeed = 0.01;

            hookServo.scaleRange(openServoPosition, closeServoPosition);
            if ((gamepad2.a) && currentServoPosition < closeServoPosition) {
                currentServoPosition += servoSpeed;
            } else if ((gamepad2.b) && currentServoPosition > openServoPosition) {
                currentServoPosition -= servoSpeed;
            }

            hookServo.setPosition(currentServoPosition);
            //currentServoPosition = Range.clip(currentServoPosition, openServoPosition, closeServoPosition);
            telemetry.addData("Servo Position: ", currentServoPosition);


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            final double frontLeftPower = (ly + lx + rx) / denominator;
            final double backLeftPower = (ly - lx + rx) / denominator;
            final double frontRightPower = (ly - lx - rx) / denominator;
            final double backRightPower = (ly + lx - rx) / denominator;

            leftFront.setPower(-frontLeftPower);
            leftRear.setPower(-backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // after movement: handle expansion/retraction of boat hook arm
            final double shrinkTrigger = respectDeadZones(gamepad2.left_trigger);
            final double expandTrigger = respectDeadZones(gamepad2.right_trigger);

            double carbonFiberPower = clampToMotorDomain(expandTrigger - shrinkTrigger);

            // before setting the power, find area under the curve and analyze if a subsequent run would set our value to 0
            // extension encoder is attached to the CW motor
            // CW: expansion
            // CCW: extraction
            final double clockwiseEncoderReading = clockwiseMotor.getVelocity(AngleUnit.RADIANS);
            if (timeStampAtLastOpModeRun != -1d) {
                // component of discrete integral
                trackedExtensionRadians += (clockwiseEncoderReading) * (timeStampAtLastOpModeRun - currentRunTimeStamp);
            }

            telemetry.addData("trackedExtensionRadians", trackedExtensionRadians);

            // as carbon fiber extends, clockwise +power and counterclockwise -power
            // as carbon fiber extends, clockwise -power and counterclockwise +power
            clockwiseMotor.setPower(-carbonFiberPower);
            counterclockwiseMotor.setPower(carbonFiberPower);

            // arm rotation motor
            double rotateTrigger = respectDeadZones(gamepad2.right_stick_y);
            final double  rotateMultiplier = 0.5;

            // do same integral work
            final double rotationEncoderReading = rotationMotor.getVelocity(AngleUnit.RADIANS);
            if (timeStampAtLastOpModeRun != -1d) {
                // component of discrete integral
                trackedRotationRadians += (rotationEncoderReading) * (timeStampAtLastOpModeRun - currentRunTimeStamp);
            }

            telemetry.addData("trackedRotationRadians", trackedRotationRadians);

            rotationMotor.setPower(rotateTrigger);

            // TODO: implement limits and velocity limits as we reach limit (adjust carbonFiberPower)
            // TODO: implement trig to make sure that the length never exceeds the limit

            // we finished an iteration, record the time the last value was recorded for use in finding sum
            timeStampAtLastOpModeRun = currentRunTimeStamp;
            telemetry.update();
        }
    }

    public static final double DEAD_ZONE_LIMIT = 0.15d;

    /**
     * Stick return is unreliable near inside, toss signals that are less than a threshold to maintain stationary behaviour when sticks may or may not be being minimally actuated by using this method to wrap a double value.
     * @param input
     * @return
     */
    public static double respectDeadZones(double input) {
        if (Math.abs(input) < DEAD_ZONE_LIMIT) {
            return 0.0d;
        } else {
            return input;
        }
    }

    /**
     * Clamps an input double to a range between [-1, 1] to allow it to be passed to a motor.
     * @param input
     * @return
     */
    public static double clampToMotorDomain(double input) {
        return Math.min(1.0d, Math.max(input, -1.0d));
    }
}