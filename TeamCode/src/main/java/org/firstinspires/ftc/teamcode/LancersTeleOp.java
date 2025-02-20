package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp()
@Config
public class LancersTeleOp extends LinearOpMode {
    public static final String TAG = "LancerTeleOp";

    private double trackedExtensionRadians = 0.0d; // units : radians/sec * sec
    private double trackedRotationRadians = 0.0d;

    // do not make final in order to edit from dashboard
    public final static double LAWFUL_MINIMUM_HORIZONTAL_EXTENSION_RADIANS = 1.375;
    public static double LAWFUL_MINIMUM_EXTENSION_RADIANS = 0; // negative: expansion // 42 inch limit at base
    public static double MECHANICAL_ABSOLUTE_MINIMUM_EXTENSION_RADIANS = 10;
    // TODO: Find Mechanical Limit

    public static double DEGREES_FROM_BASELINE_AT_MINIMUM = -180.0d;
    public static double DEGREES_FROM_BASELINE_AT_MAXIMUM = 180.0d;

    public static double MINIMUM_ROTATION_RADIANS = -Double.MAX_VALUE;
    public static double MAXIMUM_ROTATION_RADIANS = Double.MAX_VALUE;

    public double getArmDeviationFromBaselineDegrees() {
        // convert range of trackedRotationRadians to DEGREES
        return Range.scale(trackedRotationRadians, MINIMUM_ROTATION_RADIANS, MAXIMUM_ROTATION_RADIANS, DEGREES_FROM_BASELINE_AT_MINIMUM, DEGREES_FROM_BASELINE_AT_MAXIMUM);
    }

    public static double ROTATE_MAX_SPEED_MULTIPLIER = 0.4;
    public static double CLAW_SERVO_SPEED = 0.4;

    public static double OPEN_SERVO_POSITION = 0;
    public static double CLOSE_SERVO_POSITION = 1; // Value at comp was 0.55, adjust later
    
    private long currentRunTimeStamp = -1;
    private long timeStampAtLastOpModeRun = -1;

    //private Encoder parallelEncoder, perpendicularEncoder;

    @Override
    public void runOpMode() throws InterruptedException  {
        // init work (reset)
        trackedExtensionRadians = 0.0d;
        trackedRotationRadians = 2.26893;

        // Get from hardwaremap, initialize variables as DcMotor type
        final DcMotor leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx leftRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        //DcMotorEx inherits from DcMotor class,
        //DcMotorEx used in pretty much the same way as DcMotor
        //I honestly don't really know the difference, but DcMotorEx seems to have more functionality

        final DcMotorEx clockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.CLOCKWISE_EXPAND_MOTOR);
        clockwiseMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx counterclockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTERCLOCKWISE_EXPAND_MOTOR);
        counterclockwiseMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        final DcMotorEx rotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.ROTATION_MOTOR);
        rotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotorEx counterRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTER_ROTATION_MOTOR);
        counterRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);
        hookServo.scaleRange(OPEN_SERVO_POSITION, CLOSE_SERVO_POSITION); // also scales getPosition

        //Declaring odometry pods/dead wheels/encoders/whatever you want to call it
        //Pulling from hardware map again
        //parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_RIGHT_MOTOR));
        //perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_LEFT_MOTOR));

        // go!!
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            currentRunTimeStamp = System.currentTimeMillis();

            // slow claw movement
            final double currentServoPosition = hookServo.getPosition();
            //Do not use the commented code below, they don't work
            //if (gamepad2.right_trigger > 0) {
                // positive movement
                //hookServo.setPosition(Math.max(currentServoPosition + CLAW_SERVO_SPEED*(gamepad2.left_trigger/10), 1.0d));
            //} else if (gamepad2.left_trigger > 0) {
                // negative movement
                //hookServo.setPosition(Math.min(currentServoPosition - CLAW_SERVO_SPEED*(gamepad2.right_trigger/10), 0.0d))}
            if (gamepad2.left_bumper) {
                // snap to open
                hookServo.setPosition(OPEN_SERVO_POSITION);
            } else if (gamepad2.right_bumper) {
                // snap to close
                hookServo.setPosition(CLOSE_SERVO_POSITION);
            }

            telemetry.addData("currentServoPosition", currentServoPosition);
            //display on driver hub

            // movement
            final double speedMultiplier = gamepad1.a ? 1.0d : 0.8d;

            // Gamepad positions; Motors are swapped
            final double ly = -respectDeadZones(gamepad1.left_stick_y) * speedMultiplier; // Remember, Y stick value is reversed
            final double lx = -respectDeadZones(gamepad1.left_stick_x) * speedMultiplier; // Counteract imperfect strafing
            final double rx = respectDeadZones(gamepad1.right_stick_x) * speedMultiplier;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            //Calculations, more in depth here:
            //https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html

            final double frontLeftPower = (ly + lx + rx) / denominator;
            final double backLeftPower = (ly - lx + rx) / denominator;
            final double frontRightPower = (ly - lx - rx) / denominator;
            final double backRightPower = (ly + lx - rx) / denominator;

            //Speed multipliers by .9, reduces speed of motor
            //Motors get very funky when running at maximum capacity, cap their speed
            leftFront.setPower(-frontLeftPower*0.9);
            leftRear.setPower(-backLeftPower*0.9);
            rightFront.setPower(frontRightPower*0.9);
            rightRear.setPower(backRightPower*0.9);

            // arm rotation motor
            double rotateTrigger = respectDeadZones(gamepad2.right_stick_y) * ROTATE_MAX_SPEED_MULTIPLIER;

            // do same integral work
            double rotationEncoderReading = rotationMotor.getVelocity(AngleUnit.RADIANS);

            // sets a new extension limit based on the current angle of rotation and horizontal limit
            setExtensionLimit();

            final double angleRatioMultiplier = 1.3;

            if ((timeStampAtLastOpModeRun != -1d) && (respectDeadZones(gamepad2.right_stick_y) != 0)) {
                // component of discrete integral
                trackedRotationRadians += (-rotationEncoderReading) * (angleRatioMultiplier) * (timeStampAtLastOpModeRun - currentRunTimeStamp)/1000;
            }

            if (trackedRotationRadians < MINIMUM_ROTATION_RADIANS) {
                // abort rotation
                rotateTrigger = Math.min(0.5d, rotateTrigger);
            } else if (trackedRotationRadians > MAXIMUM_ROTATION_RADIANS) {
                // abort rotation
                rotateTrigger = Math.max(-0.5d, rotateTrigger);
            }
            final double relativeRotation = getArmDeviationFromBaselineDegrees();

            //parallelEncoder.setDirection(Encoder.Direction.REVERSE);
            //perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);

            double degrees = (trackedRotationRadians * (180/Math.PI))%360;

            telemetry.addData("trackedRotationRadians", trackedRotationRadians);
            telemetry.addData("rotation (degrees)", degrees);
            telemetry.addData("relativeRotation", relativeRotation);

            rotationMotor.setPower(rotateTrigger);
            counterRotationMotor.setPower(-rotateTrigger);

            // arm extension motor(s)
            // after movement: handle expansion/retraction of boat hook arm
            final double expandShrinkJoystick = respectDeadZones(gamepad2.left_stick_y);

            // positive: expand, negative: contract
            double carbonFiberPower = Range.clip(expandShrinkJoystick, -1, 1);

            // before setting the power, find area under the curve and analyze if a subsequent run would set our value to 0
            // extension encoder is attached to the CW motor
            // CW: expansion
            // CCW: extraction
            final double clockwiseEncoderReading = clockwiseMotor.getVelocity(AngleUnit.RADIANS);
            final double counterclockwiseEncoderReading = leftRear.getVelocity(AngleUnit.RADIANS);

            boolean isTogether = true;
            if (Math.abs(clockwiseEncoderReading-counterclockwiseEncoderReading) > 10) {
                isTogether = false;
            }


            if ((timeStampAtLastOpModeRun != -1d) && ((respectDeadZones(gamepad2.left_stick_y) != 0) && (respectDeadZones(gamepad2.right_stick_y) == 0))) {
                trackedExtensionRadians += (counterclockwiseEncoderReading) * (timeStampAtLastOpModeRun - currentRunTimeStamp)/1000;
            }

            final boolean armTooLongToBeLegal = trackedExtensionRadians > LAWFUL_MINIMUM_EXTENSION_RADIANS;
            final boolean armTooLongMechanically = trackedExtensionRadians > MECHANICAL_ABSOLUTE_MINIMUM_EXTENSION_RADIANS;

            // over minimum: over max extension, pull in by setting a minimum value
            if (armTooLongToBeLegal || armTooLongMechanically) {
                // abort rotation; pull in until we are within bounds
                carbonFiberPower = -0.8;
            }

            telemetry.addData("armTooLongToBeLegal", armTooLongToBeLegal);
            telemetry.addData("armTooLongMechanically", armTooLongMechanically);
            telemetry.addData("trackedExtensionRadians", trackedExtensionRadians);
            telemetry.addData("Current Extension Limit", LAWFUL_MINIMUM_EXTENSION_RADIANS);

            telemetry.addData("Clockwise Reading", clockwiseEncoderReading);
            telemetry.addData("Extension Motors are Together", isTogether);

            // as carbon fiber extends, clockwise +power and counterclockwise -power
            // as carbon fiber extends, clockwise -power and counterclockwise +power
            clockwiseMotor.setPower(carbonFiberPower);
            counterclockwiseMotor.setPower(-carbonFiberPower);

            // we finished an iteration, record the time the last value was recorded for use in finding sum
            timeStampAtLastOpModeRun = currentRunTimeStamp;

            //telemetry.addData("X-value", parallelEncoder.getCurrentPosition());
            //telemetry.addData("Y-value", perpendicularEncoder.getCurrentPosition());

            telemetry.update();
        }
    }

    public void setExtensionLimit(){
        double radians = trackedRotationRadians;
        double cosValue = Math.cos(radians);
        double armLimit = (LAWFUL_MINIMUM_HORIZONTAL_EXTENSION_RADIANS)/(cosValue);

        //LAWFUL_MINIMUM_EXTENSION_RADIANS = Math.abs(armLimit);
        LAWFUL_MINIMUM_EXTENSION_RADIANS = 100;
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
}