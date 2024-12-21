package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp()
@Config
public class LancersTeleOp extends LinearOpMode {
    public static final String TAG = "LancerTeleOp";
    private double trackedExtensionRadians = 0.0d; // units : radians/sec * sec
    private double trackedRotationRadians = 0.0d;

    // do not make final in order to edit from dashboard
    public static double LAWFUL_MINIMUM_EXTENSION_RADIANS = -1.8d; // negative: expansion // 42 inch limit at base
    public static double MECHANICAL_ABSOLUTE_MINIMUM_EXTENSION_RADIANS = -2.3d;

    public static double DEGREES_FROM_BASELINE_AT_MINIMUM = -180.0d;
    public static double DEGREES_FROM_BASELINE_AT_MAXIMUM = 180.0d;

    public static double MINIMUM_ROTATION_RADIANS = -Double.MAX_VALUE;
    public static double MAXIMUM_ROTATION_RADIANS = Double.MAX_VALUE;

    public double getArmDeviationFromBaselineDegrees() {
        // convert range of trackedRotationRadians to DEGREES
        return Range.scale(trackedRotationRadians, MINIMUM_ROTATION_RADIANS, MAXIMUM_ROTATION_RADIANS, DEGREES_FROM_BASELINE_AT_MINIMUM, DEGREES_FROM_BASELINE_AT_MAXIMUM);
    }

    public static double ROTATE_MAX_SPEED_MULTIPLIER = 0.5;
    public static double CLAW_SERVO_SPEED = 0.4;
    public static double ROTATE_SERVO_SPEED = 1;

    public static double OPEN_SERVO_POSITION = 0.05;
    public static double CLOSE_SERVO_POSITION = 0.50; // Value at comp was 0.55, adjust later

    public static double ROTATE_UP_SERVO_POSITION = 0.73;
    public static double ROTATE_DOWN_SERVO_POSITION = 1;

    public final double slidesMultiplier = -1;
    public static double upliftSlidesPower = -0.1;
    public static boolean upliftMode = false;

    private long currentRunTimeStamp = -1;
    private long timeStampAtLastOpModeRun = -1;

    private Encoder parallelEncoder, perpendicularEncoder;

    @Override
    public void runOpMode() throws InterruptedException  {
        // init work (reset)
        trackedExtensionRadians = 0.0d;
        trackedRotationRadians = 0.0d;

        final DcMotor leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor leftRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        final DcMotor rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        final DcMotor slidesMotor = hardwareMap.dcMotor.get(LancersBotConfig.SLIDES_MOTOR);

        // EDIT LATER #sliderotateservo
        //final Servo slidesServo1 = hardwareMap.servo.get(LancersBotConfig.SLIDES_SERVO1);
        //final Servo slidesServo2 = hardwareMap.servo.get(LancersBotConfig.SLIDES_SERVO2);

        final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);
        hookServo.scaleRange(OPEN_SERVO_POSITION, CLOSE_SERVO_POSITION); // also scales getPosition

        final Servo slideServo1 = hardwareMap.servo.get(LancersBotConfig.SLIDES_SERVO1);
        slideServo1.scaleRange(ROTATE_UP_SERVO_POSITION, ROTATE_DOWN_SERVO_POSITION); // also scales getPosition
        final Servo slideServo2 = hardwareMap.servo.get(LancersBotConfig.SLIDES_SERVO2);
        slideServo2.scaleRange(ROTATE_UP_SERVO_POSITION, ROTATE_DOWN_SERVO_POSITION); // also scales getPosition

        slideServo1.setDirection(Servo.Direction.REVERSE);
        slideServo2.setDirection(Servo.Direction.REVERSE);

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_RIGHT_MOTOR));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_LEFT_MOTOR));

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            currentRunTimeStamp = System.currentTimeMillis();

            // slow claw movement
            final double currentServoPosition = hookServo.getPosition();
            if (gamepad2.left_bumper) {
                // snap to open
                hookServo.setPosition(OPEN_SERVO_POSITION);
            } else if (gamepad2.right_bumper) {
                // snap to close
                hookServo.setPosition(CLOSE_SERVO_POSITION);
            }

            final double current1RotateServoPosition = slideServo1.getPosition();
            final double current2RotateServoPosition = slideServo2.getPosition();

            if (gamepad2.a){
                upliftMode = !upliftMode;
            }
            if (upliftMode && (gamepad1.left_trigger == 0 && gamepad2.right_trigger == 0)){
                slidesMotor.setPower(upliftSlidesPower);
            }

            if (gamepad2.left_stick_y > 0) {
                // positive movement

                slideServo1.setPosition(Math.max(current1RotateServoPosition + ROTATE_SERVO_SPEED * (gamepad2.left_stick_y/10), 1.0d));
                slideServo2.setPosition(Math.max(current2RotateServoPosition + ROTATE_SERVO_SPEED * (gamepad2.left_stick_y/10), 1.0d));
            } else if (gamepad2.left_stick_y < 0) {
                // negative movement
                slideServo1.setPosition(Math.max(current1RotateServoPosition - ROTATE_SERVO_SPEED * (gamepad2.left_stick_y /10), 1.0d));
                slideServo2.setPosition(Math.max(current2RotateServoPosition - ROTATE_SERVO_SPEED * (gamepad2.left_stick_y /10), 1.0d));
            }

            if (gamepad2.x) {
                // snap to open
                slideServo1.setPosition(ROTATE_UP_SERVO_POSITION);
                slideServo2.setPosition(ROTATE_UP_SERVO_POSITION);
            } else if (gamepad2.y) {
                // snap to close
                slideServo1.setPosition(ROTATE_DOWN_SERVO_POSITION);
                slideServo2.setPosition(ROTATE_DOWN_SERVO_POSITION);
            }

            telemetry.addData("servo 1", current1RotateServoPosition);
            telemetry.addData("servo 2", current2RotateServoPosition);

            telemetry.addData("currentServoPosition", currentServoPosition);

            // movement
            final double speedMultiplier = gamepad1.a ? 1.0d : 0.8d;

            // Gamepad positions; Motors are swapped
            final double ly = -respectDeadZones(gamepad1.left_stick_y) * speedMultiplier; // Remember, Y stick value is reversed
            final double lx = -respectDeadZones(gamepad1.left_stick_x) * speedMultiplier; // Counteract imperfect strafing
            final double rx = respectDeadZones(gamepad1.right_stick_x) * speedMultiplier;

            final double slidesPositive = gamepad2.left_trigger;
            final double slidesNegative = gamepad2.right_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            final double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

            final double frontLeftPower = (ly + lx + rx) / denominator;
            final double backLeftPower = (ly - lx + rx) / denominator;
            final double frontRightPower = (ly - lx - rx) / denominator;
            final double backRightPower = (ly + lx - rx) / denominator;

            double slidesPower = 0.0d;
            final float TRIGGER_THRESHOLD = 0.15f;

            if (slidesPositive > TRIGGER_THRESHOLD){
                slidesPower = (slidesPositive - TRIGGER_THRESHOLD) * (1f / (1f - TRIGGER_THRESHOLD));
            }
            if (slidesNegative > TRIGGER_THRESHOLD){
                slidesPower =  -(slidesNegative - TRIGGER_THRESHOLD) * (1f / (1f - TRIGGER_THRESHOLD));
            }
            slidesPower *= slidesMultiplier;


            leftFront.setPower(-frontLeftPower*0.9);
            leftRear.setPower(-backLeftPower*0.9);
            rightFront.setPower(frontRightPower*0.9);
            rightRear.setPower(backRightPower*0.9);

            slidesMotor.setPower(slidesPower);

            telemetry.addData("X-value", parallelEncoder.getCurrentPosition());
            telemetry.addData("Y-value", perpendicularEncoder.getCurrentPosition());

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
}