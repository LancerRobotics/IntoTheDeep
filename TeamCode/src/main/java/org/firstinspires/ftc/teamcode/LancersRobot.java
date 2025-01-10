package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.jetbrains.annotations.NotNull;

import com.qualcomm.hardware.lynx.LynxModule;


public class LancersRobot {
    private final HardwareMap hardwareMap;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    public Servo hookServo;

    public DcMotorEx clockwiseMotor;
    public DcMotorEx counterclockwiseMotor;
    public DcMotorEx rotationMotor;
    public DcMotorEx counterRotationMotor;

    private LinearOpMode linearOpMode;

    public LancersRobot (final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);

        hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);

        clockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.CLOCKWISE_EXPAND_MOTOR);
        clockwiseMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        counterclockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTERCLOCKWISE_EXPAND_MOTOR);
        counterclockwiseMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        rotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.ROTATION_MOTOR);
        rotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        counterRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTER_ROTATION_MOTOR);
        counterRotationMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        configureMotors(hardwareMap);
    }

    public void configureMotors(final @NotNull HardwareMap hardwareMap) {



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void configureMotors() {
        configureMotors(hardwareMap);
    }

    // hook servo

    public static double openServoPosition = 0.15;
    public static double closeServoPosition = 0.515;
    public static final double servoSpeed = 0.01;

    public void extendServo() {
        changeServoPosition(servoSpeed);
    }

    public void retrieveServo() {
        changeServoPosition(-servoSpeed);
    }

    public void changeServoPosition(double servoSpeed) {
        final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);
        double currentServoPosition = hookServo.getPosition();
        if((servoSpeed < 0 && currentServoPosition > openServoPosition)
                || (servoSpeed > 0 && currentServoPosition < closeServoPosition)) {
            currentServoPosition += servoSpeed;
        }
        setServoPosition(currentServoPosition);
    }

    public void setServoPosition(double currentServoPosition) {
        final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);
        hookServo.setPosition(currentServoPosition);
    }

    // All of the functions below are used for wheel movement in time-based autons

    public void pauseMotors(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
    }
    public void forward(){
        long start = System.nanoTime();
        while (System.nanoTime() - start < 15E8) { // 2 seconds
            leftFront.setPower(0.5);
            rightFront.setPower(0.5);
            rightRear.setPower(0.5);
            leftRear.setPower(0.5);
        }
    }
    public void forward(double power){
        //long start = System.nanoTime();
        //while (System.nanoTime() - start < 15E8) { // 2 seconds
        leftFront.setPower(power);
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(power);
        //}
    }
    public void backward() {
        long start = System.nanoTime();
        while (System.nanoTime() - start < 15E8) { // 2 seconds
            leftFront.setPower(-0.5);
            rightFront.setPower(-0.5);
            rightRear.setPower(-0.5);
            leftRear.setPower(-0.5);
        }
    }
    public void backward(double power){
        long start = System.nanoTime();
        while (System.nanoTime() - start < 15E8) { // 2 seconds
            leftFront.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);
            leftRear.setPower(power);
        }
    }
    public void turnLeft() {
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(0.5);
        leftRear.setPower(-0.5);
    }
    public void turnLeft(double power) {
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(0.5);
        leftRear.setPower(-0.5);
    }
    public void turnRight() {
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(-0.5);
        leftRear.setPower(0.5);
    }
    public void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(-power);
        leftRear.setPower(power);
    }
    public void strafeLeft(){ // EXPERIMENTAL DUE TO WEIGHT ISSUES
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(-0.5);
        leftRear.setPower(0.5);
    }
    public void strafeLeft(double power){ // EXPERIMENTAL DUE TO WEIGHT ISSUES
        leftFront.setPower(-power);
        rightFront.setPower(power);
        rightRear.setPower(-power);
        leftRear.setPower(power);
    }
    public void strafeRight() { // EXPERIMENTAL DUE TO WEIGHT ISSUES
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(0.5);
        leftRear.setPower(-0.5);
    }
    public void strafeRight(double power) { // EXPERIMENTAL DUE TO WEIGHT ISSUES
        leftFront.setPower(power);
        rightFront.setPower(-power);
        rightRear.setPower(power);
        leftRear.setPower(-power);
    }

    // functions below are for the arm and the claw


    public void slidesMovement(boolean isGoingUp, double power){
        long start = System.nanoTime();
        if (isGoingUp){
            //while (System.nanoTime() - start < 5E9){ // 5 seconds
            slidesMotor.setPower(-power); // Will need to adjust this over time
            //}
        }
        else {
            //while (System.nanoTime() - start < 45E8){ // 5 seconds
            slidesMotor.setPower(power); // Will need to adjust this over time
            //}
        }
    }
    public void clawOpen(){
        hookServo.setPosition(LancersTeleOp.OPEN_SERVO_POSITION);
    }
    public void clawClose(){
        hookServo.setPosition(LancersTeleOp.CLOSE_SERVO_POSITION);
    }
    // TODO: FIX SERVO POSITION VARIABLE DECLARATIONS TO
    //  BE INSIDE THE LANCERS ROBOT CLASS

}
