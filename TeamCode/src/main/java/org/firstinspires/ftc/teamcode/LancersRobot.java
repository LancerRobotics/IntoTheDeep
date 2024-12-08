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

    public DcMotor slidesMotor;
    public Servo hookServo;

    private LinearOpMode linearOpMode;


    public LancersRobot (final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        configureMotors(hardwareMap);
    }

    public void configureMotors(final @NotNull HardwareMap hardwareMap) {
        leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        leftRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void pause(int sleepTime){
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

        linearOpMode.sleep(sleepTime);
    }
    public void pause(){
        leftFront.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);

        linearOpMode.sleep(1300);
    }
    public void forward(){
        leftFront.setPower(0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(0.5);
        leftRear.setPower(0.5);
    }
    public void backward() {
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(-0.5);
        leftRear.setPower(-0.5);
    }
    public void turnLeft() {
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
    public void strafeLeft(){ // EXPERIMENTAL DUE TO WEIGHT ISSUES
        leftFront.setPower(-0.5);
        rightFront.setPower(0.5);
        rightRear.setPower(-0.5);
        leftRear.setPower(0.5);
    }
    public void strafeRight() { // EXPERIMENTAL DUE TO WEIGHT ISSUES
        leftFront.setPower(0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(0.5);
        leftRear.setPower(-0.5);
    }

    // functions below are for the slides and the claw

    public void slidesMovement(boolean isGoingUp){
        slidesMotor.setPower(isGoingUp ? 1 : -1); // Will need to adjust this over time
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
