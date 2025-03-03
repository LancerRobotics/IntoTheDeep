package org.firstinspires.ftc.teamcode.RRAutons;

import static org.firstinspires.ftc.teamcode.LancersTeleOp.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.LancersBotConfig;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.LancersRobot;

import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.HardwareMap;



class Claw implements Action {
    boolean open;
    private HardwareMap hardwareMap;
    private Servo hookServo;
    public Claw(boolean open, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.open = open;
        this.hookServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, LancersBotConfig.HOOK_SERVO); ;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (open) {
            hookServo.setPosition(0);
            packet.put("Auton_Servo", "Extended");
        }
        else {
            hookServo.setPosition(1);
            packet.put("Auton_Servo", "Retrieved");
        }
        return false;

    }
}

class Movement implements Action {
    private HardwareMap hardwareMap;
    private double time;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    public Movement(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        this.leftFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        this.leftRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        this.rightFront = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        this.rightRear = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        leftFront.setPower(0.2);
        leftRear.setPower(0.2);
        rightFront.setPower(-0.2);
        rightRear.setPower(-0.2);

        return false;

    }

    public void stopMotors(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }
}

class Arm implements Action {
    private HardwareMap hardwareMap;
    private Servo hookServo;
    private boolean extendOrReel;
    private boolean pow;
    private double time;
    private double extendPower;
    private double rotatePower;

    private DcMotorEx clockwiseMotor;
    private DcMotorEx counterclockwiseMotor;
    private DcMotorEx rotationMotor;
    private DcMotorEx counterRotationMotor;
    public Arm(boolean extendOrReel, boolean pow, double time, double extendPower, double rotatePower, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.extendOrReel = extendOrReel;
        this.pow = pow;
        this.extendPower = extendPower;
        this.rotatePower = rotatePower;

        this.hookServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, LancersBotConfig.HOOK_SERVO);

        this.clockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.CLOCKWISE_EXPAND_MOTOR);
        this.counterclockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTERCLOCKWISE_EXPAND_MOTOR);
        this.rotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.ROTATION_MOTOR);
        this.counterRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTER_ROTATION_MOTOR);
    }

    public Arm(boolean extendOrReel, boolean pow, double time, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.extendOrReel = extendOrReel;
        this.pow = pow;
        this.extendPower = 1;
        this.rotatePower = 0.5;

        this.hookServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, LancersBotConfig.HOOK_SERVO);

        this.clockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.CLOCKWISE_EXPAND_MOTOR);
        this.counterclockwiseMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTERCLOCKWISE_EXPAND_MOTOR);
        this.rotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.ROTATION_MOTOR);
        this.counterRotationMotor = (DcMotorEx) hardwareMap.dcMotor.get(LancersBotConfig.COUNTER_ROTATION_MOTOR);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(extendOrReel) { //extends or reels arm
            if (pow) { // extends arm | 0.43 seconds
                clockwiseMotor.setPower(-extendPower);
                counterclockwiseMotor.setPower(extendPower);
                packet.put("Auton_Arm", "Extended");
            }
            else { // reels arm
                clockwiseMotor.setPower(extendPower);
                counterclockwiseMotor.setPower(-extendPower);
                packet.put("Auton_Arm", "Reeled");
            }
        }
        else { //rotates arm
            if (pow) { // rotates towards front | 1.5 seconds if fully extended to touch pole
                rotationMotor.setPower(-rotatePower);
                counterRotationMotor.setPower(rotatePower);
            }
            else { // rotates towards back
                rotationMotor.setPower(rotatePower);
                counterRotationMotor.setPower(-rotatePower);
            }
        }

        return false;

    }

    public void stopMotors(){
        clockwiseMotor.setPower(0);
        counterclockwiseMotor.setPower(0);
        rotationMotor.setPower(0);
        counterRotationMotor.setPower(0);
    }
}