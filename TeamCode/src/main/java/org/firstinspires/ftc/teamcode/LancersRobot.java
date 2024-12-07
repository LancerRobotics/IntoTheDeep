package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import com.qualcomm.hardware.lynx.LynxModule;


public class LancersRobot {
    private final HardwareMap hardwareMap;
    public LancersRobot (final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public static void configureMotors(final @NotNull HardwareMap hardwareMap) {
        final @NotNull DcMotor leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        final @NotNull DcMotor leftRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        final @NotNull DcMotor rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        final @NotNull DcMotor rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);
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


}
