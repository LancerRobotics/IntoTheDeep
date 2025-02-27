package org.firstinspires.ftc.teamcode.testingOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class MotorTest extends OpMode {
    public static double motorPower = 0.1;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    int selectedMotorIndex = 0;
    DcMotor[] motors;
    String[] motorNames = {"rightBack", "leftBack", "rightFront", "leftFront"};

    @Override
    public void init() {
        backRightMotor = hardwareMap.get(DcMotor.class, motorNames[0]);
        backLeftMotor = hardwareMap.get(DcMotor.class, motorNames[1]);
        frontRightMotor = hardwareMap.get(DcMotor.class, motorNames[2]);
        frontLeftMotor = hardwareMap.get(DcMotor.class, motorNames[3]);

        motors = new DcMotor[]{backRightMotor, backLeftMotor, frontRightMotor, frontLeftMotor};
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            selectedMotorIndex = (selectedMotorIndex + 1) % motors.length;
        } else if (gamepad1.dpad_down) {
            selectedMotorIndex = (selectedMotorIndex - 1 + motors.length) % motors.length;
        }

        for (int i = 0; i < motors.length; i++) {
            if (i == selectedMotorIndex) {
                motors[i].setPower(motorPower);
            } else {
                motors[i].setPower(0);
            }
        }

        telemetry.addData("Selected Motor", motorNames[selectedMotorIndex]);
        telemetry.addData("Motor Power", motorPower);
        telemetry.update();
    }
}
