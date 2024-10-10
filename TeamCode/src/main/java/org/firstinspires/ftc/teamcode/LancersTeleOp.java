package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public final class LancersTeleOp extends LinearOpMode {

    private void mecanumMovement(final Gamepad gamepad) {
        final double counteractValue = 1.05; // Value to counteract imperfect strafing
        //Declaration of motors
        final DcMotor frontLeftMotor = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
        final DcMotor rearLeftMotor = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
        final DcMotor frontRightMotor = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
        final DcMotor rearRightMotor = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        final double ly = -gamepad.left_stick_y; // Y stick values are reversed
        final double lx = gamepad.left_stick_x * counteractValue; // Counteract imperfect strafing
        final double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // Ensures all power maintain the same ratio, but only when
        // at least one is out of range [-1, 1]
        final double denominator = Math.max(1, Math.abs(ly) + Math.abs(lx) + Math.abs(rx));
        final double frontLeftPower = (ly + lx + rx) / denominator;
        final double frontRightPower = (ly - lx - rx);
        final double rearLeftPower = (ly - lx + rx);
        final double rearRightPower = (ly + lx - rx);

        frontLeftMotor.setPower(frontLeftPower);
        rearLeftMotor.setPower(rearLeftPower);
        frontRightMotor.setPower(frontRightPower);
        rearRightMotor.setPower(rearRightPower);
    }

    @Override
    public void runOpMode(){
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()) {
            mecanumMovement(gamepad1);
        }
    }
}