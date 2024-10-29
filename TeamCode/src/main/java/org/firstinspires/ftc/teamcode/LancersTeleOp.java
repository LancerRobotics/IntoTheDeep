package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.LancersBotConfig;

@TeleOp()
public class LancersTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException  {
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            final double speedMultiplier = (gamepad1.a ? 1.0d : 0.8d);

            final DcMotor leftFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_LEFT_MOTOR);
            final DcMotor leftRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_LEFT_MOTOR);
            final DcMotor rightFront = hardwareMap.dcMotor.get(LancersBotConfig.FRONT_RIGHT_MOTOR);
            final DcMotor rightRear = hardwareMap.dcMotor.get(LancersBotConfig.REAR_RIGHT_MOTOR);

            // Gamepad positions; Motors are swapped
            final double ly = -gamepad1.left_stick_y * speedMultiplier; // Remember, Y stick value is reversed
            final double lx = -gamepad1.left_stick_x * speedMultiplier; // Counteract imperfect strafing
            final double rx = gamepad1.right_stick_x * speedMultiplier;


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

        }
    }
}