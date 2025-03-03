package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name="leftBasket", group="TimeBasedAutons")
@Config
public class leftBasket extends LinearOpMode {
    // purpose: If the robot is on the left side of the field, it will try to score a low basket
    // BOT SHOULD BE FACED FORWARDS

    LancersRobot robot;

    public leftBasket(){

    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LancersRobot(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.addData("trackedExtensionRadians", robot.trackedExtensionRadians);
        telemetry.update();

        if (opModeIsActive()){
            robot.rotateArm(-0.5);
            sleep(325);
            robot.pauseMotors();
            sleep(1000);
            robot.extendUntilMax(); //no sleep function is needed
            robot.pauseMotors();
            telemetry.addData("trackedExtensionRadians", robot.trackedExtensionRadians);
            telemetry.update();

            robot.rotateArm(0.4);
            sleep(1700);
            robot.pauseMotors();

            //movement here
            robot.forward(0.5);
            sleep(392);

            robot.pauseMotors();
            robot.strafeLeft(0.5);
            sleep(392);
            robot.pauseMotors();

            robot.turnLeft(0.5);
            sleep(800);
            robot.pauseMotors();

            robot.turnLeft(0.5);
            sleep(800);
            robot.pauseMotors();

            robot.forward(0.5);
            sleep(200);
            robot.pauseMotors();
            //claw open
            robot.clawOpen(); // by this point, the block should be in the low basket
            sleep(400);
            robot.pauseMotors();



            //ALL GOOD
        }
    }
}
