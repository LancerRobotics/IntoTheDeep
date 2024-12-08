package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="rightPark", group="TimeBasedAutons")
@Config
public class rightPark extends LinearOpMode {
    // purpose: If the robot is on the right side of the field, it will park the robot
    // BOT SHOULD BE FACED TOWARDS THE PARKING SPOT

    LancersRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        final LancersRobot robot = new LancersRobot(hardwareMap);
        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.update();

        if (opModeIsActive()){

            robot.forward();

            robot.pause();

        }
    }
}
