package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="leftBasket", group="TimeBasedAutons")
@Config
public class leftBasket extends LinearOpMode {
    // purpose: If the robot is on the left side of the field, it will try to score a low basket
    // BOT SHOULD BE FACED TOWARDS THE BASKET

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
