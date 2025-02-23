package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="leftParkDelayed", group="TimeBasedAutons")
@Config
public class leftParkDelayed extends LinearOpMode {
    // purpose: If the robot is on the left side of the field, it will park the robot
    // BOT SHOULD BE FACED TOWARDS THE PARKING SPOT

    LancersRobot robot;

    public leftParkDelayed(){

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LancersRobot(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.update();

        if (opModeIsActive()){
            sleep(20000); // Waits 20 seconds
            robot.forward(0.5);
            sleep(2350);
            robot.pauseMotors();



            //ALL GOOD
        }
    }
}
