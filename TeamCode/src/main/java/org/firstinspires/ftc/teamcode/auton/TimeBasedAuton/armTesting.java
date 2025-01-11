package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="armTesting", group="TimeBasedAutons")
@Config
public class armTesting extends LinearOpMode {
    // purpose: If the robot is on the left side of the field, it will park the robot
    // BOT SHOULD BE FACED TOWARDS THE PARKING SPOT

    LancersRobot robot;

    public armTesting(){

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
            sleep(450);
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

            //claw open
            robot.clawOpen(); // by this point, the block should be in the low basket
            sleep(400);
            robot.pauseMotors();



            //ALL GOOD
        }
    }
}
