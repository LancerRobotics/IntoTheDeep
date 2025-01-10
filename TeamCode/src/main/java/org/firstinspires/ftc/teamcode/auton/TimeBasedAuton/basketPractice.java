package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="basketPractice", group="TimeBasedAutons")
@Config
public class basketPractice extends LinearOpMode {
    // THIS IS FOR TESTING SLIDES ONLY

    private LancersRobot robot = null;

    public basketPractice(){
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LancersRobot(hardwareMap);
        int power = 1;

        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.addData("Slides power", power);
        telemetry.update();

        if (opModeIsActive()){
            robot.slidesMovement(true, power);
            robot.pauseMotors();
            sleep(300);
            robot.clawOpen();
            robot.pauseMotors();
            sleep(300);
        }


    }
}
