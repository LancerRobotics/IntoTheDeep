package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="slidesTesting", group="TimeBasedAutons")
@Config
public class slidesTesting extends LinearOpMode {
    // THIS IS FOR TESTING SLIDES ONLY

    private LancersRobot robot = null;

    public slidesTesting(){
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LancersRobot(hardwareMap);
        double power = 0.4;

        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.addData("Slides power", power);
        telemetry.update();

        if (opModeIsActive()){
            robot.slidesMovement(true, 0.4);
            sleep(2000);
            robot.pauseMotors();
            power = 0.2;
            robot.slidesMovement(false, 0.2);
            sleep(1000);
            robot.pauseMotors();
        }


    }
}
