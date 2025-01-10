package org.firstinspires.ftc.teamcode.auton.TimeBasedAuton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LancersRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="rightBasket", group="TimeBasedAutons")
@Config
public class rightBasket extends LinearOpMode {
    // purpose: If the robot is on the left side of the field, it will try to score a low basket
    // BOT SHOULD BE FACED TOWARDS THE BASKET

    LancersRobot robot;

    public rightBasket(){

    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LancersRobot(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.update();

        if (opModeIsActive()){

            robot.forward(0.6);
            sleep(200);
            robot.pauseMotors();
            robot.slidesMovement(true);
            sleep(200);
            robot.pauseMotors();
            robot.clawOpen(); // by this point, the block should be in the low basket
            sleep(400);
            robot.pauseMotors();
            robot.slidesMovement(false);
            sleep(300);
            robot.pauseMotors();
        }
    }
}
