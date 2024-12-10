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
            robot.pauseMotors();
            sleep(200);
            robot.slidesMovement(true);
            robot.pauseMotors();
            sleep(200);
            robot.clawOpen(); // by this point, the block should be in the low basket
            robot.pauseMotors();
            sleep(400);
            robot.slidesMovement(false);
            robot.pauseMotors();
            sleep(300);
        }
    }
}
