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

    public leftBasket(){

    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new LancersRobot(hardwareMap);

        waitForStart();

        telemetry.addData("Status", "Its working :)");
        telemetry.update();

        if (opModeIsActive()){

            robot.strafeRight(0.5); // This is only working due to the weight imbalance
            sleep(150);
            robot.pauseMotors();
            robot.slidesMovement(true);
            sleep(5000);
            robot.pauseMotors();
            robot.clawOpen(); // by this point, the block should be in the low basket
            robot.pauseMotors();
            sleep(600);
            robot.slidesMovement(false);
            robot.pauseMotors();
            sleep(200);
            robot.backward();
            robot.pauseMotors();
            sleep(200);
            robot.backward();
            robot.pauseMotors(); // should be parked by now
            sleep(200);

        }
    }
}
