package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="leftBasketOne")
public class leftBasketOne extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-39.5,-61.5,Math.toRadians(180)));

        Arm arm = new Arm(true, true, 0.43, hardwareMap);
        Movement movement = new Movement(hardwareMap);

        waitForStart(); // This is required in all autons!!!

        //.stopAndAdd(new Claw(false))
        //.stopAndAdd(new Claw(true))

        //.stopAndAdd(new Arm(true, true, 0.43))
        //.stopAndAdd(new Arm(false, true, 1.5))




        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-39.5, -61.5,Math.toRadians(180)))
                        .stopAndAdd(new Claw(false, hardwareMap))
                        .strafeTo(new Vector2d(-39.5, -55))
                        .strafeTo(new Vector2d(-59, -59))
                        .turnTo(Math.toRadians(225))

                        .waitSeconds(1)
                        //.strafeTo(new Vector2d(-10, -34))
                        .stopAndAdd(new Arm(true, true, 0.75, hardwareMap))
                        .waitSeconds(0.75)
                        .stopAndAdd(() -> arm.stopMotors())
                        .waitSeconds(1)
                        .stopAndAdd(new Arm(false, true, 1.5, hardwareMap))
                        .waitSeconds(1.5)
                        .stopAndAdd(() -> arm.stopMotors())

                        .stopAndAdd(new Claw(true, hardwareMap)) // put block in basket
                        .waitSeconds(1)
                        .stopAndAdd(new Arm(false, false, 2, hardwareMap))
                        .waitSeconds(2)
                        .stopAndAdd(() -> arm.stopMotors())
                        .stopAndAdd(new Arm(true, false, 0.5, hardwareMap))
                        .waitSeconds(0.5)
                        .stopAndAdd(() -> arm.stopMotors())

                        .stopAndAdd(new Arm(false, false, 1.5, hardwareMap))
                        .waitSeconds(1.5)
                        .stopAndAdd(() -> arm.stopMotors())


                        // PARKING TIME!!!!!!!!!!!!!
                        .strafeTo(new Vector2d(-57,-56))
                        .turnTo(Math.toRadians(0))
                        .strafeTo(new Vector2d(54,-56))
                        .build()
        );

    }
}
