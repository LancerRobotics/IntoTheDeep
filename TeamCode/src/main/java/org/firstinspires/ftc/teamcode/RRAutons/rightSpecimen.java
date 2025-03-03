package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous(name="rightSpecimen")
public class rightSpecimen extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(49,-61.5,Math.toRadians(180)));

        Arm arm = new Arm(true, true, 0.43, hardwareMap);
        Movement movement = new Movement(hardwareMap);

        waitForStart(); // This is required in all autons!!!

        //.stopAndAdd(new Claw(false))
        //.stopAndAdd(new Claw(true))

        //.stopAndAdd(new Arm(true, true, 0.43))
        //.stopAndAdd(new Arm(false, true, 1.5))




        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(49, -61.5,Math.toRadians(180)))
                        .strafeTo(new Vector2d(20, -41))
                        .turnTo(Math.toRadians(90))


                        .stopAndAdd(new Claw(false, hardwareMap))
                        .waitSeconds(3)
                        //.strafeTo(new Vector2d(-10, -34))
                        .stopAndAdd(new Arm(true, true, 0.43, hardwareMap))
                        .waitSeconds(0.43)
                        .stopAndAdd(() -> arm.stopMotors())
                        .waitSeconds(3)
                        .stopAndAdd(new Arm(false, true, 1.5, hardwareMap))
                        .waitSeconds(1.5)
                        .stopAndAdd(() -> arm.stopMotors())
                        .waitSeconds(2)
                        .stopAndAdd(new Arm(true, false, 0.25, hardwareMap))
                        .stopAndAdd(new Movement(hardwareMap))
                        .waitSeconds(0.25)
                        .stopAndAdd(() -> movement.stopMotors())
                        .stopAndAdd(() -> arm.stopMotors())
                        .stopAndAdd(new Claw(true, hardwareMap))
                        .stopAndAdd(new Arm(true, false, 0.4, hardwareMap))
                        .waitSeconds(0.2)
                        .stopAndAdd(() -> arm.stopMotors())
                        /*
                        // hook
                        .turnTo(Math.toRadians(270))
                        .strafeTo(new Vector2d(54,-48.5)) // spline through it
                        // grab block

                        .strafeTo(new Vector2d(-5, -34))
                        .turnTo(Math.toRadians(90))
                        // hook

                        .turnTo(Math.toRadians(270))
                        .strafeTo(new Vector2d(54,-48.5))
                        //grab block

                        .strafeTo(new Vector2d(0, -34))
                        .turnTo(Math.toRadians(90))
                        // hook

                        .turnTo(Math.toRadians(270))
                        .strafeTo(new Vector2d(54,-48.5))
                        // grab block

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(5, -34))
                        //hook

                        // PARKING TIME!!!!!!!!!!!!!
                        .strafeTo(new Vector2d(60,-56))*/
                        .build()
        );

    }
}
