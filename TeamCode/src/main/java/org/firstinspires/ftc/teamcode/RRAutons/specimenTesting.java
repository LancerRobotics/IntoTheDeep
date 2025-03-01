package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="specimenTesting")
public class specimenTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(90)));

        waitForStart(); // This is required in all autons!!!

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(10, -70,Math.toRadians(180)))
                        .strafeTo(new Vector2d(-10, -34))
                        // hook

                        .turnTo(grabAngle)
                        .strafeTo(getPos) // spline through it
                        // grab block

                        .strafeTo(new Vector2d(-5, -34))
                        .turnTo(hookAngle)
                        // hook

                        .turnTo(grabAngle)
                        .strafeTo(getPos)
                        //grab block

                        .strafeTo(new Vector2d(0, -34))
                        .turnTo(hookAngle)
                        // hook

                        .turnTo(grabAngle)
                        .strafeTo(getPos)
                        // grab block

                        .turnTo(hookAngle)
                        .strafeTo(new Vector2d(5, -34))
                        //hook

                        // PARKING TIME!!!!!!!!!!!!!
                        .strafeTo(new Vector2d(60,-56))
                        .build()
        );

    }
}
