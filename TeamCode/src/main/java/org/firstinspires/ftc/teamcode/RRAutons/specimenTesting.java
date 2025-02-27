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
                drive.actionBuilder(new Pose2d(0,0,Math.toRadians(90)))
                        // Get first block (already oriented and positioned to do so)

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(48, -14))
                        .strafeTo(new Vector2d(0,24))
                        // hook block?

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(0,-28))
                        // grab block
                        .turnTo(Math.toRadians(-90))
                        .strafeTo(new Vector2d(4,20))
                        // hook blocks?

                        .turnTo(Math.toRadians(90))
                        .strafeTo(new Vector2d(0,-28))
                        .build()
        );

    }
}
