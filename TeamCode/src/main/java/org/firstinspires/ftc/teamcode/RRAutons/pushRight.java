package org.firstinspires.ftc.teamcode.RRAutons;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="pushRight")
public class pushRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart(); // This is required in all autons!!!

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0,0,0))
                        .strafeTo(new Vector2d(0, -8))
                        .strafeTo(new Vector2d(60,-8))
                        .strafeTo(new Vector2d(60, -16))
                        .strafeTo(new Vector2d(10,-16))

                        .strafeTo(new Vector2d(60,-16))
                        .strafeTo(new Vector2d(60,-28))
                        .strafeTo(new Vector2d(10,-28))

                        .strafeTo(new Vector2d(60,-28))
                        .strafeTo(new Vector2d(60,-36))
                        .strafeTo(new Vector2d(10,-36))
                        .build()
        );

    }
}
