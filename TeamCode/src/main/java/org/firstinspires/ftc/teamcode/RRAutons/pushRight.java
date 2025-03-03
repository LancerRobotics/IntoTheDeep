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
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(25, -61.5, Math.toRadians(0)));

        waitForStart(); // This is required in all autons!!!

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(25, -61.5, Math.toRadians(0)))
                        .strafeTo(new Vector2d(25, -57))
                        .strafeTo(new Vector2d(36, -57))
                        .turnTo(Math.toRadians(90))

                        .strafeTo(new Vector2d(36,0))
                        .strafeTo(new Vector2d(45, 0))
                        .strafeTo(new Vector2d(45, -59))

                        .strafeTo(new Vector2d(45,0))
                        .strafeTo(new Vector2d(53,0))
                        .strafeTo(new Vector2d(53,-57))

                        .strafeTo(new Vector2d(53,0))
                        .strafeTo(new Vector2d(61,0))
                        .strafeTo(new Vector2d(61,-55))
                        .build()
        );

    }
}
