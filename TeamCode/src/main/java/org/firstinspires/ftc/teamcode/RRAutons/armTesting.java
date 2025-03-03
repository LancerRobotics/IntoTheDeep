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
public class armTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,Math.toRadians(0)));

        Arm arm = new Arm(true, true, 0.43, hardwareMap);
        Movement movement = new Movement(hardwareMap);

        waitForStart(); // This is required in all autons!!!




        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(0, 0,Math.toRadians(180)))
                        .stopAndAdd(new Arm(true, true, 0.75, hardwareMap))
                        .waitSeconds(0.75)
                        .stopAndAdd(() -> arm.stopMotors())
                        .build()
        );

    }
}
