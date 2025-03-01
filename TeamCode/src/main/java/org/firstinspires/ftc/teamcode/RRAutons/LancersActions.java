package org.firstinspires.ftc.teamcode.RRAutons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.LancersRobot;



public class Servo implements Action {
    boolean open;

    public Servo(boolean open) {
        this.open = open;
        this.servo = s;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if(open) {
            LancersRobot.extendServo();
            packet.put("Auton_Servo", "Extended");
        }
        else {
            LancersRobot.retrieveServo();
            packet.put("Auton_Servo", "Retrieved");
        }
        return false;

    }
}

public class LancersActions {
    private MecanumDrive drive;

    public LancersActions(MecanumDrive drive) {
        this.drive = drive;
    }
}

