package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LancersRobot {
    private final HardwareMap hardwareMap;
    public LancersRobot (final HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    // hook servo

    double openServoPosition = 0.15;
    double closeServoPosition = 0.515;
    final double servoSpeed = 0.01;

    public void extendServo() {
        changeServoPosition(servoSpeed);
    }

    public void retrieveServo() {
        changeServoPosition(-servoSpeed);
    }

    public void changeServoPosition(double servoSpeed) {
        final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);
        double currentServoPosition = hookServo.getPosition();
        if((servoSpeed < 0 && currentServoPosition > openServoPosition)
                || (servoSpeed > 0 && currentServoPosition < closeServoPosition)) {
            currentServoPosition += servoSpeed;
        }
        setServoPosition(currentServoPosition);
    }

    public void setServoPosition(double currentServoPosition) {
        final Servo hookServo = hardwareMap.servo.get(LancersBotConfig.HOOK_SERVO);
        hookServo.setPosition(currentServoPosition);
    }
}
