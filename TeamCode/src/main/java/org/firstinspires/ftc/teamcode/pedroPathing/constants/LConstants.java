package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.LancersBotConfig;
import org.firstinspires.ftc.teamcode.util.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder.*;
import com.qualcomm.hardware.rev.*;


public class LConstants {
    private Encoder parallelEncoder, forwardEncoder;
    public LConstants(HardwareMap hardwareMap){
        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_RIGHT_MOTOR));
        forwardEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LancersBotConfig.FRONT_LEFT_MOTOR));

    }

    static {
        TwoWheelConstants.forwardTicksToInches = .001989436789;
        TwoWheelConstants.strafeTicksToInches = .001989436789;
        TwoWheelConstants.forwardY = 1;
        TwoWheelConstants.strafeX = -2.5;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "frontLeft";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "frontRight";
        TwoWheelConstants.forwardEncoderDirection = -1;
        TwoWheelConstants.strafeEncoderDirection = 1;
        TwoWheelConstants.IMU_HardwareMapName = "imu";
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




