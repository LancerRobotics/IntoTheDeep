package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.LancersBotConfig;


public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0078;
        TwoWheelConstants.strafeTicksToInches = .001989436789;
        TwoWheelConstants.forwardY = -2.6;
        TwoWheelConstants.strafeX = -1.25;
        TwoWheelConstants.forwardEncoder_HardwareMapName = LancersBotConfig.FRONT_RIGHT_MOTOR;
        TwoWheelConstants.strafeEncoder_HardwareMapName = LancersBotConfig.FRONT_LEFT_MOTOR;
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = LancersBotConfig.IMU;
        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
    }
}




