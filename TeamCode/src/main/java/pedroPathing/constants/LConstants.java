package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;

public class LConstants {
    static {
        DriveEncoderConstants.forwardTicksToInches = 0.01406667; //0.01430.0 + 0.0129 +0.015 /3
        DriveEncoderConstants.strafeTicksToInches = 0.13716675; //0.071 + 0.0466 + 0.003963910686823792 + 0.03467612777483593
        DriveEncoderConstants.turnTicksToInches = 0.01385; // 0.0136 + 0.0141 + 0.0139 + 0.0138/4

        DriveEncoderConstants.robot_Width = 14.33071;
        DriveEncoderConstants.robot_Length = 10.43307;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.REVERSE;


    }
}
