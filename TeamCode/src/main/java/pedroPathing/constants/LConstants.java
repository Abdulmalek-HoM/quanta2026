package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.sun.tools.javac.comp.Todo;

public class LConstants {
    static {

//        old unstable values
//        DriveEncoderConstants.forwardTicksToInches = 0.01406667; //0.01430.0 + 0.0129 +0.015 /3
//        DriveEncoderConstants.strafeTicksToInches = 0.13716675; //0.071 + 0.0466 + 0.003963910686823792 + 0.03467612777483593
//        DriveEncoderConstants.turnTicksToInches = 0.01385; // 0.0136 + 0.0141 + 0.0139 + 0.0138/4

//        new values
        DriveEncoderConstants.forwardTicksToInches = 0.00476666667; // 0.0048 + 0.0048 + 0.0047 / 3
        DriveEncoderConstants.strafeTicksToInches = 0.00653333333; //-0.0057 - -0.0057 -0.0055 - 0.0055 - 0.0056 - 0.0056 - 0.0056
        // Auto Tuner Strafing is incorrect due to imbalanced weight between front and back.
        DriveEncoderConstants.turnTicksToInches = 0.0098; // 0.0099 + 0.0097 + 0.0098 // or 0.010125

        DriveEncoderConstants.robot_Width = 14.33071;
        DriveEncoderConstants.robot_Length = 10.43307;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.REVERSE;


    }
}
