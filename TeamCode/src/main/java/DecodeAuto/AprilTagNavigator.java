package DecodeAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagNavigator {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // Drive constants (Adjustable)
    final double DESIRED_DISTANCE = 12.0; 
    final double SPEED_GAIN  =  0.02;
    final double STRAFE_GAIN =  0.015;
    final double TURN_GAIN   =  0.01;
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    public AprilTagNavigator(HardwareMap hardwareMap, String webcamName) {
        initAprilTag(hardwareMap, webcamName);
    }

    private void initAprilTag(HardwareMap hardwareMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (webcamName != null) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                    .addProcessor(aprilTag)
                    .build();
        } else {
             visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /**
     * Scans for tags and returns the detected RandomizationPattern based on IDs found.
     * Returns UNKNOWN if no relevant tags are found.
     */
    public TagConfiguration.RandomizationPattern detectRandomizationPattern() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                TagConfiguration.RandomizationPattern pattern = TagConfiguration.getPatternFromId(detection.id);
                if (pattern != TagConfiguration.RandomizationPattern.UNKNOWN) {
                    return pattern;
                }
            }
        }
        return TagConfiguration.RandomizationPattern.UNKNOWN;
    }

    /**
     * Calculates drive powers to align with a specific tag.
     * Returns null if tag not found.
     * Returns array [drive, strafe, turn]
     */
    public double[] getAlignmentPowers(int targetTagId) {
        boolean targetFound = false;
        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == targetTagId) {
                targetFound = true;
                desiredTag = detection;
                break;
            }
        }

        if (targetFound) {
            double rangeError   = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError     = desiredTag.ftcPose.yaw;

            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            return new double[]{drive, strafe, turn};
        }

        return null;
    }

    public AprilTagDetection getDesiredTag() {
        return desiredTag;
    }
    
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
