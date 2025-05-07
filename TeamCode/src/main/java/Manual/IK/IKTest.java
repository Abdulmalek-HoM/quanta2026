//package Manual.IK;
//
//import android.util.Size;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
//import org.firstinspires.ftc.vision.opencv.ColorRange;
//import org.firstinspires.ftc.vision.opencv.ImageRegion;
//import org.opencv.core.RotatedRect;
//
//import java.util.List;
//
//@TeleOp(name="IK Test")
//public class IKTest extends LinearOpMode {
//    public void runOpMode() {
//        // Init hardware
//        waitForStart();
//
//        // Test positions
//        ArmSolution test1 = calculateIK(0.3, 0, 0.035, 45);  // Forward
//        ArmSolution test2 = calculateIK(-0.2, 0.15, 0.035, -30);  // Diagonal
//
//        moveToPosition(test1);
//        sleep(3000);
//        moveToPosition(test2);
//    }
//}
