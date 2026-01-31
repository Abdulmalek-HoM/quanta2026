package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.SimpleRevolver;
@Disabled
@TeleOp(name = "Simple TeleOp", group = "Test")
public class SimpleTeleOp extends OpMode {

    private SimpleRevolver revolver;
    
    private boolean lastCross = false;
    private boolean lastCircle = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    private boolean isShooterOn = false;
    private boolean lastLeftBumper = false;

    @Override
    public void init() {
        revolver = new SimpleRevolver(hardwareMap);
        telemetry.addData("Status", "Simple Mode Ready");
    }

    @Override
    public void loop() {
        // 1. Intake Control (Triggers)
        // Right Trigger = Intake (1.0)
        // Left Trigger = Outtake / Reverse (-1.0)
        // Priority to Intake if both pressed? Or stop? Let's prioritize Outtake (Left) if undefined, or just simple if/else if.
        if (gamepad1.right_trigger > 0.1) {
             revolver.setIntakePower(1.0);
        } else if (gamepad1.left_trigger > 0.1) {
             revolver.setIntakePower(-1.0);
        } else {
             revolver.setIntakePower(0);
        }

        // 2. Index / Load (Press Cross) - Moves 1 Slot
        if (gamepad1.cross && !lastCross) {
            revolver.moveNextSlot();
        }
        lastCross = gamepad1.cross;

        // 3. Shooter Toggle (Press Circle)
        // Toggles between 0.8 and 0 power
        if (gamepad1.circle && !lastCircle) {
            isShooterOn = !isShooterOn;
            if (isShooterOn) {
                revolver.setShooterPower(0.8);
            } else {
                revolver.setShooterPower(0);
            }
        }
        lastCircle = gamepad1.circle;
        
        // 4. Kick (Press Left Bumper)
        if (gamepad1.left_bumper && !lastLeftBumper) {
             revolver.kick();
        }
        lastLeftBumper = gamepad1.left_bumper;
        
        // 5. Manual Trim (D-Pad Up/Down) - Fixes Drift
        if (gamepad1.dpad_up && !lastUp) {
            revolver.manualAdjust(5); // Add 5 ticks
        }
        if (gamepad1.dpad_down && !lastDown) {
            revolver.manualAdjust(-5); // Remove 5 ticks
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        revolver.update();

        // Telemetry
        telemetry.addData("Shooter", isShooterOn ? "ON (0.8)" : "OFF");
        telemetry.addData("Target", revolver.getTargetPos());
        telemetry.addData("Current", revolver.getCurrentPos());
        telemetry.addData("Controls", "RT=In, LT=Out, LB=Kick, O=ShootToggle, X=Next");
        telemetry.update();
    }
}
