package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.SimpleRevolver;

@TeleOp(name = "Simple TeleOp", group = "Test")
public class SimpleTeleOp extends OpMode {

    private SimpleRevolver revolver;
    
    private boolean lastCross = false;
    private boolean lastCircle = false;
    private boolean lastUp = false;
    private boolean lastDown = false;

    @Override
    public void init() {
        revolver = new SimpleRevolver(hardwareMap);
        telemetry.addData("Status", "Simple Mode Ready");
    }

    @Override
    public void loop() {
        // 1. Intake (Hold)
        if (gamepad1.right_trigger > 0.1) {
            revolver.setIntakePower(1.0);
        } else {
            revolver.setIntakePower(0);
        }

        // 2. Index / Load (Press Cross) - Moves 1 Slot
        if (gamepad1.cross && !lastCross) {
            revolver.moveNextSlot();
        }
        lastCross = gamepad1.cross;

        // 3. Shoot (Press Circle) - Spin & Kick only (No Rotation)
        if (gamepad1.circle && !lastCircle) {
            // revolver.moveToShooter(); // Removed per user request
            revolver.setShooterPower(0.7);
            revolver.kick();
        }
        lastCircle = gamepad1.circle;
        
        // 4. Kick (Press Triangle or Trigger)
        // Separate button for safety? Or combine with Circle?
        // Let's use Left Trigger for Kick (User familiar)
        if (gamepad1.left_trigger > 0.1) {
             revolver.kick();
        }
        
        // 5. Manual Trim (D-Pad Up/Down) - Fixes Drift
        if (gamepad1.dpad_up && !lastUp) {
            revolver.manualAdjust(5); // Add 5 ticks
        }
        if (gamepad1.dpad_down && !lastDown) {
            revolver.manualAdjust(-5); // Remove 5 ticks
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Stop shooter if logic done? 
        // For now, let's keep it manual toggle or just ON during align.
        // Let's say: If Kicking is done, maybe user wants to stop shooter manually?
        // Or hold Left Bumper to stop shooter?
        if (gamepad1.left_bumper) {
            revolver.setShooterPower(0);
        }

        revolver.update();

        // Telemetry
        telemetry.addData("Target", revolver.getTargetPos());
        telemetry.addData("Current", revolver.getCurrentPos());
        telemetry.addData("Diff", revolver.getTargetPos() - revolver.getCurrentPos());
        telemetry.addData("Instr", "X=Idx, O=Align, LTrig=Kick, LB=StopShoot");
        telemetry.update();
    }
}
