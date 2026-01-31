package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.RevolverSubsystem;
@Disabled
@TeleOp(name = "Decode TeleOp", group = "Decode")
public class DecodeTeleOp extends OpMode {

    private RevolverSubsystem revolver;

    @Override
    public void init() {
        revolver = new RevolverSubsystem(hardwareMap);
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void loop() {
        // Inputs
        // Intake: Right Trigger (Toggle or Hold? Prompt says "gamepad1.right_trigger: Intake". 
        // We implemented setIntakeEnabled(boolean).
        // Let's assume Hold logic: if trigger > 0.1, enabled.
        revolver.setIntakeEnabled(gamepad1.right_trigger > 0.1);

        // Shooter Trigger: Left Trigger
        // If aligned and ready, this confirms the shot.
        if (gamepad1.left_trigger > 0.1) {
            revolver.confirmShoot();
        } 
        // Note: Manual Shooter Power overriding is removed as per prompt "Autonomously turn on shooter, then wait for trigger".
        // The subsystem handles shooter power in SHOOTING_ALIGN and READY_TO_SHOOT states.

        // Shoot Requests
        if (gamepad1.cross) {
            revolver.requestShootGreen();
        }
        if (gamepad1.circle) {
            revolver.requestShootPurple();
        }

        // Subsystem Update
        revolver.update();

        // Telemetry
        telemetry.addData("Shooter Power", gamepad1.left_trigger > 0.1 ? 1.0 : 0.0);
        telemetry.addData("Intake Request", gamepad1.right_trigger > 0.1);
        telemetry.addData("State", revolver.getState());
        telemetry.addData("Inventory", revolver.getInventoryString());
        telemetry.addData("Intake Slot", revolver.getIntakeSlot());
        telemetry.addData("Target Slot", revolver.getTargetSlot());
        telemetry.addData("Indexer Pos", revolver.getIndexerCurrentPos());
        telemetry.addData("Indexer Target", revolver.getIndexerTargetPos());
        telemetry.addData("Detected Color", revolver.getCurrentColor());
        telemetry.addData("Distance (CM)", "%.2f", revolver.getDistance());
        telemetry.addData("RGB", "%d, %d, %d", revolver.getRed(), revolver.getGreen(), revolver.getBlue());
        telemetry.update();
    }
}
