package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Arrays;

public class RevolverSubsystem {

    // Hardware
    private DcMotorEx indexer;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo kicker;
    private RevColorSensorV3 sensor;

    // Constants
    private static final int TICKS_PER_SLOT = 96; // 120 degrees
    private static final int TICKS_TO_SHOOTER = 192; // 240 degrees (2 jumps)
    private static final int TICKS_PER_REVOLUTION = 288; // 360 degrees
    private static final double INDEXER_POWER = 0.3;
    private static final double INTAKE_POWER = 1.0;
    private static final double KICKER_EJECT_POS = 1.0;
    private static final double KICKER_RETRACT_POS = 0.0;
    private static final double COLOR_DETECT_DISTANCE_CM = 3.5; // Reduced to avoid false positives

    // State Machine
    public enum RevolverState {
        LOADING, // Aligning an EMPTY slot to Intake (0 deg)
        INDEXING, // Moving to the next EMPTY slot
        WAIT_FOR_INTAKE_CLEAR, // Wait for ball to clear or time delay
        SHOOTING_ALIGN, // Rotating a specific color slot to Shooter (240 deg)
        READY_TO_SHOOT, // Aligned, waiting for manual trigger
        SHOOTING, // Activating Kicker sequence
        FULL // All slots full, waiting for shoot command
    }

    private RevolverState currentState = RevolverState.LOADING;

    // Inventory Management
    public enum SlotColor {
        EMPTY,
        GREEN,
        PURPLE,
        UNKNOWN
    }

    // Slots: 0, 1, 2.
    private volatile SlotColor[] inventory = new SlotColor[] { SlotColor.EMPTY, SlotColor.EMPTY, SlotColor.EMPTY };

    // Navigation
    private int currentIntakeSlot = 0; // Which slot is currently at the Intake position?
    private long globalTargetTicks = 0; // Absolute tick target

    // Logic Variables
    private long shootingTimer = 0;
    private long waitTimer = 0;
    private boolean shootGreenRequest = false;
    private boolean shootPurpleRequest = false;
    private boolean manualTrigger = false;
    private boolean intakeEnabled = false;
    private SlotColor lastDetectedColor = SlotColor.EMPTY;

    // Alignment Safety
    private boolean aligningStarted = false; // Prevents infinite recalculation

    public RevolverSubsystem(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");

        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setTargetPosition(0);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setVelocityPIDFCoefficients(40, 5, 5, 10);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        // 1. Read Sensors
        double distance = sensor.getDistance(DistanceUnit.CM);
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        SlotColor detected = determineColor(distance, r, g, b);
        lastDetectedColor = detected;

        // 2. State Machine
        switch (currentState) {
            case LOADING:
                kicker.setPosition(KICKER_RETRACT_POS);
                aligningStarted = false;

                // Reset shoot requests to prevent "Runaway" rotation where it immediately goes
                // to shooter
                shootGreenRequest = false;
                shootPurpleRequest = false;

                // Removed recalibrateIfNecessary() to prevent position drift from blind resets

                if (inventory[currentIntakeSlot] == SlotColor.EMPTY) {
                    if (intakeEnabled)
                        intake.setPower(INTAKE_POWER);
                    else
                        intake.setPower(0);

                    if (intakeEnabled && detected != SlotColor.EMPTY && detected != SlotColor.UNKNOWN) {
                        inventory[currentIntakeSlot] = detected;
                        currentState = RevolverState.INDEXING;
                    }
                } else {
                    currentState = RevolverState.INDEXING;
                }

                checkShootRequests();
                break;

            case INDEXING:
                intake.setPower(0);
                aligningStarted = false;

                int nextEmpty = findNextEmptySlot(currentIntakeSlot);

                if (nextEmpty == -1) {
                    currentState = RevolverState.FULL;
                } else {
                    // Determine steps needed: 1 or 2 slots forward.
                    int stepsNeeded = calculateStepsForward(currentIntakeSlot, nextEmpty);
                    long newTarget = globalTargetTicks + (stepsNeeded * TICKS_PER_SLOT);

                    if (!indexer.isBusy() && indexer.getTargetPosition() != (int) newTarget) {
                        globalTargetTicks = newTarget;
                        moveToTarget(globalTargetTicks);
                    }

                    // Check Arrival
                    long distRemaining = Math.abs(globalTargetTicks - indexer.getCurrentPosition());
                    if (!indexer.isBusy() || distRemaining < 5) {
                        currentIntakeSlot = nextEmpty; // We arrived
                        waitTimer = System.currentTimeMillis();
                        currentState = RevolverState.WAIT_FOR_INTAKE_CLEAR;
                    }
                }
                checkShootRequests();
                break;

            case WAIT_FOR_INTAKE_CLEAR:
                intake.setPower(0);
                if (System.currentTimeMillis() - waitTimer > 500) {
                    currentState = RevolverState.LOADING;
                }
                checkShootRequests();
                break;

            case FULL:
                intake.setPower(0);
                checkShootRequests();
                break;

            case SHOOTING_ALIGN:
                intake.setPower(0);
                setShooterPower(1.0);

                if (!aligningStarted) {
                    // Start Alignment Logic - CALCULATE ONCE!

                    // 1. Where is the target slot relative to Intake (0)?
                    int targetRelativePos = (targetShootSlot - currentIntakeSlot + 3) % 3;

                    // 2. Shooter is at Position 2 (240 degrees relative to Intake).
                    // We need to move the revolver so that 'targetRelativePos' reaches '2'.
                    // Steps Forward = (Destination - CurrentRelative + 3) % 3
                    int stepsToShooter = (2 - targetRelativePos + 3) % 3;

                    // 3. Set Target
                    globalTargetTicks = globalTargetTicks + (stepsToShooter * TICKS_PER_SLOT);
                    moveToTarget(globalTargetTicks);

                    aligningStarted = true; // Lock calculation
                }

                // STRICT Arrival Check
                long error = Math.abs(globalTargetTicks - indexer.getCurrentPosition());

                // Wait until motor stops AND we are close
                if (!indexer.isBusy() && error < 5) {
                    // We are aligned!
                    // Calculate where our Intake Slot is now (it moved 'stepsToShooter')
                    int targetRelativePos = (targetShootSlot - currentIntakeSlot + 3) % 3;
                    int stepsMoved = (2 - targetRelativePos + 3) % 3;

                    currentIntakeSlot = (currentIntakeSlot + stepsMoved) % 3;

                    currentState = RevolverState.READY_TO_SHOOT;
                }
                break;

            case READY_TO_SHOOT:
                intake.setPower(0);
                setShooterPower(1.0); // Keep spinning

                if (manualTrigger) {
                    currentState = RevolverState.SHOOTING;
                    manualTrigger = false;
                    shootingTimer = 0;
                }
                break;

            case SHOOTING:
                if (shootingTimer == 0)
                    shootingTimer = System.currentTimeMillis();
                long elapsed = System.currentTimeMillis() - shootingTimer;

                if (elapsed < 600) { // Increased from 250 to 600ms for full extension
                    kicker.setPosition(KICKER_EJECT_POS);
                } else if (elapsed < 1000) { // Increased retract delay
                    kicker.setPosition(KICKER_RETRACT_POS);
                } else {
                    setShooterPower(0);
                    inventory[targetShootSlot] = SlotColor.EMPTY;
                    shootGreenRequest = false;
                    shootPurpleRequest = false;
                    currentState = RevolverState.LOADING;
                    shootingTimer = 0;
                }
                break;
        }
    }

    // Command Variables
    private int targetShootSlot = -1;

    // recalibrateIfNecessary removed to prevent drift

    private int calculateStepsForward(int current, int target) {
        return (target - current + 3) % 3;
    }

    private void checkShootRequests() {
        if (shootGreenRequest) {
            findAndAlignShooter(SlotColor.GREEN);
        } else if (shootPurpleRequest) {
            findAndAlignShooter(SlotColor.PURPLE);
        }
    }

    private void findAndAlignShooter(SlotColor color) {
        for (int i = 0; i < 3; i++) {
            if (inventory[i] == color) {
                targetShootSlot = i; // Target is set
                aligningStarted = false; // Reset flag so SHOOTING_ALIGN calculates
                manualTrigger = false; // Prevent phantom kicks from stale commands
                currentState = RevolverState.SHOOTING_ALIGN;
                return;
            }
        }
    }

    private int findNextEmptySlot(int current) {
        for (int i = 1; i <= 3; i++) {
            int next = (current + i) % 3;
            if (inventory[next] == SlotColor.EMPTY)
                return next;
        }
        return -1; // Full
    }

    private void moveToTarget(long ticks) {
        indexer.setTargetPosition((int) ticks);
        indexer.setPower(INDEXER_POWER);
    }

    public void confirmShoot() {
        this.manualTrigger = true;
    }

    // --- Helpers & Telemetry ---

    public RevolverState getState() {
        return currentState;
    }

    public String getInventoryString() {
        return Arrays.toString(inventory);
    }

    public int getIntakeSlot() {
        return currentIntakeSlot;
    }

    private SlotColor determineColor(double dist, int r, int g, int b) {
        // DISTANCE CHECK: Must be close to detect
        if (dist > COLOR_DETECT_DISTANCE_CM)
            return SlotColor.EMPTY;

        // MINIMUM BRIGHTNESS: Avoid scattered light false positives
        int totalBrightness = r + g + b;
        if (totalBrightness < 150)
            return SlotColor.EMPTY;

        // Based on latest sensor data (5 purple samples):
        // GREEN: G=86, R=47, B=55 → G dominates both R and B by 30+
        // PURPLE: G=92-389, R=58-258, B=79-500
        // Key pattern: R is ALWAYS lowest, G and B are both significantly higher
        // G-B difference varies widely (13 to 111), so we DON'T constrain it

        // GREEN DETECTION: Green dominates BOTH Red and Blue
        if (g > (r + 30) && g > (b + 30)) {
            return SlotColor.GREEN;
        }

        // PURPLE DETECTION: Red is significantly lower than BOTH Green and Blue
        // Don't care about the G-B relationship
        if (r < (g - 20) && r < (b - 20)) {
            return SlotColor.PURPLE;
        }

        return SlotColor.UNKNOWN;
    }

    // Command Variables for Telemetry
    public SlotColor getCurrentColor() {
        return lastDetectedColor;
    }

    /**
     * Read color sensor NOW (not cached). Use this in autonomous
     * when update() is not being called.
     */
    public SlotColor readColorNow() {
        double dist = sensor.getDistance(DistanceUnit.CM);
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        return determineColor(dist, r, g, b);
    }

    public int getRed() {
        return sensor.red();
    }

    public int getGreen() {
        return sensor.green();
    }

    public int getBlue() {
        return sensor.blue();
    }

    public double getDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public long getTargetSlot() {
        return globalTargetTicks / TICKS_PER_SLOT;
    } // Approx

    public int getIndexerCurrentPos() {
        return indexer.getCurrentPosition();
    }

    public long getIndexerTargetPos() {
        return globalTargetTicks;
    }

    // External Controls
    public void setShooterPower(double power) {
        shooter.setVelocity(power * 3000);
        shooter.setPower(power);
    }

    public void requestShootGreen() {
        shootGreenRequest = true;
        shootPurpleRequest = false;
    }

    public void requestShootPurple() {
        shootPurpleRequest = true;
        shootGreenRequest = false;
    }

    public void setIntakeEnabled(boolean enabled) {
        this.intakeEnabled = enabled;
    }

    /**
     * Preload the inventory for autonomous.
     * This marks slots as filled so the shooting logic will work.
     * 
     * @param slot0 Color of slot 0
     * @param slot1 Color of slot 1
     * @param slot2 Color of slot 2
     */
    public void preloadInventory(SlotColor slot0, SlotColor slot1, SlotColor slot2) {
        inventory[0] = slot0;
        inventory[1] = slot1;
        inventory[2] = slot2;
    }

    /**
     * Direct intake power control for autonomous.
     * Bypasses the state machine intake control.
     * 
     * @param power Motor power (-1.0 to 1.0)
     */
    public void setIntakePowerDirect(double power) {
        intake.setPower(power);
    }

    /**
     * Direct shooter power control for autonomous.
     * 
     * @param power Motor power (0.0 to 1.0)
     */
    public void setShooterPowerDirect(double power) {
        shooter.setPower(power);
    }

    // =========================================================================
    // DIRECT CONTROL METHODS FOR AUTONOMOUS
    // These bypass the state machine for more reliable autonomous operation
    // =========================================================================

    /**
     * Directly set the kicker servo position.
     * 
     * @param position 0.0 = retracted, 1.0 = extended (ejecting)
     */
    public void setKickerPosition(double position) {
        kicker.setPosition(position);
    }

    /**
     * Kick to eject position (direct control).
     */
    public void kickerEject() {
        kicker.setPosition(KICKER_EJECT_POS);
    }

    /**
     * Retract kicker (direct control).
     */
    public void kickerRetract() {
        kicker.setPosition(KICKER_RETRACT_POS);
    }

    /**
     * Move the indexer by a number of slots (direct control).
     * Each slot is 96 ticks = 60 degrees.
     * 
     * @param slots Number of slots to move forward (positive = forward)
     */
    public void moveIndexerSlots(int slots) {
        int ticksToMove = slots * TICKS_PER_SLOT;
        int newTarget = indexer.getCurrentPosition() + ticksToMove;
        indexer.setTargetPosition(newTarget);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setPower(0.5);
    }

    /**
     * Move the indexer by one slot forward for intake positioning.
     */
    public void indexerNextSlot() {
        moveIndexerSlots(1);
    }

    /**
     * Check if the indexer has reached its target position.
     * 
     * @return true if at target (not busy)
     */
    public boolean isIndexerAtTarget() {
        return !indexer.isBusy() &&
                Math.abs(indexer.getCurrentPosition() - indexer.getTargetPosition()) < 10;
    }

    /**
     * Get the indexer's current position in ticks.
     */
    public int getIndexerPosition() {
        return indexer.getCurrentPosition();
    }

    /**
     * Get the indexer's target position in ticks.
     */
    public int getIndexerTarget() {
        return indexer.getTargetPosition();
    }

    /**
     * Reset the state machine to LOADING state (for after autonomous use).
     */
    public void resetState() {
        currentState = RevolverState.LOADING;
        shootGreenRequest = false;
        shootPurpleRequest = false;
        manualTrigger = false;
        aligningStarted = false;
    }
}
