package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SimpleRevolver {

    // Hardware
    private DcMotorEx indexer;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private Servo kicker;

    // Constants
    // NOTE: If your motor has a gearbox, multiply these by the ratio!
    // Example: 2:1 gear ratio -> set GEAR_RATIO = 2.0
    private static final double GEAR_RATIO = 1.0; 
    private static final int TICKS_PER_REV = (int)(288 * GEAR_RATIO); 
    private static final int TICKS_PER_SLOT = TICKS_PER_REV / 3; // 96
    private static final int TICKS_TO_SHOOTER = (TICKS_PER_SLOT * 2); // 192 (2 slots away)

    private static final double INDEXER_POWER = 0.5;
    private static final double KICKER_EJECT = 0.8; // Adjust as needed
    private static final double KICKER_RETRACT = 0.3; 

    // State Variables
    private int targetTicks = 0;
    private long kickTimer = 0;
    private boolean isKicking = false;

    public SimpleRevolver(HardwareMap hardwareMap) {
        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");

        // Indexer Setup
        indexer.setDirection(DcMotorSimple.Direction.REVERSE);
        indexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        indexer.setTargetPosition(0);
        indexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexer.setVelocityPIDFCoefficients(50, 2, 1, 12); // Stiff holding
        
        // Intake Setup
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Shooter Setup
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        kicker.setPosition(KICKER_RETRACT);
    }

    public void update() {
        // Enforce Indexer Position
        if (indexer.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            indexer.setTargetPosition(targetTicks);
            indexer.setPower(INDEXER_POWER);
        }

        // Kicker Logic
        if (isKicking) {
            long elapsed = System.currentTimeMillis() - kickTimer;
            
            if (elapsed < 500) {
                // Spool up time (Wait for Shooter)
                kicker.setPosition(KICKER_RETRACT);
            } else if (elapsed < 1100) { // 500 + 600 (Eject Duration)
                kicker.setPosition(KICKER_EJECT);
            } else if (elapsed < 1700) { // 1100 + 600 (Retract Safety)
                kicker.setPosition(KICKER_RETRACT);
            } else {
                isKicking = false; 
                // Auto-stop shooter after kick? Optional.
                // setShooterPower(0); 
            }
        }
    }

    // --- Actions ---

    /**
     * Move ONE slot forward (Load Logic)
     */
    public void moveNextSlot() {
        if (isKicking) return; // Safety: Block rotation while kicker is active
        targetTicks += TICKS_PER_SLOT;
    }

    /**
     * Move TWO slots forward (Shoot Logic)
     */
    public void moveToShooter() {
        if (isKicking) return; // Safety: Block rotation while kicker is active
        targetTicks += TICKS_TO_SHOOTER;
    }

    /**
     * Start the kick sequence. 
     * You should usually call this AFTER arriving at the shooter position.
     */
    public void kick() {
        isKicking = true;
        kickTimer = System.currentTimeMillis();
    }
    
    /**
     * Manual Trim adjustment. Useful if the tick counts are slightly off.
     */
    public void manualAdjust(int deltaTicks) {
        targetTicks += deltaTicks;
    }

    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setShooterPower(double power) {
        shooter.setPower(power);
    }
    
    // --- Telemetry ---
    
    public int getCurrentPos() { return indexer.getCurrentPosition(); }
    public int getTargetPos() { return targetTicks; }
    public boolean isBusy() { return indexer.isBusy(); }
}
