# FTC 2025 Season - Decode Robotics

**Team:** Decode Robotics  
**Season:** INTO THE DEEP (2024-2025)  
**Framework:** Road Runner 1.0 + FTC SDK

---

## 📋 Table of Contents

- [Overview](#overview)
- [Autonomous Modes](#autonomous-modes)
- [TeleOp Modes](#teleop-modes)
- [Key Subsystems](#key-subsystems)
- [Technical Innovations](#technical-innovations)
- [Project Structure](#project-structure)
- [Setup & Installation](#setup--installation)

---

## 🎯 Overview

This repository contains the complete software stack for our FTC robot, featuring advanced autonomous navigation, AprilTag-based localization, smart ball collection, and precision shooting mechanisms. Our codebase integrates **Road Runner 1.0** for trajectory planning with custom subsystems for game-specific tasks.

### Key Features

- ✅ **Dual Alliance Support** - Mirrored autonomous for Blue and Red alliances
- ✅ **Smart Intake System** - Pause-and-collect ball detection with color sensors
- ✅ **AprilTag Navigation** - Real-time randomization pattern detection
- ✅ **Field-Centric Drive** - IMU-based orientation control for TeleOp
- ✅ **Closed-Loop Control** - Heading and strafe correction during autonomous
- ✅ **Modular Architecture** - Reusable subsystems and actions

---

## 🤖 Autonomous Modes

### 1. **Decode Shooting Auto v4.0** (Blue Alliance)
**File:** [`DecodeShootingAuto.java`](TeamCode/src/main/java/DecodeAuto/DecodeShootingAuto.java)

**Competition-ready autonomous for the Blue alliance side.**

#### Sequence:
1. **AprilTag Detection** - Moves to observation position and detects randomization pattern (Green/Purple/Purple)
2. **Shoot Preloaded** - Fires 3 pre-loaded artifacts into the high basket
3. **Smart Intake** - Navigates to neutral zone and collects up to 3 balls using:
   - Color sensor detection (GREEN/PURPLE)
   - Pause-on-detection for indexing
   - Closed-loop heading correction (Kp = 2.0)
   - Strafe drift compensation
4. **Return & Shoot** - Returns to shooting position and scores collected balls
5. **Park** - Moves to observation zone

#### Key Coordinates (Blue):
| Waypoint | Position (X, Y, θ) |
|----------|-------------------|
| Start | (-56, -47, 55°) |
| AprilTag | (-43, -31, 160°) |
| Shooting | (-41, -33, 240°) |
| Intake | (-13, -5, 280°) |
| Park | (-5, -30, 90°) |

#### Technical Highlights:
- **Direct motor control** bypasses FSM for reliable timing
- **SmartIntakeAction** state machine with 4 states:
  - `MOVING_FORWARD` - Scans for balls while moving
  - `PAUSED_FOR_INDEX` - Stops to index detected ball
  - `RETURNING` - Returns to start position
  - `DONE` - Completes action
- **Configurable timings** via FTC Dashboard (@Config)

---

### 2. **Decode Shooting Auto RED v4.0** (Red Alliance)
**File:** [`DecodeShootingAutoRed.java`](TeamCode/src/main/java/DecodeAuto/DecodeShootingAutoRed.java)

**Mirrored version of Blue autonomous for Red alliance.**

#### Transformation Applied:
- **Y-axis inverted** (Y → -Y)
- **Headings adjusted** (+100° to all angles, normalized)
- **Strafe correction inverted** for 90° heading orientation

#### Key Coordinates (Red):
| Waypoint | Position (X, Y, θ) |
|----------|-------------------|
| Start | (-56, +47, -55°) |
| AprilTag | (-43, +31, 215°) |
| Shooting | (-41, +33, 135°) |
| Intake | (-13, +5, 90°) |
| Park | (-5, +30, 270°) |

#### Critical Fix:
At 90° heading, the relationship between global X drift and robot strafe is **inverted** compared to 280°:
```java
// Red side (90° heading): NEGATED strafe correction
double strafeCorrection = -xError * strafeKp;
```

---

### 3. **Drivetrain Test Auto v1.0**
**File:** [`DrivetrainTestAuto.java`](TeamCode/src/main/java/DecodeAuto/DrivetrainTestAuto.java)

**Pure trajectory testing without mechanisms.**

Tests Road Runner path following through all waypoints with 1-second pauses for verification. Used for:
- Tuning PID constants
- Verifying localization accuracy
- Calibrating heading angles

---

### 4. **Decode Auto Mode** (Legacy FSM)
**File:** [`DecodeAutoMode.java`](TeamCode/src/main/java/DecodeAuto/DecodeAutoMode.java)

**Original FSM-based autonomous with AprilTag navigation.**

Features state-based control flow with AprilTag-guided shooting alignment. Superseded by v4.0 for competition reliability.

---

## 🎮 TeleOp Modes

### 1. **A1 - Manual Control**
**File:** [`A1.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/A1.java)

**Field-centric drive with SimpleRevolver subsystem.**

#### Controls:
| Button | Function |
|--------|----------|
| **Left Stick** | Strafe (X/Y) |
| **Right Stick X** | Rotate |
| **Right Trigger** | Intake forward |
| **Left Trigger** | Intake reverse |
| **Circle (○)** | Toggle shooter on/off |
| **Left Bumper** | Kick ball |
| **Cross (✕)** | Index next slot |
| **D-pad Up/Down** | Manual indexer adjust (±5 ticks) |
| **Right Bumper** | Slow drive mode (0.3x) |
| **Options** | Reset IMU heading |

#### Features:
- Field-centric drive using IMU
- Toggle-based shooter control
- Manual indexer positioning
- Configurable power settings via constants

---

### 2. **A2 - AprilTag Assisted**
**File:** [`A2.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/A2.java)

**Adds AprilTag detection for shooting alignment.**

Same controls as A1, plus:
- **Share Button** - Activate AprilTag alignment mode
- Real-time tag detection display
- Shooting goal ID tracking (Red: 24, Blue: 20)

---

### 3. **A3 - Smart Auto-Indexing** ⭐
**File:** [`A3.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/A3.java)

**Competition TeleOp with automatic ball detection and indexing.**

#### Auto-Indexing System:
When **Right Trigger** (intake) is held:
1. Color sensor continuously monitors for balls
2. **Automatic detection** of GREEN or PURPLE balls
3. **Auto-indexes** to next slot when ball detected
4. Brief delay prevents double-detection
5. Continues until 6 slots filled

#### Manual Override:
- **Cross (✕)** - Force index next slot
- **D-pad Up/Down** - Manual indexer adjust

#### Advantages:
- Faster ball collection during TeleOp
- Reduces driver workload
- Consistent indexing timing
- Uses same `RevolverSubsystem` as autonomous

---

### 4. **SimpleTeleOp**
**File:** [`SimpleTeleOp.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/SimpleTeleOp.java)

**Basic robot-centric drive for testing.**

Minimal implementation for drivetrain verification without subsystems.

---

### 5. **DecodeTeleOp**
**File:** [`DecodeTeleOp.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/DecodeTeleOp.java)

**Advanced TeleOp with full RevolverSubsystem integration.**

Includes color sensor telemetry and FSM state monitoring for debugging.

---

## 🔧 Key Subsystems

### RevolverSubsystem
**File:** [`RevolverSubsystem.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/RevolverSubsystem.java)

**Integrated ball handling system with 6-slot revolver indexer.**

#### Components:
- **Indexer Motor** - DC motor with encoder (96 ticks/slot = 60°)
- **Shooter Motor** - Flywheel for launching balls
- **Intake Motor** - Continuous intake roller
- **Kicker Servo** - Pneumatic-style ejection (0.3 retract, 0.8 eject)
- **Color Sensor** - REV Color Sensor V3 for ball detection

#### Color Detection Algorithm:
```java
// GREEN: G > R AND G > B (green dominates)
if (g > r && g > b) return GREEN;

// PURPLE: R significantly lower than G and B
if (r < g * 0.7 && r < b * 0.7) return PURPLE;

// Detection distance: 3.5 cm
```

#### Key Methods:
- `indexerNextSlot()` - Rotate to next slot (96 ticks)
- `setShooterPowerDirect(power)` - Direct PWM control
- `kickerEject()` / `kickerRetract()` - Servo control
- `readColorNow()` - Live color sensor reading
- `isIndexerAtTarget()` - Check indexer position

---

### AprilTagNavigator
**File:** [`AprilTagNavigator.java`](TeamCode/src/main/java/DecodeAuto/AprilTagNavigator.java)

**AprilTag detection and randomization pattern recognition.**

#### Tag Configuration:
| Tag ID | Purpose |
|--------|---------|
| 21, 22, 23 | Randomization pattern (Green/Purple/Purple) |
| 20 | Blue alliance shooting goal |
| 24 | Red alliance shooting goal |

#### Pattern Detection:
```java
// Detects which of the 3 randomization tags is visible
// Returns: GREEN, PURPLE_LEFT, PURPLE_RIGHT, or UNKNOWN
RandomizationPattern detectRandomizationPattern()
```

---

### MecanumDrive (Road Runner 1.0)
**File:** [`MecanumDrive.java`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/MecanumDrive.java)

**Road Runner integration for trajectory planning.**

#### Features:
- Dead wheel odometry localization
- Spline trajectory generation
- Action-based command system
- Real-time pose estimation

---

## 💡 Technical Innovations

### 1. Smart Intake Action
**Pause-and-collect ball detection system**

Traditional approach: Run intake continuously while moving → balls may jam or miss indexer

**Our solution:**
1. Robot moves forward while scanning with color sensor
2. **Pauses movement** when ball detected
3. Indexes ball to empty slot (1.2s wait)
4. **Resumes movement** to collect next ball
5. Returns to start after collecting target count

**Benefits:**
- ✅ Reliable ball collection (no jams)
- ✅ Precise indexer timing
- ✅ Closed-loop position control during movement

---

### 2. Closed-Loop Heading & Strafe Correction

**Problem:** Robot drifts during autonomous movement due to:
- Wheel slippage
- Uneven field surface
- Motor power variations

**Solution:** Proportional control for heading and lateral drift

```java
// Heading correction
double headingError = targetHeading - currentHeading;
double turnPower = headingError * Kp;  // Kp = 2.0

// Strafe drift correction
double xError = targetX - currentX;
double strafeCorrection = xError * strafeKp;  // strafeKp = 0.15

// Apply corrections
drive.setDrivePowers(new PoseVelocity2d(
    new Vector2d(forwardSpeed, strafeCorrection), 
    turnPower
));
```

**Results:**
- Maintains straight line within ±2 inches
- Heading accuracy within ±3 degrees
- Compensates for field imperfections

---

### 3. Coordinate System Mirroring for Red Alliance

**Challenge:** Road Runner uses field-centric coordinates, but Red alliance is mirrored

**Transformation:**
```
Y_red = -Y_blue
Heading_red = Heading_blue + 100°  (normalized to [-180°, 180°])
```

**Critical insight:** Strafe correction sign must be inverted for different headings:
- At 280° (Blue): `strafeCorrection = +xError * Kp`
- At 90° (Red): `strafeCorrection = -xError * Kp`

This is because the relationship between global X drift and robot strafe direction reverses at perpendicular orientations.

---

### 4. Direct Motor Control for Autonomous Reliability

**Issue:** RevolverSubsystem FSM designed for TeleOp has non-deterministic timing

**Solution:** Bypass FSM in autonomous using direct control methods:
```java
revolver.setShooterPowerDirect(0.7);  // Direct PWM
revolver.kickerEject();               // Direct servo
revolver.indexerNextSlot();           // Direct encoder target
```

**Timing constants:**
- Shooter spin-up: 1000ms
- Kicker extend: 600ms
- Kicker retract: 400ms
- Indexer settle: 500ms

---

## 📁 Project Structure

```
TeamCode/src/main/java/
├── DecodeAuto/                          # Autonomous OpModes
│   ├── DecodeShootingAuto.java          # Blue alliance autonomous
│   ├── DecodeShootingAutoRed.java       # Red alliance autonomous
│   ├── DrivetrainTestAuto.java          # Trajectory testing
│   ├── DecodeAutoMode.java              # Legacy FSM autonomous
│   ├── AprilTagNavigator.java           # AprilTag detection
│   ├── TagConfiguration.java            # Tag ID mappings
│   ├── IntakeDebugOpMode.java           # Color sensor debugging
│   └── AutoState.java                   # FSM state enum
│
├── org/firstinspires/ftc/teamcode/
│   ├── opmodes/                         # TeleOp OpModes
│   │   ├── A1.java                      # Manual control
│   │   ├── A2.java                      # AprilTag assisted
│   │   ├── A3.java                      # Smart auto-indexing ⭐
│   │   ├── DecodeTeleOp.java            # Advanced TeleOp
│   │   └── SimpleTeleOp.java            # Basic testing
│   │
│   ├── subsystems/                      # Robot subsystems
│   │   ├── RevolverSubsystem.java       # Ball handling system
│   │   └── SimpleRevolver.java          # Simplified version
│   │
│   └── MecanumDrive.java                # Road Runner integration
│
└── apriltags/                           # AprilTag utilities
    └── RobotAutoDriveToAprilTagOmni.java
```

---

## 🚀 Setup & Installation

### Prerequisites
- **FTC SDK** 9.0+
- **Road Runner** 1.0
- **Android Studio** Hedgehog or later
- **REV Hardware Client** for configuration

### Hardware Configuration

#### Motors:
- `leftFront`, `leftBack`, `rightFront`, `rightBack` - Drivetrain
- `indexerMotor` - Revolver indexer (with encoder)
- `shooterMotor` - Flywheel
- `intakeMotor` - Intake roller

#### Servos:
- `kickerServo` - Ball ejection (0.3 = retract, 0.8 = eject)

#### Sensors:
- `imu` - REV Control Hub IMU (field-centric drive)
- `colorSensor` - REV Color Sensor V3 (ball detection)
- `webCam1` - Webcam for AprilTag detection

### Build Instructions

1. **Clone repository:**
   ```bash
   git clone <repository-url>
   cd road-runner-quickstart-master
   ```

2. **Open in Android Studio:**
   - File → Open → Select project directory
   - Wait for Gradle sync

3. **Configure hardware:**
   - Connect to Robot Controller via WiFi
   - Open REV Hardware Client
   - Match hardware names to configuration above

4. **Build and deploy:**
   ```bash
   ./gradlew build
   ./gradlew installDebug
   ```

5. **Select OpMode:**
   - Driver Station → Select OpMode
   - Choose from Autonomous or TeleOp groups

---

## 📊 Tuning Parameters

### Road Runner Tuning
Refer to Road Runner quickstart guide for:
- `DriveConstants.java` - Motor specs, wheel diameter, track width
- `StraightTest`, `TurnTest` - Calibration OpModes
- PID coefficients for trajectory following

### Autonomous Tuning (via FTC Dashboard)
Access at `http://192.168.43.1:8080/dash`

**DecodeShootingAuto:**
- `START_X`, `START_Y`, `START_HEADING_DEG` - Initial position
- `SHOOTER_POWER` - Flywheel speed (0.0-1.0)
- `INTAKE_POWER` - Intake motor power
- `SHOOTER_SPINUP_MS` - Flywheel spin-up time
- `KICKER_EXTEND_MS` / `KICKER_RETRACT_MS` - Kicker timing
- `INDEXER_SETTLE_MS` - Indexer movement time
- `INTAKE_PAUSE_MS` - Wait time for ball indexing

**SmartIntakeAction:**
- `startY`, `endY` - Intake path bounds
- `moveSpeed` - Forward movement speed (0.15 default)
- `targetHeading` - Desired heading during intake
- `headingKp` - Heading correction gain (2.0 default)
- `strafeKp` - Drift correction gain (0.15 default)

---

## 🏆 Competition Readiness

### Pre-Match Checklist
- [ ] Verify hardware configuration matches code
- [ ] Test color sensor detection (green/purple balls)
- [ ] Calibrate IMU heading (press Options in TeleOp)
- [ ] Verify AprilTag camera stream active
- [ ] Test shooter power and indexer timing
- [ ] Confirm alliance color (Blue/Red autonomous)
- [ ] Check battery voltage (>12.5V recommended)

### Recommended OpModes
- **Autonomous:** `DecodeShootingAuto` (Blue) or `DecodeShootingAutoRed` (Red)
- **TeleOp:** `A3 - Smart Indexing` for competition
- **Testing:** `DrivetrainTestAuto` for trajectory verification

---

## 📝 Documentation

Additional documentation available in `/docs`:
- `DecodeShootingAuto_Documentation_Jan23.tex` - Detailed autonomous flowcharts and FSM diagrams

---

## 👥 Team

**Decode Robotics** - FTC Team  
Season: 2024-2025 INTO THE DEEP

---

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

## 🙏 Acknowledgments

- **FIRST Tech Challenge** for the game design
- **Road Runner** team for trajectory planning framework
- **FTC Community** for shared knowledge and resources

---

**Last Updated:** January 2026  
**Version:** 4.0 (Competition Ready)
