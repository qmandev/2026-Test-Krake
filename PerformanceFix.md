# PerformanceFix.md — Execution Plan for Competitive Improvements

**Target branch:** `dcmp-beta` (create a working branch off it, e.g. `perf-fixes`)
**Context docs:** `CLAUDE.md` (build commands, conventions), `FRC3566Design.md` §7 (the analysis Fixes 1–9 come from) and §11 (2026 rules deep-dive behind Fix 10)
**Plan written:** 2026-07-07 against commit `84b56a0`. Line numbers below are approximate —
**always locate code by searching for the quoted snippet, never by line number alone.**

## How to use this file

- Work **one fix at a time, in order** (later fixes depend on earlier ones; dependencies are called out).
- After every fix: run `./gradlew build` — it must compile with zero errors before moving on.
- Fixes marked **[HW-TEST]** change robot behavior and must be validated on the real robot
  before competition. Fixes marked **[SIM-OK]** can be validated with `./gradlew simulateJava`.
- **Do not** delete commented-out code unless a step says to — the team uses it as reference.
- Checkboxes track progress. Update them as you complete steps.

## Global safety rules (read before touching anything)

1. **Units convention in this codebase**: shooter/feeder "power" methods take **rotations per
   second (RPS)** for closed-loop velocity; intake roller methods named "rps" actually set
   **duty cycle** (`rps/100`). Do not "fix" call-site values when changing method internals —
   keep commanded behavior identical unless a step says otherwise.
2. **Phoenix 6 config trap**: `motor.getConfigurator().apply(fullTalonFXConfiguration)`
   overwrites **every** config group on the device — any field you didn't set in code resets
   to factory default, wiping Tuner-X-flashed tuning (inversion, limits, gains). Never apply
   a full `TalonFXConfiguration` unless the code object is a *complete* description of the
   intended device state. Applying a single group object (e.g. `Slot0Configs`) only
   overwrites that group.
3. **WPILib composition trap**: two commands inside the same `ParallelCommandGroup` /
   `Commands.race(...)` / `ParallelDeadlineGroup` **may not require the same subsystem** —
   it throws `IllegalArgumentException` at construction (robot code crashes on boot).
   This is why Fix 4 requires the subsystem split before adding `addRequirements`.
4. WPILib commands can only be **composed or scheduled once**. Never reuse a `Command`
   instance in two compositions. When in doubt, wrap in a factory/`ProxyCommand` (see
   `RobotContainer.getAutonomousCommand()` for the existing pattern and its comment).

---

## Fix 1 — Apply motor configs in code [HW-TEST]

**Problem:** `Shooter.java` and `Intake.java` build `TalonFXConfiguration` objects but every
`apply(...)` call is commented out. All real config (PID, current limits, neutral mode,
CANcoder fusing, soft limits) lives only in motor-controller flash. A motor swap or factory
reset at an event yields an unconfigured motor.

**Files:** `src/main/java/frc/robot/subsystems/Shooter.java`,
`src/main/java/frc/robot/subsystems/Intake.java`

### Step 1.1 — Snapshot the flash configs (BLOCKING; requires robot access)

- [ ] Connect to the robot with **Phoenix Tuner X**. For each device — TalonFX 50, 51, 52,
      53 (shooter/indexer), 40 (roller), 42 (arm), CANcoder 41 — open the config page and
      export/record **all** config values (gains for every slot, current limits, inversion,
      neutral mode, feedback settings, Motion Magic, soft limits, ramps).
- [ ] Save the snapshot into a new file `docs/motor-flash-snapshot-<date>.md` in this repo
      (create the `docs/` directory). This is the ground truth for Step 1.2.

> ⚠️ If you cannot access the robot, **stop Fix 1 after writing the code but leave the
> `apply()` calls behind a disabled flag** (see Step 1.4) and flag this in your report.
> Do not enable code-side config with unverified values.

### Step 1.2 — Reconcile code constants with flash

- [ ] For every field where flash differs from the values currently written in
      `Shooter.java` / `Intake.java` (e.g. `kP = 0.55`, `kV = 0.12`, stator 160 A /
      supply 80 A), **update the code to match flash** — flash is the competition-tuned
      truth. Record each difference in the snapshot doc.
- [ ] Fields present in flash but absent from the code config objects (e.g. motor
      **inversion** on shooter/indexer/roller, any ramps) must be **added** to the code
      config objects. The code object must become a complete device description
      (see Global rule 2).
- [ ] Two deliberate deviations from flash (make these, and only these, on purpose):
      - Flywheel + indexer `MotorOutput.NeutralMode` = **`NeutralModeValue.Coast`**
        (currently `Brake` in the dead code — braking a flywheel dumps energy and stresses
        the belt). Keep the **arm** in `Brake`.
      - Arm soft limits **enabled**: in `Intake.java`, change
        `armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;` → `true;` and
        `ReverseSoftLimitEnable = false;` → `true;` (thresholds already come from
        `Constants.Arm`). Verify on-robot that the arm's full legal travel stays inside
        [−0.05, 0.45] rotations before trusting this in a match.

### Step 1.3 — Write the apply helper

Add a small retry helper (Phoenix config calls can fail transiently on a busy CAN bus).
Put it in a new file `src/main/java/frc/robot/util/PhoenixUtil.java`:

```java
package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;

public final class PhoenixUtil {
    private PhoenixUtil() {}

    /** Runs a Phoenix config call up to 5 times until it returns OK. Reports failure to DS. */
    public static void tryUntilOk(Supplier<StatusCode> configCall, String deviceName) {
        for (int i = 0; i < 5; i++) {
            if (configCall.get().isOK()) return;
        }
        DriverStation.reportError("Config apply FAILED for " + deviceName, false);
    }
}
```

### Step 1.4 — Enable the applies

- [ ] In `Shooter.java`, replace the commented block

  ```java
  // ShooterLeft.getConfigurator().apply(upperMotorConfig, 0.05);
  // ShooterRight.getConfigurator().apply(upperMotorConfig, 0.05);
  // IndexerLeft.getConfigurator().apply(lowerMotorConfig, 0.05);
  // IndexerRight.getConfigurator().apply(lowerMotorConfig, 0.05);
  ```

  with

  ```java
  PhoenixUtil.tryUntilOk(() -> ShooterLeft.getConfigurator().apply(upperMotorConfig), "ShooterLeft");
  PhoenixUtil.tryUntilOk(() -> ShooterRight.getConfigurator().apply(upperMotorConfig), "ShooterRight");
  PhoenixUtil.tryUntilOk(() -> IndexerLeft.getConfigurator().apply(lowerMotorConfig), "IndexerLeft");
  PhoenixUtil.tryUntilOk(() -> IndexerRight.getConfigurator().apply(lowerMotorConfig), "IndexerRight");
  ```

  Note: if left and right motors have **different inversion** in flash, you need per-motor
  config objects (clone the config, set `MotorOutput.Inverted` per side). Check the
  snapshot. The `StrictFollower` calls that follow must stay **after** the config applies.
- [ ] In `Intake.java`: enable `armLeaderMotor.getConfigurator().apply(armConfig)` (wrapped
      in `tryUntilOk`). For the roller, the existing `rollerConfig` is only a `Slot0Configs`;
      build a full `TalonFXConfiguration` for the roller from the snapshot (inversion,
      current limits, neutral mode, Slot0) and apply that instead.
- [ ] Add a supply-current limit to the **roller** and **arm** configs if flash doesn't have
      one (suggested: 40 A supply, matching the brownout mitigation intent of commit
      `cc85063`). Record what you chose.
- [ ] Resolve the stale comment `//TODO: DO NOT USE VALUES, USE FILE CONFIG INSTEAD` —
      delete it and replace with a comment pointing to the snapshot doc.

### Step 1.5 — Verify

- [ ] `./gradlew build` passes.
- [ ] **[HW-TEST]** Deploy. Confirm in Tuner X that device configs now match the snapshot
      (plus the two deliberate deviations). Drive, intake, and shoot; behavior must be
      unchanged except the flywheel now coasts down slowly when stopped.
- [ ] Check the Driver Station log for any "Config apply FAILED" messages — there must be none.

---

## Fix 2 — Real flywheel velocity + at-speed feed gating [HW-TEST]

**Problem:** `Shooter.getUpperVelocity()` returns `ShooterLeft.get() * 100` — duty cycle,
not velocity. The teleop feed interlock (`RobotContainer`, first driver's left trigger) and
therefore shot quality depend on it. Separately, `PrimeAndShoot`/`PrimeAndShootFixed` feed
on a timer, not on the flywheel actually being at speed.

**Files:** `Shooter.java`, `RobotContainer.java`,
`commands/shooter/PrimeAndShoot.java`, `commands/shooter/PrimeAndShootFixed.java`

### Step 2.1 — Track the commanded setpoint in `Shooter`

- [ ] Add a field: `private double upperSetpointRps = 0;`
- [ ] In `setUpperPower(double rps)`: add `upperSetpointRps = rps;` before the `setControl`.
- [ ] In `stopUpper()`: add `upperSetpointRps = 0;`.

### Step 2.2 — Fix the velocity getter and add the gate

- [ ] Replace the body of `getUpperVelocity()`:

  ```java
  /** Measured flywheel velocity in rotations per second (leader motor). */
  public double getUpperVelocity(){
      return ShooterRight.getVelocity().getValueAsDouble();
  }

  /** Last commanded flywheel setpoint in RPS (0 when stopped). */
  public double getUpperSetpoint() {
      return upperSetpointRps;
  }

  /**
   * True when a nonzero setpoint is commanded and measured velocity is within
   * tolerance of it. Tolerance: 5% of setpoint, floor of 2 RPS.
   */
  public boolean isUpperAtSpeed() {
      if (upperSetpointRps <= 0) return false;
      double tol = Math.max(2.0, 0.05 * upperSetpointRps);
      return Math.abs(getUpperVelocity() - upperSetpointRps) <= tol;
  }
  ```

  ⚠️ Search the codebase for other callers of `getUpperVelocity()` (`grep -rn
  getUpperVelocity src/`) — as of `84b56a0` the only caller is the LT binding in
  `RobotContainer`; if more appeared since, re-check each one's threshold semantics.
- [ ] Add to `periodic()`:
      `SmartDashboard.putBoolean("Shooter/At Speed", isUpperAtSpeed());`
      `SmartDashboard.putNumber("Shooter/Setpoint (rps)", upperSetpointRps);`

### Step 2.3 — Update the teleop feed interlock

In `RobotContainer.configureBindings()`, find the `firstDriver.leftTrigger().whileTrue(` block
containing `shooter.getUpperVelocity() > 10` and replace the condition with
`shooter.isUpperAtSpeed()`. Keep the runEnd structure and `stopLower()` calls as-is.

### Step 2.4 — Gate feeding in PrimeAndShoot / PrimeAndShootFixed

In **both** commands, replace the `execute()` feed logic. Current form:

```java
if (timer.get() > primeTime) {
    shooter.setLowerPower(80);   // 90 in PrimeAndShootFixed
}
```

New form (identical in both, keep each file's RPS number):

```java
// Feed when the flywheel is truly at speed. Fallback: if the at-speed check
// never passes (e.g. bad tolerance), start feeding anyway after 2x primeTime
// so autos can't dead-stall.
boolean atSpeed = shooter.isUpperAtSpeed();
boolean fallback = timer.get() > (2.0 * primeTime);
if ((atSpeed || fallback) && timer.get() > primeTime * 0.5) {
    shooter.setLowerPower(80);
} else {
    shooter.stopLower();   // pause feeding while flywheel recovers between balls
}
```

Note this also *pauses* the feeder during between-ball velocity dips — that is intentional
and is the main accuracy win.

### Step 2.5 — Verify

- [ ] `./gradlew build` passes.
- [ ] **[HW-TEST]** On robot: command X (50 RPS) on driver 2, watch `Shooter/At Speed` go
      true only after spin-up; hold LT — feeder must not start until at-speed. Shoot a
      3+ ball volley and confirm the feeder visibly pauses between balls and grouping
      tightens. If the feeder chatters (rapid start/stop), widen the tolerance floor from
      2.0 → 3.0 RPS or add a small debounce.

---

## Fix 3 — Make `AutoPrime` actually prime [SIM-OK to compile; HW-TEST for timing]

**Problem:** `commands/shooter/AutoPrime.java` is registered as the "prime while driving"
NamedCommand but only runs the intake roller; the flywheel spin-up is commented out. Autos
pay full spin-up at every shooting stop.

**Files:** `AutoPrime.java`, `RobotContainer.java`

### Steps

- [ ] Add a speed field and constructor to `AutoPrime`:

  ```java
  private final double speedRps;

  public AutoPrime(Shooter shooter, Intake intake, double speedRps) {
      this.shooter = shooter;
      this.intake = intake;
      this.speedRps = speedRps;
  }
  ```

  Keep the existing 2-arg constructor delegating with `speedRps = 0`
  (`this(shooter, intake, 0);`). ⚠️ **Constructor-overload hazard:** there is an existing
  `AutoPrime(Shooter, Intake, int timeOut)` constructor. An `int` literal third argument
  will bind to it, a `double` will bind to the new one. The `timeOut` field is never used —
  **delete the `int timeOut` constructor and the `timeOut` field** to remove the ambiguity
  (verify no callers first: `grep -rn "new AutoPrime" src/`).
- [ ] In `initialize()`, under `intake.rollerIn(70);`, add:

  ```java
  if (speedRps > 0) {
      shooter.setUpperPower(speedRps);
  }
  ```

- [ ] Deliberately do **not** stop the flywheel in `end()` — AutoPrime is raced against a
      path; when the following `PrimeAndShoot*` starts it takes over an already-spinning
      wheel. Add a comment saying exactly that.
- [ ] In `RobotContainer.configureAutoCommand()`, change the registration:

  ```java
  NamedCommands.registerCommand(
      "AutoPrime",
      new AutoPrime(shooter, intake, autonomousPower)
  );
  ```

  (`autonomousPower` is a `double` field = 55, so it binds to the new constructor.)
- [ ] While in that method: **delete the duplicate second registration of
      `"SwipePrimeAndShoot"`** (the identical block registered twice back-to-back).
- [ ] Verify: `./gradlew build`; **[HW-TEST]** run one shooting auto and confirm the wheel
      is already spinning on arrival and total time-to-first-shot drops.

---

## Fix 4 — Subsystem split (Arm/Roller) + correct `addRequirements` [SIM-OK + HW sanity]

**Problem:** `PrimeAndShoot` and `ArmUpAndDown` declare no requirements, so the scheduler
cannot arbitrate (the universal cancel can't interrupt `PrimeAndShoot`; concurrent commands
can fight over motors). Requirements can't simply be added because
`TrigPrimeAndShoot` runs `Commands.race(PrimeAndShoot(...uses roller...),
...ArmUpAndDown(...uses arm...))` — with a shared `Intake` requirement that composition
**throws at construction** (Global rule 3). The arm and roller are physically independent,
so split the subsystem.

**Files:** `Intake.java` (split), `RobotContainer.java`, `Robot.java`, and every command
under `commands/intake/` and `commands/shooter/`.

### Step 4.1 — Split `Intake` into `IntakeArm` and `IntakeRoller`

- [ ] Create `src/main/java/frc/robot/subsystems/IntakeArm.java`: move from `Intake.java`
      everything arm-related — `armLeaderMotor`, `ArmCanCoder`, `motionMagicRequest`,
      `mmUp`/`mmDown` + MM constants, `ARM_GEAR_RATIO`, the arm config block from the
      constructor (including the Fix-1 apply), and methods `setArmPosition`,
      `getArmPosition`, `armAtSetpoint`, `stopArm`, `armUp`, `armDown`, `setArmPower`,
      `setArmEncoderPosition`, plus the arm-related `periodic()` dashboard lines.
- [ ] Create `src/main/java/frc/robot/subsystems/IntakeRoller.java`: move `rollerMotor`,
      the roller config block (with Fix-1 apply), `rollerIn`, `rollerOut`, `stopRoller`,
      and the roller `periodic()` dashboard lines.
- [ ] Delete `Intake.java` once nothing references it.
- [ ] Keep all method names, dashboard keys, CAN IDs, and behavior **identical** — this is
      a mechanical move, not a redesign.

### Step 4.2 — Update every reference

Run `grep -rn "Intake" src/main/java --include=*.java -l` and fix each file:

- [ ] `RobotContainer.java`: replace `public final Intake intake = new Intake();` with
      `public final IntakeArm intakeArm = new IntakeArm();` and
      `public final IntakeRoller intakeRoller = new IntakeRoller();`. Update every binding:
      roller calls (`rollerIn/rollerOut/stopRoller`, `TrigRollerIn`) → `intakeRoller`;
      arm calls (`ArmToSetpoint`, `ArmUpAndDown`, `setArmEncoderPosition`, `setArmPower`,
      `stopArm`) → `intakeArm`; `PrimeAndShoot*`/`AutoPrime` constructor args per Step 4.3.
- [ ] `Robot.java`: `disabledInit` → `m_robotContainer.intakeArm.stopArm();`
      `autonomousExit` → `m_robotContainer.intakeArm.setArmPosition(...)`.
- [ ] `commands/intake/ArmToSetpoint.java`, `ArmUpAndDown.java`, `ArmSwitch.java`,
      `GetIntakeData.java`: change `Intake` → `IntakeArm` (`GetIntakeData` may touch both —
      read it and route each call to the right class).
- [ ] `commands/shooter/PrimeAndShoot.java`, `PrimeAndShootFixed.java`, `AutoPrime.java`:
      change the `Intake` field/parameter to `IntakeRoller` (these commands only use
      `rollerIn`/`stopRoller`).
- [ ] `commands/TestCommands.java`: update whatever it references.

### Step 4.3 — Add the requirements

- [ ] `PrimeAndShoot`: in the main constructor add `addRequirements(shooter, roller);`
- [ ] `PrimeAndShootFixed`: change existing `addRequirements(shooter, intake)` to
      `addRequirements(shooter, roller);`
- [ ] `AutoPrime`: add `addRequirements(roller);` — **not** `shooter`. Reason: `AutoPrime`
      runs in parallel/race with paths while a subsequent shoot command may overlap during
      transitions; it sets the flywheel once in `initialize()` and never needs exclusive
      ownership. (If post-change testing shows flywheel command fights, revisit.)
- [ ] `ArmUpAndDown`: add `addRequirements(intake);` (the `IntakeArm` parameter).
- [ ] `ArmToSetpoint`: already has `addRequirements(intake)` — keep (now `IntakeArm`).

### Step 4.4 — Verify the compositions still construct

- [ ] `./gradlew build`.
- [ ] `./gradlew simulateJava`: robot code must boot with **no
      `IllegalArgumentException: ...cannot require the same subsystems`** in the console.
      Specifically exercise: select each auto in the chooser and enable autonomous in the
      sim GUI — every `Commands.race(...)` in `configureAutoCommand` constructs at bind
      time, but scheduling flushes out runtime conflicts.
- [ ] Confirm the second-driver **A** (universal cancel, requires `shooter`) now interrupts
      a running `PrimeAndShoot` in sim (schedule via auto, press A → command's
      `end(interrupted=true)` print appears).

---

## Fix 5 — Vision in autonomous + real filtering [HW-TEST]

**Problem:** (a) vision pose updates are disabled during auto (pure odometry → left/right
shot-power asymmetry); (b) the ambiguity filter rebuilds the target list but keeps the pose
computed from all targets; (c) the rotation gate at 180° never rejects; (d) one "lost robot"
counter is shared by all three cameras.

**Files:** `Vision.java`, `Robot.java`

### Step 5.1 — Per-camera lost counter

- [ ] In the `Cameras` enum, add a field `public double longDistancePoseCount = 0;`
- [ ] In `filterPose(...)`, the counter logic currently uses the subsystem-level
      `longDistangePoseEstimationCount`. `filterPose` receives only the camera *name* —
      change its signature to take the `Cameras camera` enum instead of `String cameraName`
      (derive `cameraName = camera.name()` inside; update the single call site in
      `updatePoseEstimation`). Then replace every use of the shared counter with
      `camera.longDistancePoseCount`. Delete the now-unused subsystem field.

### Step 5.2 — Make the ambiguity filter reject contaminated frames

- [ ] In `filterPose`, find the block that builds `goodTargets` and then constructs
      `var filtered = new EstimatedRobotPose(pose.get().estimatedPose, ...)`.
      Change the policy from "rebuild with good targets" to "reject the frame if any used
      target is bad":

  ```java
  boolean anyBadTarget = pose.get().targetsUsed.stream()
      .anyMatch(t -> t.getPoseAmbiguity() >= 0 && t.getPoseAmbiguity() >= activeThreshold);
  if (anyBadTarget) {
      SmartDashboard.putString(cameraKey + "/Filter Reject Reason",
          "Contains target above ambiguity threshold (" + activeThreshold + ")");
      return Optional.empty();
  }
  var filtered = pose.get();  // frame is clean; use as-is
  ```

  Keep the existing dashboard logging of good/total counts above it. Delete the
  now-dead `goodTargets.isEmpty()` branch and the `new EstimatedRobotPose(...)` rebuild.
- [ ] Change `private final double maximumRotationDifferenceDeg = 180;` → `= 35;`.
      (The rotation-vs-odometry gate directly below the ambiguity code now actually works.)

### Step 5.3 — Multi-tag-only mode for autonomous

- [ ] Add to `Vision`:

  ```java
  /** When true, only multi-tag pose estimates are accepted (used during auto). */
  private boolean multiTagOnly = false;
  public void setMultiTagOnly(boolean enabled) {
      multiTagOnly = enabled;
      SmartDashboard.putBoolean("Vision/MultiTagOnly", enabled);
  }
  ```

- [ ] At the top of `filterPose`, after the empty-check, add:

  ```java
  if (multiTagOnly && pose.get().targetsUsed.size() < 2) {
      SmartDashboard.putString(cameraKey + "/Filter Reject Reason", "MultiTagOnly: single tag");
      return Optional.empty();
  }
  ```

- [ ] In `Robot.java`:
      - `autonomousInit()`: add `m_robotContainer.vision.setMultiTagOnly(true);`
      - `autonomousPeriodic()`: replace the commented-out line with a live call:
        `m_robotContainer.vision.updatePoseEstimation(m_robotContainer.drivetrain);`
      - `teleopInit()`: add `m_robotContainer.vision.setMultiTagOnly(false);`
      - `autonomousExit()`: the existing one-shot `updatePoseEstimation` call can stay.

### Step 5.4 — Verify

- [ ] `./gradlew build`; `./gradlew simulateJava` — in the sim GUI, run an auto and watch
      `Vision/<cam>/Filtered Pose Available`: single-tag frames must be rejected during
      auto (`Filter Reject Reason` = "MultiTagOnly: single tag") and accepted again in teleop.
- [ ] **[HW-TEST]** On the practice field: run Left and Right shooting autos and compare
      `AutoPower/Distance to Target (m)` at each stop against tape-measure truth. If pose
      is now accurate on both sides, switch the Right-side autos from `PrimeAndShootFixed`
      back to pose-based `PrimeAndShoot` (edit the NamedCommand registrations, one at a
      time, retesting each) — that is the payoff of this fix.
- [ ] If auto paths visibly "jump" when vision corrects mid-path, raise the translation
      stddevs (first elements of the `VecBuilder.fill` per camera) until corrections are
      gentle, or gate updates to when the robot is slow
      (`Math.hypot(speeds.vx, speeds.vy) < 1.0`).

---

## Fix 6 — One-button Aim-and-Shoot supercommand [HW-TEST]

**Problem:** a scored shot needs three inputs across two drivers (driver 1 A = aim,
driver 2 X/B/Y = power, driver 1 LT = feed). Combine into one command with interlocks.
**Depends on:** Fix 2 (`isUpperAtSpeed`), Fix 4 (`IntakeRoller` + requirements discipline).

**Files:** new `src/main/java/frc/robot/commands/shooter/AimAndShoot.java`, `RobotContainer.java`

### Step 6.1 — Create the command

One class, owning drivetrain + shooter + roller, mirroring `TurnToHub`'s heading logic and
`AutoPower`'s power logic (open both files and copy the referenced pieces exactly):

```java
package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.vision.TagUtil;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.Shooter;

/**
 * Hold-to-score: aims at the hub (driver keeps translation), spins the flywheel from
 * pose distance, and feeds automatically once heading AND flywheel are within tolerance.
 */
public class AimAndShoot extends Command {
    private static final double HEADING_TOL_DEG = 2.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final Shooter shooter;
    private final IntakeRoller roller;
    private final DoubleSupplier velocityX, velocityY;
    private final Supplier<Translation2d> targetSupplier;
    private boolean isBlueAlliance;
    private Translation2d target;

    private final SwerveRequest.FieldCentricFacingAngle drive;

    public AimAndShoot(CommandSwerveDrivetrain drivetrain, Shooter shooter, IntakeRoller roller,
                       DoubleSupplier velocityX, DoubleSupplier velocityY,
                       double maxSpeed) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.roller = roller;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.targetSupplier = TagUtil::getHubCenterTranslation;

        drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);
        drive.HeadingController.setPID(4.0, 0.0, 0.1);   // same as TurnToHub
        drive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, shooter, roller);
    }

    @Override
    public void initialize() {
        isBlueAlliance = DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        target = targetSupplier.get();
        SmartDashboard.putBoolean("AimAndShoot/Enabled", true);
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getPose();

        // --- aim (same math + red-alliance offset as TurnToHub) ---
        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        Rotation2d targetHeading = new Rotation2d(Math.atan2(dy, dx));
        if (!isBlueAlliance) targetHeading = targetHeading.plus(Rotation2d.kPi);
        drivetrain.setControl(
            drive.withVelocityX(velocityX.getAsDouble())
                 .withVelocityY(velocityY.getAsDouble())
                 .withTargetDirection(targetHeading));

        // --- power ---
        double distance = pose.getTranslation().getDistance(target);
        shooter.autoPower(distance);

        // --- feed gate ---
        double headingErrDeg = Math.abs(
            targetHeading.minus(pose.getRotation()).getDegrees());
        boolean aimed = headingErrDeg < HEADING_TOL_DEG;
        boolean ready = aimed && shooter.isUpperAtSpeed();
        SmartDashboard.putNumber("AimAndShoot/Heading Error (deg)", headingErrDeg);
        SmartDashboard.putBoolean("AimAndShoot/Ready", ready);
        if (ready) {
            shooter.setLowerPower(80);
            roller.rollerIn(70);
        } else {
            shooter.stopLower();
            roller.stopRoller();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopUpper();
        shooter.stopLower();
        roller.stopRoller();
        SmartDashboard.putBoolean("AimAndShoot/Enabled", false);
        SmartDashboard.putBoolean("AimAndShoot/Ready", false);
    }

    @Override
    public boolean isFinished() { return false; }   // hold-to-run
}
```

⚠️ Heading-error caveat: on red alliance, `targetHeading` has the +180° operator-perspective
offset while `pose.getRotation()` is field-frame — verify on-robot which frame
`getState().Pose` heading is in for red before trusting `headingErrDeg`; if the error reads
~180° when visually aimed, compute the error against the un-offset field heading instead
(keep a second local `fieldHeading` without the `plus(kPi)` for the error calculation only).
`TurnToHub` publishes the same style of error today (`TurnToHub/Heading Error (deg)`) —
compare against it on red.

### Step 6.2 — Bind it

- [ ] In `RobotContainer.configureBindings()`, replace the `firstDriver.a().whileTrue(new
      TurnToHub(...))` binding with:

  ```java
  firstDriver.a().whileTrue(new AimAndShoot(drivetrain, shooter, intakeRoller,
      () -> -firstDriver.getLeftY() * MaxSpeed,
      () -> -firstDriver.getLeftX() * MaxSpeed,
      MaxSpeed));
  ```

  Keep `TurnToHub` and the LT feed binding intact as the fallback path — do not delete them.
- [ ] Note in the commit message that driver 2's X/B fixed-power modes remain untouched as
      the manual fallback.

### Step 6.3 — Verify

- [ ] `./gradlew build`; sim boot clean (requirement conflicts would throw here — `AimAndShoot`
      requires `shooter`, and driver 2's Y (`AutoPower`) also requires `shooter`; that is
      *fine* because they are separate bindings, not one composition — last-scheduled wins).
- [ ] **[HW-TEST]**: from three field positions, hold A with balls loaded — robot must snap
      heading, spin up, and volley without any second-driver input. Confirm A on driver 2
      still cancels everything (it requires `shooter`, so it interrupts `AimAndShoot`; the
      command's `end()` then stops the drivetrain requests naturally on unbind).

---

## Fix 7 — Beam-break sensing + shoot-until-empty [BLOCKED ON HARDWARE]

**Precondition:** electrical must mount a beam-break sensor in the feeder path and give you
its DIO port number. **Until then, implement behind a feature flag** so the code merges
safely and activates by flipping one constant.

**Files:** `Constants.java`, `Shooter.java` (or the feeder-owning subsystem),
`PrimeAndShoot.java`, `PrimeAndShootFixed.java`, `AimAndShoot.java`

### Steps

- [ ] `Constants.java` — add:

  ```java
  public static class Sensors {
      /** DIO port of the feeder beam break. Set to -1 if not installed. */
      public static final int FeederBeamBreakDIO = -1;
  }
  ```

- [ ] In `Shooter` (the feeder lives there): add a `DigitalInput` created only when the
      port is `>= 0`; expose:

  ```java
  /** True when a ball blocks the feeder beam break. Returns false if no sensor. */
  public boolean feederHasBall() {
      return beamBreak != null && !beamBreak.get();   // beam breaks read LOW when blocked; verify polarity on-robot
  }
  public boolean hasBallSensor() { return beamBreak != null; }
  ```

      Publish `Shooter/Feeder Has Ball` in `periodic()`.
- [ ] Shot counting: in `periodic()`, detect falling edges of `feederHasBall()` while the
      feeder is commanded, increment `Shooter/Balls Shot` on the dashboard, and expose
      `getTimeSinceLastBallSeconds()` (a `Timer` restarted on every edge/presence).
- [ ] Shoot-until-empty: in `PrimeAndShoot`/`PrimeAndShootFixed`/`AimAndShoot` `isFinished()`
      (for the two auto commands; `AimAndShoot` stays hold-to-run), add an early-exit:

  ```java
  // Early exit once feeding has begun and no ball has been seen for 0.5 s
  if (shooter.hasBallSensor()
      && timer.get() > primeTime + 0.5
      && shooter.getTimeSinceLastBallSeconds() > 0.5) {
      return true;
  }
  ```

      The existing `primeTime + shootTime` timeout remains as the fallback — with the sensor
      installed, `shootTime` becomes a ceiling instead of the actual duration.
- [ ] Verify build + sim (flag off = behavior identical). Once hardware exists: set the DIO
      port, verify polarity, then **[HW-TEST]** a 3-ball auto — it should end early and move on.

---

## Fix 8 — Dashboard-tunable shot constants [SIM-OK]

**Problem:** shot powers are hard-coded across `RobotContainer` (55/60/65, inline 54) and
command bodies (80/90 feeder). Pit retunes require redeploys.

**Approach:** WPILib `Preferences` — persisted on the roboRIO, editable live from
Shuffleboard/Elastic's NetworkTables view under `Preferences/`, read at command start.

### Steps

- [ ] Create `src/main/java/frc/robot/util/TunableShots.java`:

  ```java
  package frc.robot.util;

  import edu.wpi.first.wpilibj.Preferences;

  /** Live-tunable shot values. Edit under NetworkTables Preferences/ — persists on the RIO. */
  public final class TunableShots {
      private TunableShots() {}

      static {
          Preferences.initDouble("Shots/AutonomousRPS", 55);
          Preferences.initDouble("Shots/TrenchRPS", 60);
          Preferences.initDouble("Shots/SwipeRPS", 65);
          Preferences.initDouble("Shots/CenterRPS", 54);
          Preferences.initDouble("Shots/TeleopScoreRPS", 50);
          Preferences.initDouble("Shots/TeleopPassRPS", 75);
          Preferences.initDouble("Shots/FeederRPS", 80);
      }

      public static double autonomous() { return Preferences.getDouble("Shots/AutonomousRPS", 55); }
      public static double trench()     { return Preferences.getDouble("Shots/TrenchRPS", 60); }
      public static double swipe()      { return Preferences.getDouble("Shots/SwipeRPS", 65); }
      public static double center()     { return Preferences.getDouble("Shots/CenterRPS", 54); }
      public static double teleopScore(){ return Preferences.getDouble("Shots/TeleopScoreRPS", 50); }
      public static double teleopPass() { return Preferences.getDouble("Shots/TeleopPassRPS", 75); }
      public static double feeder()     { return Preferences.getDouble("Shots/FeederRPS", 80); }
  }
  ```

- [ ] **Read-at-start rule:** commands must read the value in `initialize()` (or in a
      `Supplier` evaluated at schedule time), never once at construction — NamedCommands are
      constructed at boot and would freeze the value. Concretely:
      - `PrimeAndShootFixed`: change the registrations in `configureAutoCommand` to pass a
        `DoubleSupplier` (add a supplier-based constructor; resolve it in `initialize()`),
        e.g. `new PrimeAndShootFixed(shooter, intakeRoller, TunableShots::autonomous, 1, 4)`.
      - Feeder RPS in `PrimeAndShoot*`/`AimAndShoot`/the LT binding: replace literal
        `80`/`90` with `TunableShots.feeder()` read in `execute()` (cheap NT read; fine).
      - Driver 2 X/B bindings: replace `50`/`75` with `TunableShots.teleopScore()` /
        `teleopPass()` inside the existing `runOnce` lambdas (lambdas run at press time —
        already correct).
- [ ] Delete the now-unused fields `autonomousPower`, `trenchPower`, `swipePower` from
      `RobotContainer` **after** every use is migrated (`grep -n "Power" RobotContainer.java`).
- [ ] Verify: `./gradlew simulateJava` → change `Preferences/Shots/TeleopScoreRPS` in the
      sim NT viewer → press X → `Shooter/Setpoint (rps)` reflects the new value without redeploy.

---

## Fix 9 — ShotCalculator (shoot-on-the-move) integration [HW-TEST, stretch goal]

**Do not start until Fixes 1–6 are merged and validated on the robot.** This is the highest
ceiling and the highest risk; keep it on a separate branch until proven.

**Files:** `frc/firecontrol/*` (existing), new command, `Shooter.java`, `RobotContainer.java`

### Phase 9a — Build a trustworthy LUT

- [ ] Read `frc/firecontrol/GenerateLookUpTable.java` and `ProjectileSimulator.java` — the
      generator runs offline (it has a `main`-style entry). Configure it with the robot's
      real launch height/angle and wheel-speed→exit-velocity ratio, generate a
      distance→(RPM, TOF) table.
- [ ] **Empirically verify** at ≥4 distances on the practice field: command the LUT's RPM at
      a measured distance and record make/miss. Adjust the simulator's exit-velocity
      coefficient until LUT RPM matches the proven `10.818*ln(d)+38.335` curve within ~5%
      at the distances where the log-fit is known-good. Store the final table as a constant
      array in a new file `frc/firecontrol/ShooterLUT.java` (no runtime generation on the RIO).

### Phase 9b — Wire the solver

- [ ] Instantiate `ShotCalculator` in `RobotContainer` per the usage javadoc at the top of
      `ShotCalculator.java` (measure `launcherOffsetX/Y` from CAD; load every reachable LUT
      entry via `loadLUTEntry`).
- [ ] Create `AimAndShootMoving` as a **copy of `AimAndShoot`** (Fix 6) with the aim/power
      section replaced: build `ShotCalculator.ShotInputs` from
      `drivetrain.getState().Pose`, field/robot velocities (`getState().Speeds` — convert to
      field frame for the field-velocity argument), hub center from `TagUtil`, and call
      `calc.calculate(inputs)`. When `result.isValid() && result.confidence() > 50`, use
      `result.rpm()` for `setUpperPower` (convert RPM→RPS: divide by 60 — **check the LUT's
      unit convention first**, `ShotCalculator` speaks RPM, `Shooter` speaks RPS) and
      `result.driveAngle()` for `withTargetDirection`. When invalid/low-confidence, fall
      back to the stationary `AimAndShoot` behavior.
- [ ] Bind to an unused button (suggest `firstDriver.povDown()`) — do not replace the
      stationary `AimAndShoot` until drivers prefer it.

### Phase 9c — Validate

- [ ] Unit-test the pure math first (this package has no WPILib hardware deps): create
      `src/test/java/frc/firecontrol/ShotCalculatorTest.java` asserting (a) a stationary
      robot's solution matches the raw LUT entry, (b) a robot strafing left aims right of
      the hub, (c) convergence within 5 iterations across the LUT's distance range. Run
      with `./gradlew test`.
- [ ] **[HW-TEST]** Rolling shots at slow speed (~1 m/s) first, then game speed. Log
      commanded RPM/heading vs make rate via SignalLogger for tuning.

---

## Fix 10 — Hub-active tracker + driver rumble cue [SIM-OK; rumble is HW-nice-to-have]

**Independent of Fixes 1–9 — can be done at any time.** Source analysis: `FRC3566Design.md` §11.3.

**Problem:** in 2026 REBUILT, during Shifts 1–4 only one alliance's HUB is active at a
time — fuel shot into an inactive hub is **wasted**. Which hub is inactive first is decided
by who scored more fuel in AUTO, and FMS tells robot code the answer:
`DriverStation.getGameSpecificMessage()` returns `"B"` or `"R"` at teleop start (the
alliance that won AUTO; that alliance's hub is inactive for Shift 1, then it alternates
every shift; both hubs are active during the Transition shift and Endgame).
`Robot.setTeleopShifts()` already computes shift names but **not** hub state — drivers
must remember the alternation themselves.

**Files:** `Robot.java`, `RobotContainer.java`

### Steps

- [ ] In `RobotContainer`, add a rumble passthrough (the controllers are private fields —
      search for `firstDriver`):

  ```java
  /** Rumble both driver-1 grips; value 0..1, 0 stops. Legal robot→driver channel (FMS/HID). */
  public void setDriverRumble(double value) {
      firstDriver.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, value);
  }
  ```

- [ ] In `Robot.java`, add fields and a `setHubStatus()` method (call it from
      `teleopPeriodic()` right next to the existing `setTeleopShifts()` call):

  ```java
  private boolean lastHubActive = true;
  private double rumbleEndTime = 0;

  /** Publishes whether OUR hub is active this shift. GSM ("B"/"R") = alliance that won
   *  AUTO; their hub is INACTIVE for Shift 1, alternating each shift. Both hubs active
   *  in Transition and Endgame. */
  public void setHubStatus() {
      String gsm = DriverStation.getGameSpecificMessage();
      var alliance = DriverStation.getAlliance();
      double t = DriverStation.getMatchTime();

      boolean ourHubActive = true; // default: transition, endgame, or unknown
      boolean known = (gsm.equals("B") || gsm.equals("R")) && alliance.isPresent() && t >= 0;
      if (known && t <= 130 && t > 30) {
          int shift = (t > 105) ? 1 : (t > 80) ? 2 : (t > 55) ? 3 : 4;
          boolean weWonAuto =
              (alliance.get() == DriverStation.Alliance.Blue) == gsm.equals("B");
          // auto winner sits out odd shifts (1,3); auto loser sits out even shifts (2,4)
          ourHubActive = !(weWonAuto == (shift % 2 == 1));
      }

      SmartDashboard.putBoolean("Teleop/Our Hub Active", ourHubActive);
      SmartDashboard.putString("Teleop/Strategy", ourHubActive ? "SHOOT" : "STOCKPILE / PASS");

      // 0.3 s rumble pulse on every hub-state flip
      double now = Timer.getFPGATimestamp();
      if (ourHubActive != lastHubActive) {
          rumbleEndTime = now + 0.3;
          lastHubActive = ourHubActive;
      }
      m_robotContainer.setDriverRumble(now < rumbleEndTime ? 1.0 : 0.0);
  }
  ```

- [ ] **Gotchas for whoever implements this:**
      - `getMatchTime()` returns **-1** with no FMS/practice-mode DS attached — the `t >= 0`
        guard above keeps hub state defaulting to *active* in the shop. Never use this
        value for control decisions, display/strategy only.
      - `getGameSpecificMessage()` is `""` during AUTO and until FMS sends the byte — same
        default-active path.
      - Do **not** gate the shooter on `ourHubActive` in code — referees/HUB lights are
        ground truth; this is a driver cue, not an interlock.
- [ ] Verify in sim: `./gradlew simulateJava` → sim GUI **DS → Game Data** field, type `B`,
      set alliance Blue, enable Teleop with the sim match timer running → confirm
      `Teleop/Our Hub Active` flips at 105/80/55/30 s and is `false` during Shift 1
      (Blue won auto → Blue inactive first).
- [ ] **[Non-code, pit checklist]** R704 bandwidth audit: with all three PhotonVision
      cameras (RAY/DINGYI/LEO) connected, open each camera's PhotonVision UI and cap the
      dashboard **stream** resolution/FPS so combined radio traffic stays well under
      **7.0 Mbps** (check live usage on the FMS/DS bandwidth graph at an event or with
      the radio's status page). Detection resolution is unaffected — only the MJPEG
      stream settings matter.

---

## Completion checklist

| Fix | Description | Status |
|-----|-------------|--------|
| 1 | Motor configs applied in code | ☐ |
| 2 | Real velocity + at-speed feed gate | ☐ |
| 3 | AutoPrime spins flywheel; dup registration removed | ☐ |
| 4 | IntakeArm/IntakeRoller split + requirements | ☐ |
| 5 | Vision in auto + real filters | ☐ |
| 6 | AimAndShoot supercommand | ☐ |
| 7 | Beam break (flagged, HW-blocked) | ☐ |
| 8 | Tunable shot constants | ☐ |
| 9 | Shoot-on-the-move (stretch) | ☐ |
| 10 | Hub-active tracker + rumble cue + bandwidth audit | ☐ |

**Final regression pass after all merged:** `./gradlew build && ./gradlew test`, sim-boot
every auto in the chooser without exceptions, then a full field run: every driver-2 binding,
every driver-1 binding, one full auto per starting position.
