# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

FRC Team 3566 robot code for the 2026 season, targeting a swerve-drive robot with a
Kraken-based drivetrain. Java 17, built on WPILib 2026 (GradleRIO), CTRE Phoenix 6,
PathPlanner, and PhotonVision. Robot main class is `frc.robot.Main`; team number 3566.

## Commands

All commands use the Gradle wrapper (`./gradlew`).

- **Build:** `./gradlew build`
- **Compile only:** `./gradlew compileJava`
- **Run tests:** `./gradlew test`
- **Run a single test class:** `./gradlew test --tests "frc.firecontrol.ShotCalculatorTest"`
- **Deploy to roboRIO:** `./gradlew deploy` (team number resolved from `.wpilib/wpilib_preferences.json`)
- **Simulate (with sim GUI + driverstation):** `./gradlew simulateJava`

Desktop support is enabled (`includeDesktopSupport = true`), so simulation and JUnit run locally.

## Architecture

Standard WPILib command-based structure. Two things drive most of the design:
everything routes through **CTRE Phoenix 6** hardware objects, and **PathPlanner**
owns autonomous sequencing.

### Top-level flow
- `Robot.java` (`TimedRobot`) — mode lifecycle. Runs the `CommandScheduler`, drives
  vision pose estimation in teleop, and computes the "Teleop Shift" clock/warnings on
  SmartDashboard. Uses `HootAutoReplay` to log & replay timestamp + joystick data.
- `RobotContainer.java` — the wiring hub. Instantiates all subsystems, binds both Xbox
  controllers, and registers every PathPlanner `NamedCommand` / `EventTrigger`. Start
  reading here to understand what a button or auto step does.
- `Constants.java` — CAN IDs (`Motors`), arm setpoints/soft limits (`Arm`), AprilTag IDs,
  and vision/camera config. Motor gains for the shooter/intake are set **in the subsystem
  constructors**, not here.

### Subsystems (`frc.robot.subsystems`)
- `CommandSwerveDrivetrain` — extends CTRE's `SwerveDrivetrain`, constructed via
  `TunerConstants.createDrivetrain()`. Also wires PathPlanner's `AutoBuilder`.
- `Shooter` — 4 `TalonFX`: flywheel pair (left follows right via `StrictFollower`) and
  indexer/feeder pair. Velocity is closed-loop (`VelocityVoltage`) in rotations/sec (RPS).
  `getAutoPower(distance)` is the distance→RPS lookup equation (currently a log fit).
- `Intake` — roller `TalonFX` + arm `TalonFX` with a `CANcoder`. Arm uses Motion Magic
  with separate up/down cruise/accel profiles.
- `Vision` — PhotonVision `PhotonPoseEstimator` feeding pose into the drivetrain's estimator,
  with ambiguity/rotation filtering. Includes a `VisionSystemSim` path for simulation.
  Field layout is `AprilTagFields.k2026RebuiltAndymark`.

### Commands (`frc.robot.commands`)
Grouped by subsystem: `intake/`, `shooter/`, `vision/`. Notable helpers:
- `vision/TagUtil` — the AprilTag lookup layer. Maps semantic hub positions to
  alliance-correct tag IDs (`getTargettingId()` switches blue/red) and computes drive-to poses.
- `shooter/PrimeAndShoot` vs `PrimeAndShootFixed` — the former solves flywheel power from
  pose→hub distance; the latter uses a fixed RPS passed in (used for tuned per-position autos).
- `vision/HeadToAngle`, `vision/TurnToHub`, `vision/DriveToPoseNew` — heading/pose-assist
  drive commands layered over the teleop swerve request.

### Fire control (`frc.firecontrol`)
Standalone shoot-on-the-move math, **not tied to WPILib subsystems** (borrowed from Team 5962,
MIT-licensed). `ShotCalculator` does Newton-method SOTM solving with drag compensation over a
distance→RPM/TOF lookup table; `ProjectileSimulator`/`GenerateLookUpTable` generate that LUT;
`FuelPhysicsSim` is a game-piece physics sim for the sim GUI (currently commented out at the
call sites in `Robot`/`RobotContainer`).

### Autonomous
Autos are authored in **PathPlanner**, stored under `src/main/deploy/pathplanner/` (`.path`
and `.auto` files). At runtime `AutoBuilder.buildAutoChooser()` populates a
`SendableChooser` on the "Autonomous" Shuffleboard tab. Auto behavior is defined by the
`NamedCommands`/`EventTrigger` registrations in `RobotContainer.configureAutoCommand()` —
the string names there must match the markers set in the PathPlanner GUI.

`flip_paths.py` mirrors all `.path` files over the field's horizontal center line
(`y = 4.021`) to produce left/right symmetric autos, writing `FLIPPED_<name>` files.

## Conventions & gotchas
- Shooter/indexer speeds are **rotations per second (RPS)**, not percent power despite some
  method names — e.g. `setUpperPower(50)` means 50 RPS via closed-loop velocity.
- `getAutonomousCommand()` wraps chooser selections in `ProxyCommand` so a fresh command
  instance is built each auto run — required because WPILib commands can only be scheduled once
  and the team runs auto repeatedly during testing without restarting robot code.
- There are two Tuner constant files (`TunerConstants` / `TunerConstants2`) for different
  drivebases; `RobotContainer` uses `TunerConstants`. Both are Tuner X generated — regenerate
  rather than hand-editing swerve module constants.
- Vendor dependencies live in `vendordeps/` (Phoenix6, PathplannerLib, photonlib,
  WPILibNewCommands); update via the WPILib VS Code tooling, not by hand.
