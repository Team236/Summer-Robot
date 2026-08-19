# FRC 2026 AI Developer Instructions

This guide provides critical context for AI agents working in this codebase.

## 🏗️ Architecture & Core Patterns

### AdvantageKit & IO Layer Abstraction
This project heavily relies on **AdvantageKit** for deterministic logging, replay, and simulation.
- **Hardware/Subsystem Split**: Subsystems are separated into a high-level subsystem class (e.g., `DriveSubsystem.java`), an IO interface (e.g., `DriveIO.java`), and hardware implementations (e.g., `DriveHardware.java`). This allows the subsystem logic to run identically in simulation or replay mode.
- **Inputs Structs**: Data from hardware is passed to the subsystem via nested `Inputs` classes (e.g., `DriveIO.DriveIOInputs`).
- **AutoLogging**: AdvantageKit's annotation processor generates `*AutoLogged` classes from the `Inputs` struct. Always process inputs in the subsystem's `periodic()` method using `Logger.processInputs("SubsystemName", mInputs);`.
- **Dependency Injection**: Subsystems and IO layers are instantiated in `RobotContainer.java` and passed via constructor arguments. **Avoid Singleton patterns** (e.g., `getInstance()`).

### Shared State & `RobotState`
- Use the `RobotState` class as the central repository for shared robot state (e.g., global odometry, megatag vision estimates, alliance color).
- `RobotState` uses `ConcurrentTimeInterpolatableBuffer` to handle latency compensation for vision and odometry.
- Inject `RobotState` into hardware/subsystems rather than accessing it globally.

### CTRE Phoenix 6 Swerve
- The drivetrain uses CTRE's `SwerveDrivetrain` API directly within the hardware layer (`DriveHardware.java`).
- High-frequency odometry (250Hz) is handled in a separate thread. Ensure any data passed from the CTRE telemetry thread to the main robot thread is thread-safe (e.g., using `AtomicReference<SwerveDriveState>` as seen in `DriveHardware`).

## 🔄 Developer Workflows

- **Building & Code Gen**: Because of AdvantageKit's `@AutoLog` annotation, you must compile the project to generate the `*AutoLogged` classes. **Use `./gradlew build`**. If an `*AutoLogged` class appears missing or red in the IDE, build the project first.
- **Timestamps**: Avoid WPILib's `Timer.getFPGATimestamp()`. Prefer `RobotTime.getTimestampSeconds()` or CTRE's `Utils.getCurrentTimeSeconds()` for logic requiring time to ensure compatibility with replay and simulation.
- **Replay Considerations**: The robot mode (`REAL`, `SIM`, `REPLAY`) is configured in `Constants.java` and processed in `Robot.java`. When writing subsystem logic, assume inputs can be replayed and avoid interacting with hardware directly outside of the `mIo` layer.

## 📚 Code Conventions

- **Prefixes**: Member variables should be prefixed with `m` (e.g., `mDriveSubsystem`, `mIo`, `mRobotState`).
- **Constants**: Store all configuration values in `Constants.java`. Use nested static classes for logical grouping (e.g., `Constants.DriveConstants`).
- **WPILib Command-Based**: Adhere to WPILib's command-based framework. Prefer inline commands, lambda factories, or `Commands.run(...)` inside the subsystem for simple actions, and dedicated classes in the `commands/` directory for complex actions (e.g., `TeleopSwerveDrive.java`).
