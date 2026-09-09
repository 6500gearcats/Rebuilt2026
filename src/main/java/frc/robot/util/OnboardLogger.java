package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Subsystem-scoped logger that writes structured data directly to the WPILib
 * {@link DataLog} binary file ({@code .wpilog}) every loop cycle.
 *
 * <h2>How to use</h2>
 * <ol>
 *   <li>Create one instance per logical subsystem, passing a namespace string:
 *       <pre>  OnboardLogger log = new OnboardLogger("Shooter");</pre>
 *   <li>Register the values to log via the {@code register*()} methods. Each registration
 *       stores a supplier and a log-file entry — no file I/O happens at registration time.
 *   <li>Call {@link #logAll()} once per loop cycle (e.g., from {@code robotPeriodic()})
 *       to evaluate all suppliers and write their current values to the log.
 *       <b>If {@code logAll()} is never called, no data is ever written.</b>
 * </ol>
 *
 * <h2>Namespace</h2>
 * The {@code name} passed to the constructor becomes a prefix for every registered key.
 * For example, {@code new OnboardLogger("Shooter")} with {@code registerDouble("Velocity", …)}
 * creates the log path {@code Shooter/Velocity}. In AdvantageScope this appears as a folder
 * named {@code Shooter} containing a signal named {@code Velocity}.
 *
 * <h2>Self-registration</h2>
 * Each constructor call adds the new instance to the static {@code loggers} list so that
 * {@link #logAll()} can iterate over all instances without the caller keeping a reference.
 * This is intentional: subsystem IO classes can create their loggers in constructors without
 * wiring anything up in {@code robotPeriodic()}.
 */
public class OnboardLogger {
  private static final DataLog datalog = DataLogManager.getLog();
  private static final List<OnboardLogger> loggers = new ArrayList<>();

  private final String name;

  private final List<Pair<DoubleSupplier, DoubleLogEntry>> doubleEntries;
  private final List<Pair<BooleanSupplier, BooleanLogEntry>> booleanEntries;
  private final List<Pair<Supplier<String>, StringLogEntry>> stringEntries;
  private final List<Pair<Supplier<Pose2d>, StructLogEntry<Pose2d>>> pose2dEntries;
  private final List<Pair<Supplier<Pose2d[]>, StructArrayLogEntry<Pose2d>>> pose2dArrayEntries;
  private final List<Pair<Supplier<Pose3d>, StructLogEntry<Pose3d>>> pose3dEntries;
  private final List<Pair<Supplier<Pose3d[]>, StructArrayLogEntry<Pose3d>>> pose3dArrayEntries;
  private final List<Pair<Supplier<Transform2d>, StructLogEntry<Transform2d>>> transform2dEntries;
  private final List<Pair<Supplier<SwerveModuleState[]>, StructArrayLogEntry<SwerveModuleState>>> swerveModuleStateEntries;
  private final List<Pair<Supplier<SwerveModulePosition[]>, StructArrayLogEntry<SwerveModulePosition>>> swerveModulePositionEntries;

  /**
   * Creates a new logger for the subsystem identified by {@code name} and registers it
   * with the global log-all list so {@link #logAll()} picks it up automatically.
   *
   * @param name Subsystem namespace prefix (e.g., {@code "Shooter"}, {@code "Aiming"}).
   *             All keys registered on this logger will appear under this path in the log file.
   */
  public OnboardLogger(String name) {
    this.name = name;
    doubleEntries = new ArrayList<>();
    booleanEntries = new ArrayList<>();
    stringEntries = new ArrayList<>();
    pose2dEntries = new ArrayList<>();
    pose2dArrayEntries = new ArrayList<>();
    pose3dEntries = new ArrayList<>();
    pose3dArrayEntries = new ArrayList<>();
    transform2dEntries = new ArrayList<>();
    swerveModuleStateEntries = new ArrayList<>();
    swerveModulePositionEntries = new ArrayList<>();
    loggers.add(this);
  }

  /**
   * Registers a raw {@code double} signal. Log path: {@code <namespace>/<name>}.
   *
   * @param name     Signal name within this logger's namespace.
   * @param supplier Called each {@link #logAll()} to supply the current value.
   */
  public void registerDouble(String name, DoubleSupplier supplier) {
    DoubleLogEntry entry = new DoubleLogEntry(datalog, this.name + "/" + name);
    doubleEntries.add(new Pair<>(supplier::getAsDouble, entry));
  }

  /**
   * Registers a WPILib {@link Measure} (typed unit value) as a double in the specified unit.
   *
   * <p>The unit name is stored as the log entry's metadata so AdvantageScope can display
   * the correct unit label. If the supplier returns {@code null} at log time, a
   * {@link DriverStation#reportError} is posted (visible in the DS console) and {@code 0}
   * is written to avoid corrupting the log stream.
   *
   * @param name     Signal name.
   * @param supplier Supplier for the typed measurement; must not normally return null.
   * @param unit     The unit to convert into when writing (e.g., {@code Amps}, {@code Celsius}).
   * @param <T>      The unit type.
   */
  public <T extends Unit> void registerMeasurement(String name, Supplier<Measure<T>> supplier,
      T unit) {
    DoubleLogEntry entry = new DoubleLogEntry(datalog, this.name + "/" + name, unit.name());
    doubleEntries
        .add(new Pair<DoubleSupplier, DoubleLogEntry>(() -> {
          var val = supplier.get();
          if (val == null) {
            DriverStation.reportError("Attempted to read a null measurement for logging: " + this.name + "/" + name, true);
            return 0;
          }
          return supplier.get().in(unit);
        }, entry));
  }

  /**
   * Registers an accumulated energy signal (Joules) for a motor, integrating
   * voltage &times; current &times; elapsed time every {@link #logAll()} call.
   *
   * <p>The running total is Joules consumed since robot code start (process lifetime) — this
   * class has no reset-on-enable mechanism, matching every other {@code OnboardLogger}
   * registration. Log path: {@code <namespace>/<name>EnergyJ}.
   *
   * <p>Pass either stator or supply current depending on what the caller wants to measure
   * (stator current reflects mechanical load; supply current reflects battery draw). Callers
   * typically register this once per motor alongside {@link #registerMeasurement} calls for
   * that motor's raw voltage and current.
   *
   * @param name    Signal name (before the {@code EnergyJ} suffix is appended).
   * @param voltage Supplier for the motor's voltage each loop.
   * @param current Supplier for the motor's current each loop.
   */
  public void registerEnergy(String name, Supplier<Voltage> voltage, Supplier<Current> current) {
    DoubleLogEntry entry = new DoubleLogEntry(datalog, this.name + "/" + name + "EnergyJ", "Joules");
    // [0] = accumulated Joules, [1] = timestamp of the previous sample. A length-2 array is used
    // (rather than two local doubles) because the lambda below must mutate this state across
    // calls while only capturing effectively-final references.
    double[] state = new double[] {0.0, Timer.getFPGATimestamp()};
    doubleEntries.add(new Pair<DoubleSupplier, DoubleLogEntry>(() -> {
      double now = Timer.getFPGATimestamp();
      double dt = now - state[1];
      state[1] = now;
      state[0] += voltage.get().in(Volts) * current.get().in(Amps) * dt;
      return state[0];
    }, entry));
  }

  /**
   * Registers a boolean signal. Log path: {@code <namespace>/<name>}.
   *
   * @param name     Signal name.
   * @param supplier Called each {@link #logAll()} to supply the current value.
   */
  public void registerBoolean(String name, BooleanSupplier supplier) {
    BooleanLogEntry entry = new BooleanLogEntry(datalog, this.name + "/" + name);
    booleanEntries.add(new Pair<>(supplier::getAsBoolean, entry));
  }

  /**
   * Registers a string signal. Log path: {@code <namespace>/<name>}.
   *
   * @param name     Signal name.
   * @param supplier Called each {@link #logAll()} to supply the current value.
   */
  public void registerString(String name, Supplier<String> supplier) {
    StringLogEntry entry = new StringLogEntry(datalog, this.name + "/" + name);
    stringEntries.add(new Pair<>(supplier, entry));
  }

  /**
   * Registers a {@link Pose2d} struct signal. Log path: {@code <namespace>/<name>}.
   * AdvantageScope can visualize this as a robot pose on a field diagram.
   */
  public void registerPose(String name, Supplier<Pose2d> supplier) {
    StructLogEntry<Pose2d> entry =
        StructLogEntry.create(datalog, this.name + "/" + name, Pose2d.struct);
    pose2dEntries.add(new Pair<>(supplier, entry));
  }

  /** Registers a {@link Pose3d} struct signal. */
  public void registerPose3d(String name, Supplier<Pose3d> supplier) {
    StructLogEntry<Pose3d> entry =
        StructLogEntry.create(datalog, this.name + "/" + name, Pose3d.struct);
    pose3dEntries.add(new Pair<>(supplier, entry));
  }

  /** Registers an array of {@link Pose2d} structs (e.g., multiple vision estimates). */
  public void registerPoses(String name, Supplier<Pose2d[]> supplier) {
    StructArrayLogEntry<Pose2d> entry =
        StructArrayLogEntry.create(datalog, this.name + "/" + name, Pose2d.struct);
    pose2dArrayEntries.add(new Pair<>(supplier, entry));
  }

  /** Registers an array of {@link Pose3d} structs. */
  public void registerPoses3d(String name, Supplier<Pose3d[]> supplier) {
    StructArrayLogEntry<Pose3d> entry =
        StructArrayLogEntry.create(datalog, this.name + "/" + name, Pose3d.struct);
    pose3dArrayEntries.add(new Pair<>(supplier, entry));
  }

  /**
   * Registers an array of {@link SwerveModuleState} structs.
   * AdvantageScope can visualize this as animated swerve module arrows on a robot diagram.
   */
  public void registerSwerveModuleState(String name, Supplier<SwerveModuleState[]> supplier) {
    StructArrayLogEntry<SwerveModuleState> entry = StructArrayLogEntry.create(datalog, this.name + "/" + name, SwerveModuleState.struct);
    swerveModuleStateEntries.add(new Pair<>(supplier, entry));
  }

  /** Registers an array of {@link SwerveModulePosition} structs (encoder + angle per module). */
  public void registerSwerveModulePosition(String name, Supplier<SwerveModulePosition[]> supplier) {
    StructArrayLogEntry<SwerveModulePosition> entry = StructArrayLogEntry.create(datalog, this.name + "/" + name, SwerveModulePosition.struct);
    swerveModulePositionEntries.add(new Pair<>(supplier, entry));
  }

  /** Registers a {@link Transform2d} struct signal. */
  public void registerTransform2d(String name, Supplier<Transform2d> supplier) {
    StructLogEntry<Transform2d> entry =
        StructLogEntry.create(datalog, this.name + "/" + name, Transform2d.struct);
    transform2dEntries.add(new Pair<>(supplier, entry));
  }

  /** Writes all registered signals for this logger to the log file. Called by {@link #logAll()}. */
  private void log() {
    for (Pair<DoubleSupplier, DoubleLogEntry> pair : doubleEntries) {
      pair.getSecond().update(pair.getFirst().getAsDouble());
    }
    for (Pair<BooleanSupplier, BooleanLogEntry> pair : booleanEntries) {
      pair.getSecond().update(pair.getFirst().getAsBoolean());
    }
    for (Pair<Supplier<String>, StringLogEntry> pair : stringEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Pose2d>, StructLogEntry<Pose2d>> pair : pose2dEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Pose2d[]>, StructArrayLogEntry<Pose2d>> pair : pose2dArrayEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Pose3d>, StructLogEntry<Pose3d>> pair : pose3dEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Pose3d[]>, StructArrayLogEntry<Pose3d>> pair : pose3dArrayEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<Transform2d>, StructLogEntry<Transform2d>> pair : transform2dEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<SwerveModuleState[]>, StructArrayLogEntry<SwerveModuleState>> pair : swerveModuleStateEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
    for (Pair<Supplier<SwerveModulePosition[]>, StructArrayLogEntry<SwerveModulePosition>> pair : swerveModulePositionEntries) {
      pair.getSecond().update(pair.getFirst().get());
    }
  }

  /**
   * Evaluates every registered supplier across all {@link OnboardLogger} instances and writes
   * the current values to the {@code .wpilog} file.
   *
   * <p><b>This method must be called once per loop cycle</b> — typically from
   * {@code Robot.robotPeriodic()} — or no data will ever be written to the log, regardless of
   * how many signals have been registered. The WPILib {@link DataLogManager} handles the
   * actual file I/O on a background thread, so this call is non-blocking.
   */
  public static void logAll() {
    for (OnboardLogger logger : loggers) {
      logger.log();
    }
  }
}
