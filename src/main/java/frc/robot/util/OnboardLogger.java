package frc.robot.util;

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
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A utility class to help log structured data to a DataLog without worrying about extra writes or
 * other nasty situations.
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

  public void registerDouble(String name, DoubleSupplier supplier) {
    DoubleLogEntry entry = new DoubleLogEntry(datalog, this.name + "/" + name);
    doubleEntries.add(new Pair<>(supplier::getAsDouble, entry));
  }

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

  public void registerBoolean(String name, BooleanSupplier supplier) {
    BooleanLogEntry entry = new BooleanLogEntry(datalog, this.name + "/" + name);
    booleanEntries.add(new Pair<>(supplier::getAsBoolean, entry));
  }

  public void registerString(String name, Supplier<String> supplier) {
    StringLogEntry entry = new StringLogEntry(datalog, this.name + "/" + name);
    stringEntries.add(new Pair<>(supplier, entry));
  }

  public void registerPose(String name, Supplier<Pose2d> supplier) {
    StructLogEntry<Pose2d> entry =
        StructLogEntry.create(datalog, this.name + "/" + name, Pose2d.struct);
    pose2dEntries.add(new Pair<>(supplier, entry));
  }

  public void registerPose3d(String name, Supplier<Pose3d> supplier) {
    StructLogEntry<Pose3d> entry =
        StructLogEntry.create(datalog, this.name + "/" + name, Pose3d.struct);
    pose3dEntries.add(new Pair<>(supplier, entry));
  }

  public void registerPoses(String name, Supplier<Pose2d[]> supplier) {
    StructArrayLogEntry<Pose2d> entry =
        StructArrayLogEntry.create(datalog, this.name + "/" + name, Pose2d.struct);
    pose2dArrayEntries.add(new Pair<>(supplier, entry));
  }

  public void registerPoses3d(String name, Supplier<Pose3d[]> supplier) {
    StructArrayLogEntry<Pose3d> entry =
        StructArrayLogEntry.create(datalog, this.name + "/" + name, Pose3d.struct);
    pose3dArrayEntries.add(new Pair<>(supplier, entry));
  }

  public void registerSwerveModuleState(String name, Supplier<SwerveModuleState[]> supplier) {
    StructArrayLogEntry<SwerveModuleState> entry = StructArrayLogEntry.create(datalog, this.name + "/" + name, SwerveModuleState.struct);
    swerveModuleStateEntries.add(new Pair<>(supplier, entry));
  }

  public void registerSwerveModulePosition(String name, Supplier<SwerveModulePosition[]> supplier) {
    StructArrayLogEntry<SwerveModulePosition> entry = StructArrayLogEntry.create(datalog, this.name + "/" + name, SwerveModulePosition.struct);
    swerveModulePositionEntries.add(new Pair<>(supplier, entry));
  }

  public void registerTransform2d(String name, Supplier<Transform2d> supplier) {
    StructLogEntry<Transform2d> entry =
        StructLogEntry.create(datalog, this.name + "/" + name, Transform2d.struct);
    transform2dEntries.add(new Pair<>(supplier, entry));
  }

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

  public static void logAll() {
    for (OnboardLogger logger : loggers) {
      logger.log();
    }
  }
}
