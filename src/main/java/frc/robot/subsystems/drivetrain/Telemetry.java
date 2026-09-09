package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Publishes drivetrain telemetry to NetworkTables and SmartDashboard.
 *
 * <p>Until 2026-09-09 this class also published robot pose a second time under
 * {@code Pose/robotPose} as a raw {@code double[3]} (x, y, rotation degrees) with a
 * {@code Pose/.type = "Field2d"} marker — the legacy numeric-array structured-data format
 * AdvantageScope has deprecated for removal in 2027. That publisher was pure redundancy:
 * {@link #drivePose} above already publishes the identical pose as a modern
 * {@code Pose2d.struct} under {@code DriveState/Pose}, which is what AdvantageScope should
 * be pointed at. Removed rather than migrated, since nothing needed the legacy topic. See
 * {@code plans/REVIEW_PROGRESS.md}'s "Findings during execution" section.
 */
public class Telemetry {
        private final double MaxSpeed;

        /**
         * Construct a telemetry object, with the specified max speed of the robot
         *
         * @param maxSpeed Maximum speed in meters per second
         */
        public Telemetry(double maxSpeed) {
                MaxSpeed = maxSpeed;

                /* Set up the module state Mechanism2d telemetry */
                for (int i = 0; i < 4; ++i) {
                        SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
                }
        }

        /* What to publish over networktables for telemetry */
        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

        /* Robot swerve drive state */
        private final NetworkTable driveStateTable = inst.getTable("DriveState");
        private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct)
                        .publish();
        private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable
                        .getStructTopic("Speeds", ChassisSpeeds.struct).publish();
        private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable
                        .getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
        private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable
                        .getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
        private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable
                        .getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
        private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
        private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency")
                        .publish();

        /* Mechanisms to represent the swerve module states */
        private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
                        new Mechanism2d(1, 1),
                        new Mechanism2d(1, 1),
                        new Mechanism2d(1, 1),
                        new Mechanism2d(1, 1),
        };
        /* A direction and length changing ligament for speed representation */
        private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
                        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
                        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Speed", 0.5, 0)),
        };
        /* A direction changing and length constant ligament for module direction */
        private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
                        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
                        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
                        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
                        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
                                        .append(new MechanismLigament2d("Direction", 0.1, 0, 0,
                                                        new Color8Bit(Color.kWhite))),
        };

        /**
         * Accept the swerve drive state and telemeterize it to SmartDashboard.
         */
        public void telemeterize(SwerveDriveState state) {
                /* Telemeterize the swerve drive state */
                drivePose.set(state.Pose);
                driveSpeeds.set(state.Speeds);
                driveModuleStates.set(state.ModuleStates);
                driveModuleTargets.set(state.ModuleTargets);
                driveModulePositions.set(state.ModulePositions);
                driveTimestamp.set(state.Timestamp);
                driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

                /* Telemeterize each module state to a Mechanism2d */
                for (int i = 0; i < 4; ++i) {
                        m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
                        m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
                        m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
                }
        }
}
