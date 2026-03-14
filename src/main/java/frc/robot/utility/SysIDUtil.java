package frc.robot.utility;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.subsystems.turret.Turret;

public class SysIDUtil {
        private Optional<SysIdRoutine> routine = Optional.empty();
        private final VoltageOut m_voltReq = new VoltageOut(0.0);
        private Optional<Flywheel> m_flywheel = Optional.empty();
        private Optional<Turret> m_turret = Optional.empty();

        /*
         * Uses Singal Logger from CTRE
         */
        public SysIDUtil(Flywheel flywheel) {
                m_flywheel = Optional.of(flywheel);
                if (m_flywheel.isPresent()) {
                        routine = Optional.of(
                                        new SysIdRoutine(new Config(null, // Use default ramp rate (1 V/s)
                                                        Volts.of(4), // Reduce dynamic step voltage to 4 to prevent
                                                                     // brownout
                                                        null,
                                                        (state) -> SignalLogger.writeString("flywheel state", state
                                                                        .toString())), // Use default timeout (10 s)
                                                        new Mechanism((volts) -> m_flywheel.get().setControl(
                                                                        m_voltReq.withOutput(volts.in(Volts))),
                                                                        null,
                                                                        m_flywheel.get())));
                }
        }

        /*
         * Uses Singal Logger from CTRE
         */
        public SysIDUtil(Turret turret) {
                m_turret = Optional.of(turret);
                if (m_turret.isPresent()) {
                        routine = Optional.of(
                                        new SysIdRoutine(new Config(null,
                                                        Volts.of(0.5),
                                                        Time.ofBaseUnits(4, Seconds),
                                                        (state) -> SignalLogger.writeString("turret state", state
                                                                        .toString())),
                                                        new Mechanism((volts) -> m_turret.get().setControl(
                                                                        m_voltReq.withOutput(volts.in(Volts))),
                                                                        null,
                                                                        m_turret.get())));
                }
        }

        public SysIDUtil() {
        }

        public Command sysIdQuasistatic(Direction direction) {
                return routine.get().quasistatic(direction);
        }

        public Command sysIdDynamic(Direction direction) {
                return routine.get().dynamic(direction);
        }

        public Optional<SequentialCommandGroup> sysIdAll() {
                if (m_flywheel.isPresent()) {
                        return Optional.of(new SequentialCommandGroup(
                                        Commands.runOnce(SignalLogger::start),

                                        new RunCommand(() -> m_flywheel.get().setSpeed(0), m_flywheel.get())
                                                        .until(() -> Math.abs(m_flywheel.get().getSpeed()) < 50),

                                        sysIdDynamic(Direction.kForward),

                                        new RunCommand(() -> m_flywheel.get().setSpeed(0), m_flywheel.get())
                                                        .until(() -> Math.abs(m_flywheel.get().getSpeed()) < 50),

                                        sysIdDynamic(Direction.kReverse),

                                        new RunCommand(() -> m_flywheel.get().setSpeed(0), m_flywheel.get())
                                                        .until(() -> Math.abs(m_flywheel.get().getSpeed()) < 50),

                                        sysIdQuasistatic(Direction.kForward),

                                        new RunCommand(() -> m_flywheel.get().setSpeed(0), m_flywheel.get())
                                                        .until(() -> Math.abs(m_flywheel.get().getSpeed()) < 50),

                                        sysIdQuasistatic(Direction.kReverse),

                                        new RunCommand(() -> m_flywheel.get().setSpeed(0), m_flywheel.get())
                                                        .until(() -> Math.abs(m_flywheel.get().getSpeed()) < 50),

                                        Commands.runOnce(SignalLogger::stop)));
                }

                else if (m_turret.isPresent()) {
                        return Optional.of(new SequentialCommandGroup(
                                        Commands.runOnce(SignalLogger::start),

                                        new RunCommand(() -> m_turret.get().setSpeed(0), m_turret.get())
                                                        .until(() -> Math.abs(m_turret.get().getSpeed()) < 50),

                                        sysIdDynamic(Direction.kForward),

                                        new RunCommand(() -> m_turret.get().setSpeed(0), m_turret.get())
                                                        .until(() -> Math.abs(m_turret.get().getSpeed()) < 50),

                                        sysIdDynamic(Direction.kReverse),

                                        new RunCommand(() -> m_turret.get().setSpeed(0), m_turret.get())
                                                        .until(() -> Math.abs(m_turret.get().getSpeed()) < 50),

                                        sysIdQuasistatic(Direction.kForward),

                                        new RunCommand(() -> m_turret.get().setSpeed(0), m_turret.get())
                                                        .until(() -> Math.abs(m_turret.get().getSpeed()) < 50),

                                        sysIdQuasistatic(Direction.kReverse),

                                        new RunCommand(() -> m_turret.get().setSpeed(0), m_turret.get())
                                                        .until(() -> Math.abs(m_turret.get().getSpeed()) < 50),

                                        Commands.runOnce(SignalLogger::stop)));
                } else {
                        return Optional.empty();
                }
        }

        public boolean isPresent() {
                if (m_flywheel.isPresent()) {
                        return true;
                } else if (m_turret.isPresent()) {
                        return true;
                }
                return false;
        }
}
