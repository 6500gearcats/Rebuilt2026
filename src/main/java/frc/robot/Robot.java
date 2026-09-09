// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotStateMachine.RobotState;
import frc.robot.util.OnboardLogger;

/**
 * Top-level robot class. This is instantiated by the WPILib runtime once and is responsible for
 * handing control to the {@link CommandScheduler} every loop cycle, managing match phase
 * transitions, and logging infrastructure.
 *
 * <p>{@link TimedRobot} calls the appropriate {@code Init}, {@code Periodic}, and {@code Exit}
 * methods based on the current Driver Station mode. The 20 ms robot loop frequency is set by
 * WPILib — do not add blocking calls anywhere in this class.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final RobotStateMachine m_RobotStateMachine;
  /** Rate-limits system-health SmartDashboard writes to 10 Hz to avoid NT4 flood. */
  private final Timer m_healthTimer = new Timer();

  /**
   * Robot constructor — runs once on power-on before any mode is enabled.
   *
   * <ul>
   *   <li>{@link DataLogManager#start()} opens a {@code .wpilog} binary file on the roboRIO
   *       ({@code /home/lvuser/logs/}) or a USB drive ({@code /u/logs/}) and automatically
   *       captures every NetworkTables key change to it. The Driver Station copies these files
   *       to the operator laptop after each match.
   *   <li>{@link DataLogManager#logConsoleOutput} routes {@code System.out} into the same
   *       {@code .wpilog} so error messages appear in AdvantageScope's Console tab.
   *   <li>{@link PortForwarder} tunnels PhotonVision's HTTP dashboard (port 5800) through the
   *       robot radio so it remains accessible from the Driver Station laptop.
   *   <li>The health timer is started here so {@link #robotPeriodic()} can begin rate-limiting
   *       immediately.
   * </ul>
   */
  public Robot() {
    DataLogManager.start();
    DataLogManager.logConsoleOutput(true);
    m_robotContainer = new RobotContainer();
    m_RobotStateMachine = RobotStateMachine.getInstance();
    PortForwarder.add(5800, "photonvision.local", 5800);
    m_healthTimer.start();
  }

  /**
   * Runs every 20 ms regardless of mode. Drives the WPILib command scheduler and the robot
   * state machine, publishes system-health metrics at 10 Hz, and flushes all
   * {@link OnboardLogger} registrations to the {@code .wpilog} file.
   *
   * <p>Health metrics (battery voltage, CAN utilization, RSL state) are rate-limited to 10 Hz
   * because they are slow-moving signals; writing them every 20 ms would waste NT4 bandwidth
   * and contributed to loop overruns before Stage 0 optimizations.
   *
   * <p>{@link OnboardLogger#logAll()} is called every loop, unlike the health metrics above —
   * motor voltage/current/energy are fast-moving signals where a 10 Hz sample would miss
   * transients (e.g., a current spike on ball contact). Until this call was added, every
   * value registered via {@code OnboardLogger} anywhere in the codebase was silently never
   * written — {@link OnboardLogger#logAll()} had no call site.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_RobotStateMachine.periodic();
    OnboardLogger.logAll();
    if (m_healthTimer.advanceIfElapsed(0.1)) {
      SmartDashboard.putNumber("Robot/BatteryVoltageV", RobotController.getBatteryVoltage());
      SmartDashboard.putNumber("Robot/CANBusUtilizationPct",
          RobotController.getCANStatus().percentBusUtilization * 100.0);
      SmartDashboard.putBoolean("Robot/RSLStatus", RobotController.getRSLState());
    }
  }

  /** Delegates to {@link RobotContainer#disableInitCode()} for any on-disable cleanup. */
  @Override
  public void disabledInit() {
    m_robotContainer.disableInitCode();
  }

  @Override
  public void disabledPeriodic() {
  }

  /** Delegates to {@link RobotContainer#disableExitCode()} to prepare subsystems for enable. */
  @Override
  public void disabledExit() {
    m_robotContainer.disableExitCode();
  }

  /**
   * Schedules the autonomous command chosen in Shuffleboard/SmartDashboard and activates
   * the robot state machine so the shooter and turret can operate during auto.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    m_RobotStateMachine.setState(RobotState.ACTIVE);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  /** Cancels the autonomous command so teleop bindings take over cleanly. */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * Reads the FMS game-specific message once per teleop period to determine which scoring
   * windows are active for each alliance.
   *
   * <p>The game-specific message is a string set by the FMS (or the Driver Station in practice
   * mode). The first character encodes the hub color that the <em>other</em> team is targeting:
   * <ul>
   *   <li>{@code 'R'} — the opposing alliance is targeting the Red hub.
   *   <li>{@code 'B'} — the opposing alliance is targeting the Blue hub.
   * </ul>
   *
   * <p>This robot sets its own state based on whether the hub color matches its alliance:
   * <ul>
   *   <li>If the hub color matches this robot's alliance (e.g., Blue bot + Blue hub), the robot
   *       is set INACTIVE — it is not supposed to shoot into its own hub at that moment.
   *   <li>If the hub color does NOT match (e.g., Blue bot + Red hub), the robot is ACTIVE —
   *       it should shoot now.
   * </ul>
   *
   * <p>The {@code hasData()} guard ensures this logic runs only once; subsequent calls are
   * no-ops once the message has been received and cached.
   */
  @Override
  public void teleopPeriodic() {
    if (!m_RobotStateMachine.hasData()) {
      m_RobotStateMachine.setGameData(DriverStation.getGameSpecificMessage());
      if (!m_RobotStateMachine.getGameData().isEmpty()) {
        switch (m_RobotStateMachine.getGameData().charAt(0)) {
          case 'B':
            if (m_RobotStateMachine.getAlliance().equals(Alliance.Blue)) {
              m_RobotStateMachine.setState(RobotState.INACTIVE);
            } else {
              m_RobotStateMachine.setState(RobotState.ACTIVE);
            }
            break;
          case 'R':
            if (m_RobotStateMachine.getAlliance().equals(Alliance.Red)) {
              m_RobotStateMachine.setState(RobotState.INACTIVE);
            } else {
              m_RobotStateMachine.setState(RobotState.ACTIVE);
            }
            break;
        }
      }
    }
  }

  @Override
  public void teleopExit() {
  }

  /** Cancels all running commands before test mode starts, giving test bindings a clean slate. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
