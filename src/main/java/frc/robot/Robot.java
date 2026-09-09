// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotStateMachine.RobotState;
import frc.robot.util.OnboardLogger;
import frc.robot.util.StatusSignalUtil;

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
   * Power Distribution Hub/Panel — total/per-channel current, input voltage, temperature.
   * Auto-detects CTRE PDP vs. REV PDH. Added 2026-09-09, see {@code plans/review_plan.md} R5-2.
   *
   * <p>Logged via {@link #m_robotLog} rather than {@code SmartDashboard}, alongside the other
   * whole-robot signals added in the same pass (R5-3 through R5-5). Gives a ground-truth total
   * current/energy figure to cross-check the sum of all 16 per-motor
   * {@code OnboardLogger.registerEnergy} totals against — see {@code plans/logging_plan.md}.
   */
  private final PowerDistribution m_pdh = new PowerDistribution();

  /** Timestamp of the previous {@link #robotPeriodic()} call — see R5-4. */
  private double m_lastLoopStartSec = Timer.getFPGATimestamp();
  /** Backs the {@code Robot/LoopTimeSec} signal registered in {@link #configureRobotLogging()}. */
  private double m_lastLoopDurationSec = 0.0;

  /** True once {@link #logMatchContextOnce()} has fired. See R5-3. */
  private boolean m_matchContextLogged = false;

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
   *   <li>The active AprilTag field layout is logged by name immediately after console
   *       capture starts, so every {@code .wpilog} records which field map was used without
   *       requiring a source read. See {@link Constants#FIELD_LAYOUT_SOURCE}'s Javadoc for
   *       why this matters — two different field layouts were loaded simultaneously until
   *       2026-09-09.
   *   <li>{@link PortForwarder} tunnels PhotonVision's HTTP dashboard (port 5800) through the
   *       robot radio so it remains accessible from the Driver Station laptop.
   *   <li>The health timer is started here so {@link #robotPeriodic()} can begin rate-limiting
   *       immediately.
   * </ul>
   */
  public Robot() {
    DataLogManager.start();
    DataLogManager.logConsoleOutput(true);
    System.out.println("[Constants] AprilTag field layout: " + Constants.FIELD_LAYOUT_SOURCE
        + " (" + Constants.APRIL_TAG_FIELD_LAYOUT.getTags().size() + " tags)");
    m_robotContainer = new RobotContainer();
    m_RobotStateMachine = RobotStateMachine.getInstance();
    PortForwarder.add(5800, "photonvision.local", 5800);
    m_healthTimer.start();
    configureRobotLogging();
  }

  /**
   * Registers whole-robot diagnostic signals (PDH, loop timing, brownout/CAN health) with a
   * dedicated {@code OnboardLogger}. Added 2026-09-09 — see {@code plans/review_plan.md}
   * R5-2, R5-4, R5-5. Unlike the 10 Hz {@code SmartDashboard} block in {@link #robotPeriodic()}
   * (display-only, NT4), everything here goes to the durable {@code .wpilog} file every loop.
   */
  private void configureRobotLogging() {
    OnboardLogger log = new OnboardLogger("Robot");

    // R5-4: wall-clock time between successive robotPeriodic() invocations. Not a WPILib
    // Tracer epoch — a plain field updated at the very top of robotPeriodic(), so it can't be
    // confused with the "<Subsystem>.periodic() epoch also covers simulationPeriodic()" trap
    // this session found earlier (plans/project_sim-loop-overruns memory).
    log.registerDouble("LoopTimeSec", () -> m_lastLoopDurationSec);

    // R5-2: PDH/PDP ground truth to cross-check the sum of all 16 per-motor
    // OnboardLogger.registerEnergy totals against (plans/logging_plan.md).
    log.registerDouble("PDH/TotalCurrentA", m_pdh::getTotalCurrent);
    log.registerDouble("PDH/TotalPowerW", m_pdh::getTotalPower);
    log.registerDouble("PDH/TotalEnergyJ", m_pdh::getTotalEnergy);
    log.registerDouble("PDH/VoltageV", m_pdh::getVoltage);
    log.registerDouble("PDH/TemperatureC", m_pdh::getTemperature);
    for (int channel = 0; channel < m_pdh.getNumChannels(); channel++) {
      final int ch = channel; // effectively-final capture for the lambda below
      log.registerDouble("PDH/Channel" + ch + "CurrentA", () -> m_pdh.getCurrent(ch));
    }

    // R5-7 aggregate: independent sum of all 16 per-motor OnboardLogger.registerEnergy
    // registrations, meant to be cross-checked against the PDH/PDP figures directly above —
    // see OnboardLogger.getTotalEnergyJ()'s Javadoc for what a large persistent gap means.
    log.registerDouble("EnergyJ", OnboardLogger::getTotalEnergyJ);
    log.registerDouble("PowerW", OnboardLogger::getTotalPowerW);

    // R5-5: brownout + full CAN health, not just percentBusUtilization (already on
    // SmartDashboard at 10 Hz below) — busOffCount/txFullCount/receiveErrorCount/
    // transmitErrorCount are the fields that actually diagnose a flaky bus. Battery voltage
    // is duplicated here (vs. the SmartDashboard-only copy below) so it lands in the durable
    // .wpilog, not just the live NT4 view.
    log.registerBoolean("IsBrownedOut", RobotController::isBrownedOut);
    log.registerDouble("BatteryVoltageV", RobotController::getBatteryVoltage);
    log.registerDouble("CANBusUtilizationPct",
        () -> RobotController.getCANStatus().percentBusUtilization * 100.0);
    log.registerDouble("CANBusOffCount", () -> (double) RobotController.getCANStatus().busOffCount);
    log.registerDouble("CANTxFullCount", () -> (double) RobotController.getCANStatus().txFullCount);
    log.registerDouble("CANReceiveErrorCount",
        () -> (double) RobotController.getCANStatus().receiveErrorCount);
    log.registerDouble("CANTransmitErrorCount",
        () -> (double) RobotController.getCANStatus().transmitErrorCount);
  }

  /**
   * Logs match context (event name, match type/number, alliance, FMS-attached) exactly once,
   * the first time the Driver Station is attached — so a {@code .wpilog} can be tied back to a
   * specific match after an event. Called every loop from {@link #robotPeriodic()}; the
   * {@link #m_matchContextLogged} guard makes every call after the first a no-op. Added
   * 2026-09-09 — see {@code plans/review_plan.md} R5-3.
   *
   * <p>Uses direct {@link StringLogEntry}/{@link edu.wpi.first.util.datalog.IntegerLogEntry}
   * writes rather than an {@code OnboardLogger} registration, for the same reason as
   * {@code RobotContainer.configureCommandLogging()}: this is a one-time event, not continuous
   * state a poll-every-loop supplier model fits well.
   */
  private void logMatchContextOnce() {
    if (m_matchContextLogged || !DriverStation.isDSAttached()) {
      return;
    }
    DataLog log = DataLogManager.getLog();
    new StringLogEntry(log, "Robot/MatchContext/EventName").append(DriverStation.getEventName());
    new StringLogEntry(log, "Robot/MatchContext/MatchType")
        .append(DriverStation.getMatchType().toString());
    new StringLogEntry(log, "Robot/MatchContext/Alliance")
        .append(DriverStation.getAlliance().map(Alliance::toString).orElse("Unknown"));
    new IntegerLogEntry(log, "Robot/MatchContext/MatchNumber").append(DriverStation.getMatchNumber());
    new IntegerLogEntry(log, "Robot/MatchContext/ReplayNumber").append(DriverStation.getReplayNumber());
    new BooleanLogEntry(log, "Robot/MatchContext/FMSAttached").append(DriverStation.isFMSAttached());
    m_matchContextLogged = true;
  }

  /**
   * Runs every 20 ms regardless of mode. Refreshes all bulk-registered CTRE status signals,
   * drives the WPILib command scheduler and the robot state machine, publishes system-health
   * metrics at 10 Hz, and flushes all {@link OnboardLogger} registrations to the
   * {@code .wpilog} file.
   *
   * <p>Health metrics (battery voltage, CAN utilization, RSL state) are rate-limited to 10 Hz
   * because they are slow-moving signals; writing them every 20 ms would waste NT4 bandwidth
   * and contributed to loop overruns before Stage 0 optimizations.
   *
   * <p>{@link StatusSignalUtil#refreshAll()} is called first, before the scheduler runs any
   * subsystem code, per its own documented contract. Until this call was added, it had no call
   * site anywhere in the codebase — every signal registered via
   * {@link StatusSignalUtil#registerRioSignals} (all of Shooter's and Turret's voltage/current/
   * temperature reads, used for control-loop math as well as logging) only updated at whatever
   * slow default background rate CTRE assigns each signal, not at the 50 Hz loop rate.
   *
   * <p>{@link OnboardLogger#logAll()} is called every loop, unlike the health metrics above —
   * motor voltage/current/energy are fast-moving signals where a 10 Hz sample would miss
   * transients (e.g., a current spike on ball contact). Until this call was added, every
   * value registered via {@code OnboardLogger} anywhere in the codebase was silently never
   * written — {@link OnboardLogger#logAll()} had no call site either.
   *
   * <p>Loop-duration measurement (feeds {@code Robot/LoopTimeSec}, registered in
   * {@link #configureRobotLogging()}) happens first, before anything else this method does,
   * so it captures the full wall-clock gap since the previous call — including any tail effect
   * from the previous loop. {@link #logMatchContextOnce()} is a cheap no-op after its first
   * successful call; see its own Javadoc.
   */
  @Override
  public void robotPeriodic() {
    double now = Timer.getFPGATimestamp();
    m_lastLoopDurationSec = now - m_lastLoopStartSec;
    m_lastLoopStartSec = now;
    StatusSignalUtil.refreshAll();
    CommandScheduler.getInstance().run();
    m_RobotStateMachine.periodic();
    OnboardLogger.logAll();
    logMatchContextOnce();
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
