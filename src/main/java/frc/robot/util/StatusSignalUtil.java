package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;

/**
 * Static registry for CTRE Phoenix 6 status signals, enabling bulk CAN refresh.
 *
 * <h2>Why bulk refresh?</h2>
 * In Phoenix 6, each {@link StatusSignal} is normally refreshed individually the first time
 * it is read in a loop. When many signals are read one by one, the CAN bus time is wasted
 * on separate transfer round-trips. {@link BaseStatusSignal#refreshAll} issues a single
 * batched refresh for all registered signals at once, which is significantly faster and
 * reduces loop jitter.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>In each subsystem constructor, pass the relevant signals to
 *       {@link #registerRioSignals} (or {@link #registerCANivoreSignals} for CANivore-connected
 *       devices). Pass {@code false} as the signal's {@code waitForAll} argument to avoid
 *       blocking the constructor thread.
 *   <li>Call {@link #refreshAll()} once per loop cycle, before any subsystem reads its signals.
 *       The recommended location is at the top of {@code robotPeriodic()}.
 * </ol>
 *
 * <p>Two separate lists are maintained because the roboRIO CAN bus and the CANivore bus
 * run at different frequencies and must be refreshed independently.
 */
@SuppressWarnings("rawtypes")
public class StatusSignalUtil {
  private static List<BaseStatusSignal> rioSignals = new ArrayList<>();
  private static List<BaseStatusSignal> canivoreSignals = new ArrayList<>();

  public static final CANBus canivore = new CANBus("CANivore");
  public static final CANBus rio = CANBus.roboRIO();

  /**
   * Registers signals from devices on the roboRIO CAN bus for bulk refresh.
   * Call this from each subsystem constructor.
   *
   * @param signals Signals to register (obtained via {@code motor.getVelocity(false)}, etc.).
   */
  public static void registerRioSignals(StatusSignal... signals) {
    for (StatusSignal signal : signals) {
      rioSignals.add(signal);
    }
  }

  /**
   * Registers signals from devices on the CANivore bus for bulk refresh.
   * Call this from each subsystem constructor when the device is on the CANivore.
   *
   * @param signals Signals to register.
   */
  public static void registerCANivoreSignals(StatusSignal... signals) {
    for (StatusSignal signal : signals) {
      canivoreSignals.add(signal);
    }
  }

  /**
   * Issues a bulk refresh for all registered signals.
   *
   * <p><b>This must be called once per loop cycle</b> (typically at the top of
   * {@code robotPeriodic()}) before any subsystem reads a signal value. Without this call,
   * each signal will be refreshed individually on its own CAN transaction, wasting bus time
   * and introducing inconsistent latency across subsystems.
   */
  public static void refreshAll() {
    if (!rioSignals.isEmpty()) {
      BaseStatusSignal.refreshAll(rioSignals);
    }
    if (!canivoreSignals.isEmpty()) {
      BaseStatusSignal.refreshAll(canivoreSignals);
    }
  }
}
