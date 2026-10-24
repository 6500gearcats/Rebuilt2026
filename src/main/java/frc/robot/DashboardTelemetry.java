package frc.robot;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Shared dashboard debug switch; motor control must never depend on this setting. */
public final class DashboardTelemetry {
    public static final String DEBUG_ENABLED_KEY = "Telemetry/Debug Enabled";
    private static volatile boolean debugEnabled;

    private DashboardTelemetry() {}

    /** Start each robot run with detailed telemetry disabled. */
    public static void initialize() {
        debugEnabled = false;
        SmartDashboard.putBoolean(DEBUG_ENABLED_KEY, false);
    }

    /** Read on the robot thread; drivetrain callbacks only read the cached flag. */
    public static void update() {
        debugEnabled = SmartDashboard.getBoolean(DEBUG_ENABLED_KEY, false);
    }

    /** Each telemetry owner has its own 10 Hz publication gate. */
    public static final class DebugPublisher {
        private static final double PERIOD_SECONDS = 0.1;
        private final BooleanSupplier enabled;
        private final DoubleSupplier clock;
        private final Set<String> registeredData = new HashSet<>();
        private double lastPublication = Double.NEGATIVE_INFINITY;

        public DebugPublisher() {
            this(() -> debugEnabled, Timer::getFPGATimestamp);
        }

        DebugPublisher(BooleanSupplier enabled, DoubleSupplier clock) {
            this.enabled = enabled;
            this.clock = clock;
        }

        public boolean shouldPublish() {
            if (!enabled.getAsBoolean()) {
                lastPublication = Double.NEGATIVE_INFINITY;
                return false;
            }
            double now = clock.getAsDouble();
            if (now >= lastPublication && now - lastPublication < PERIOD_SECONDS) {
                return false;
            }
            lastPublication = now;
            return true;
        }

        /** Register debug visualizations once, when their owner first publishes. */
        public void putData(String key, Sendable data) {
            if (registeredData.add(key)) {
                SmartDashboard.putData(key, data);
            }
        }
    }
}
