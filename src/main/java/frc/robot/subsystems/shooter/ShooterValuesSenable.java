package frc.robot.subsystems.shooter;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Sendable container that exposes shooter distance and speed to SmartDashboard / Shuffleboard.
 *
 * <p>Implements WPILib's {@link Sendable} so the object can be posted directly with
 * {@code SmartDashboard.putData()} and edited in the dashboard widget — useful
 * during manual tuning sessions to adjust shot parameters without redeploying.
 */
public class ShooterValuesSenable implements Sendable {
    private double distance;
    private double shooterSpeed;

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Sendable");
        builder.addDoubleProperty("Distance", this::getDist, this::setDist);
        builder.addDoubleProperty("Shooter Speed", this::getShooterSpeed, this::setShooterSpeed);
    }

    /** @param dist Robot-to-hub distance in meters. */
    public void setDist(double dist) {
        this.distance = dist;
    }

    /** @param speed Target flywheel speed in RPS. */
    public void setShooterSpeed(double speed) {
        this.shooterSpeed = speed;
    }

    /** @return Current distance value in meters. */
    public double getDist() {
        return distance;
    }

    /** @return Current flywheel speed in RPS. */
    public double getShooterSpeed() {
        return shooterSpeed;
    }
}
