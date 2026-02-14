package frc.robot.utility;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ShooterValuesSenable implements Sendable {
    private double distance;
    private double shooterSpeed;

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter Sendable");
        builder.addDoubleProperty("Distance", null, this::setDist);
        builder.addDoubleProperty("Shooter Speed", null, this::setShooterSpeed);
    }

    public void setDist(double dist) {
        this.distance = dist;
    }

    public void setShooterSpeed(double speed) {
        this.shooterSpeed = speed;
    }
}
