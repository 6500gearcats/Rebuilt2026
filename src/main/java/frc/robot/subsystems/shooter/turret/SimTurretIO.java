package frc.robot.subsystems.shooter.turret;

public class SimTurretIO implements TurretIO {

    @Override
    public void setSpeed(double speed) {
        System.out.println("Dawg this is unimplemented - SimTurretIO");
    }

    @Override
    public void setPosition(double deg) {
        System.out.println("Dawg this is unimplemented - SimTurretIO");
    }

    @Override
    public void zeroPosition() {
        System.out.println("Dawg this is unimplemented - SimTurretIO");
    }

    @Override
    public void updateSlotConfigs() {
        System.out.println("Dawg this is unimplemented - SimTurretIO");
    }

    @Override
    public double getMotorPosition() {
        System.out.println("Dawg this is unimplemented - SimTurretIO");
        return 0;
    }

    @Override
    public double getSpeed() {
        System.out.println("Dawg this is unimplemented - SimTurretIO");
        return 0;
    }

}
