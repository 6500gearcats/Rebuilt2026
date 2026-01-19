// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Shooter extends SubsystemBase {
  private TalonFX m_shooterMotor = new TalonFX(0);
  private TalonFX m_yawMotor = new TalonFX(0);
  private TalonFX m_pitchMotor = new TalonFX(0);

  /** Creates a new Shooter2. */
  public Shooter() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterSpeed", m_shooterMotor.get());
  }

  public void setShootSpeed(double speed){
    m_shooterMotor.set(speed);
  }

  public void setYawSpeed(double speed){
    m_yawMotor.set(speed);
  }

  public void setPitchSpeed(double speed){
    m_pitchMotor.set(speed);
  }
}
