// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
    private boolean lbPressed = false;
    private TalonFX m_climber = new TalonFX(14);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {}

  public void move(Boolean leftbumper, Boolean rightbumper){
    SmartDashboard.putBoolean("co_rb", rightbumper);
SmartDashboard.putNumber("ClimberEncoder", getClimbEncoder());

if(leftbumper){
  lbPressed=true;

  m_climber.set(-.5);
  SmartDashboard.putBoolean("co_lb", leftbumper);
} else if(lbPressed&&rightbumper){
  m_climber.set(.5);
}else{
  m_climber.set(0);
}
  }

  public double getClimbEncoder(){
    return m_climber.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
