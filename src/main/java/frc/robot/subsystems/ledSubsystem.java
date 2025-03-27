// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledSubsystem extends SubsystemBase {
  PWM led = new PWM(9);
  /** Creates a new ledSubsystem. */
  public ledSubsystem() {}
public void setBlinkin(double value){
  led.setSpeed(value);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
