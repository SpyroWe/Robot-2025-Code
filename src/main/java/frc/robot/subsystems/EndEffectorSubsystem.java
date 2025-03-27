// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//seems important, i took a semicolon off somewhere in the code..see if you can find it
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class EndEffectorSubsystem extends SubsystemBase {

  /** Creates a new Endofector. */

  private SparkMax m_roller = new SparkMax(12,MotorType.kBrushless);
  private CANrange ProxySensor = new CANrange(23);
  //private SparkMax m_intake = new SparkMax(13,MotorType.kBrushless);
  public EndEffectorSubsystem() {
  
  }
  public static  GenericHID.RumbleType kBothRumble = GenericHID.RumbleType.kLeftRumble;

public void rumbleController(boolean sensorTriggerd){
     CommandXboxController m_driverController = new CommandXboxController(0);
     if(sensorTriggerd){
     m_driverController.setRumble(kBothRumble, 0);
     }

}
public void move(boolean leftBumper, boolean rightBumper, boolean co_leftTrigger, boolean co_rightTrigger, double ElePos){
  SmartDashboard.putNumber("PrxySens",ProxySensor.getAmbientSignal().getValueAsDouble() );
  SmartDashboard.putBoolean("proxyTrig", sensorTriggerd());
  SmartDashboard.putBoolean("rigtrig", leftBumper);
  if(leftBumper){//&&sensorTriggerd()
  
    m_roller.set(-.4);//og speed:-.075
    SmartDashboard.putBoolean("lbProx", ProxySensor.getAmbientSignal().getValueAsDouble()<20);
  
  // if(((ElePos<-.1) || ElePos>0)){
  //   m_intake.set(-.5);
  // }
   } 
   
 
else if(rightBumper){
  m_roller.set(-.25); 
} else if (co_leftTrigger){
  m_roller.set(-.5);
} else if (co_rightTrigger && (ElePos <-15)){
  m_roller.set(1);
}else{
  m_roller.set(0);
}
}
public void resetRB() {
  hasTripped = false;
  hasReversed = false;
}
public boolean hasTripped=false;
public boolean hasReversed=false;
  public void moveee(boolean leftBumper, boolean rightBumper, double ElePos) {
    SmartDashboard.putBoolean("reverseIntakeLogk", !sensorTriggerd()&&(hasTripped&&leftBumper));
if (sensorTriggerd()){
  hasTripped=true;
}
   if((sensorTriggerd()&&leftBumper)&&!hasReversed){
    m_roller.set(-.1);
    //reverses
   } else if(!sensorTriggerd()&&(hasTripped&&leftBumper)) {
    hasReversed=true;
m_roller.set(.042);
   }else if(rightBumper){
    resetRB();
    m_roller.set(-.5);
   }else{
    m_roller.set(0);
   }
   
}


public boolean sensorTriggerd(){
  if(ProxySensor.getAmbientSignal().getValueAsDouble()<20){
   // ProxySensor.setRangingMode();
    return true;
  }
  else{
    return false;
  }
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
