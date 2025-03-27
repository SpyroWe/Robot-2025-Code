// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {

  /** Creates a new ElevatorSubsystem. */
  
  private TalonFX m_leftElevator = new TalonFX(10);//is inverted
    private TalonFX m_rightElevator = new TalonFX(9);
    public double pastElePos;
    public double RotorPosRelativeOffset=0;
    public double elevatorDirection;

 public ElevatorSubsystem() {
  
 }
 

public void moveElevator (boolean y, boolean b, boolean sensorTriggerd){
  //move down
  //makes sure coral is fully intaken(d?) before the elevator can move. Also lets the sensor ignore the elevator 
  SmartDashboard.putBoolean("testingSafetyyyy", !sensorTriggerd || getLeftEncoder()  < -1.6 );
  if(true){//!sensorTriggerd || getLeftEncoder()  < -1.6 
  if((b==true)&&(getLeftEncoder()<-.5)){
    if(getLeftEncoder()>-8.1){
      m_leftElevator.set(.05);
      m_rightElevator.set(.05);
      } else{
m_leftElevator.set(.5);
m_rightElevator.set(.5);
      }
  //checks if the elevator is higher or equal to the previous position to zero the encoder.
pastElePos = getLeftEncoder();
if(getLeftEncoder()>=pastElePos+0.5){
  RotorPosRelativeOffset = getLeftEncoder();
}
//move up
  } else if(y){//&&(getLeftEncoder()>-22.5)
    if(getLeftEncoder()<-17.5){
    m_leftElevator.set(-.3);
    m_rightElevator.set(-.3);
    } else{
    m_leftElevator.set(-.5);
    m_rightElevator.set(-.5);
    }
  }else{
    m_leftElevator.set(0);
    m_rightElevator.set(0);
  }
}
}



//sets the elevator to a certain position given a target position
public void setPoint(boolean button, double targetPos, boolean sensorTriggerd) {

    //makes sure coral is fully intaken(d?) before the elevator can move. Also lets the sensor ignor the elevator 
if(button){// && (!sensorTriggerd || getLeftEncoder()  < -1.6  )
   
  //decides if current pos is greater or less than target pos with the a buffer of .15
  
  //encoder values negative so signs are flipped
  if ((getLeftEncoder() > targetPos + 0.15)  ) {

    //decides which direction the motor should run, elevatorDirection is either 1 or -1
    elevatorDirection = (getLeftEncoder() - targetPos) / Math.abs(getLeftEncoder() - targetPos);

      //slows the speed of the motor when it is close to the target position
    if(((getLeftEncoder()-targetPos)*elevatorDirection <1) ){
      //speeds are multiplied by elevatorDirection( either 1 or -1) to make sure the motor runs in the correct direction
      m_leftElevator.set(-0.05*elevatorDirection );
      m_rightElevator.set(-0.05 *elevatorDirection);
    } 
    //changes speeds based on elvator position relative to target position
    else if (((getLeftEncoder() - targetPos)*elevatorDirection < 6) ){
      m_leftElevator.set(-0.1*elevatorDirection );
      m_rightElevator.set(-0.1 *elevatorDirection);
  
    } else if (((getLeftEncoder() - targetPos)*elevatorDirection > 6)) {
      m_leftElevator.set(-0.4 *elevatorDirection );
      m_rightElevator.set(-0.4 *elevatorDirection);

    } else {
      m_leftElevator.set(0 *elevatorDirection );
      m_rightElevator.set(0 *elevatorDirection);
    }
  } else {
    m_leftElevator.set(0);
    m_rightElevator.set(0);
    
  }
}
}




public double getLeftEncoder(){
  //returns relative encoder value 
  SmartDashboard.putNumber("LeftELE_Encoder",  m_leftElevator.getRotorPosition().getValueAsDouble()-RotorPosRelativeOffset);
return m_leftElevator.getRotorPosition().getValueAsDouble()-.8;
}




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
