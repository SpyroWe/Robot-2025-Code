// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  private ElevatorSubsystem elevatorbase;
  private BooleanSupplier yButton;
  private BooleanSupplier bButton;
  private BooleanSupplier co_aButton;
  private BooleanSupplier co_yButton;
  private BooleanSupplier co_bButton;
  private BooleanSupplier sensorTriggered;
  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(ElevatorSubsystem elesub, BooleanSupplier y, BooleanSupplier b, BooleanSupplier coA, BooleanSupplier coY, BooleanSupplier coB, BooleanSupplier sensTrig) {
    elevatorbase=elesub;
    yButton = y;
    bButton = b;
    co_aButton = coA;
    co_bButton = coB;
    co_yButton = coY;
    sensorTriggered = sensTrig;

    addRequirements(elevatorbase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("co_Y", co_yButton.getAsBoolean());
  elevatorbase.moveElevator(yButton.getAsBoolean(), bButton.getAsBoolean(),sensorTriggered.getAsBoolean());
   elevatorbase.setPoint(co_aButton.getAsBoolean(), elevatorConstants.setPoint1, sensorTriggered.getAsBoolean());
   elevatorbase.setPoint(co_bButton.getAsBoolean(), elevatorConstants.setPoint2, sensorTriggered.getAsBoolean());
   elevatorbase.setPoint(co_yButton.getAsBoolean(), elevatorConstants.setPoint3, sensorTriggered.getAsBoolean());
   SmartDashboard.putNumber("LeftELEencoder", elevatorbase.getLeftEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
