// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EndEffectorCommand extends Command {
  /** Creates a new EndEffector. */
   private EndEffectorSubsystem EndEfctBase;
   private BooleanSupplier leftBumper;
   private BooleanSupplier rightBumper;
   private DoubleSupplier ElePos;
   private BooleanSupplier co_leftTrigger;
   private BooleanSupplier co_rightTrigger;
  public EndEffectorCommand(EndEffectorSubsystem EES, BooleanSupplier lb, BooleanSupplier rb, DoubleSupplier EP, BooleanSupplier lt, BooleanSupplier rt ) {
    // Use addRequirements() here to declare subsystem dependencies.
    EndEfctBase=EES;
    leftBumper=lb;
    rightBumper = rb;
    ElePos = EP;
    co_leftTrigger = lt;
    co_rightTrigger = rt;
    addRequirements(EES);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    EndEfctBase.move(leftBumper.getAsBoolean(),rightBumper.getAsBoolean(), co_leftTrigger.getAsBoolean(), co_rightTrigger.getAsBoolean(),ElePos.getAsDouble());
    EndEfctBase.rumbleController(EndEfctBase.sensorTriggerd());
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
