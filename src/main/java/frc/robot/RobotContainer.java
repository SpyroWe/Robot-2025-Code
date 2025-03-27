// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;





import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.EndEffectorCommand;
import frc.robot.commands.LedCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ledSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ledSubsystem ledbase = new ledSubsystem();
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_coDriverController = new CommandXboxController(1);
  private final ElevatorSubsystem elevatorbase = new ElevatorSubsystem();
  private final EndEffectorSubsystem EndEfctBase = new EndEffectorSubsystem();
  private final ClimberSubsystem ClimberBase = new ClimberSubsystem();
  public static final GenericHID.RumbleType kBothRumble = GenericHID.RumbleType.kLeftRumble;

//   private ElevatorCommand autoElevatorCommand;
//   private EndEffectorCommand autoEndEffectorCommand;
  
  public RobotContainer() {
    

// Prevent the path from being flipped if the coordinates are already correct

    configureBindings();
    //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // drivebase.setDefaultCommand(
    //   !RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
  
// Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
   // NamedCommands.registerCommand("New Auto", driveFieldOrientedDirectAngle);

    Command driveFieldOrientedAnglularVelocity = //drivebase.testDriveCommand();
    drivebase.driveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getLeftY()*-1, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getLeftX()*-1, OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.povLeft().getAsBoolean(),
        () -> m_driverController.povRight().getAsBoolean());

        Command driveFieldOrientedDirectAngleSim      =drivebase.driveCommand(
          () -> MathUtil.applyDeadband(m_driverController.getLeftY()*-1, OperatorConstants.LEFT_Y_DEADBAND),
          () -> MathUtil.applyDeadband(m_driverController.getLeftX()*-1, OperatorConstants.LEFT_X_DEADBAND),
          () -> m_driverController.getRightX(),
          () -> m_driverController.y().getAsBoolean(),
          () -> m_driverController.b().getAsBoolean());

     drivebase.setDefaultCommand(
      !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

   


        
  }

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * -1,
      () -> m_driverController.getLeftX() * 1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

   SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(()->m_driverController.getRightX()*-1,
         ()-> m_driverController.getRightY()*-1)
      .headingWhile(true);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX())
      .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
      .deadband(.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim = driveAngularVelocitySim.copy()
      .withControllerHeadingAxis(() -> Math.sin(
          m_driverController.getRawAxis(2) *
              Math.PI)
          *
          (Math.PI *
              2),
          () -> Math.cos(
              m_driverController.getRawAxis(2) *
                  Math.PI)
              *
              (Math.PI *
                  2))
      .headingWhile(true);

      //Command driveFieldOrientedDirectAngleSim      = drivebase.driveFieldOriented(driveDirectAngleSim);
      //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
       //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(   driveDirectAngleKeyboard);
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  /*important*///Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  private void configureBindings() {
    m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //chat is my programming rizz? or am I cooked?

    //default for end effector
    EndEfctBase.setDefaultCommand(new EndEffectorCommand(EndEfctBase,
     m_driverController.leftBumper(), m_driverController.rightBumper(), ()-> elevatorbase.getLeftEncoder(), 
     () -> m_driverController.getLeftTriggerAxis()>0.01,
     () -> m_driverController.getRightTriggerAxis()>0.01));

     // elevator
    elevatorbase.setDefaultCommand(new ElevatorCommand(elevatorbase,
        () -> m_driverController.y().getAsBoolean(),
        () -> m_driverController.b().getAsBoolean(),
        ()-> m_coDriverController.a().getAsBoolean(),
        () -> m_coDriverController.y().getAsBoolean(),
        () -> m_coDriverController.b().getAsBoolean(),
        ()->EndEfctBase.sensorTriggerd()));
    //climber
    ClimberBase.setDefaultCommand(new ClimberCommand(ClimberBase,
        () -> m_coDriverController.rightBumper().getAsBoolean(),
        () -> m_coDriverController.leftBumper().getAsBoolean()));

    ledbase.setDefaultCommand(new LedCommand(ledbase, ()-> m_coDriverController.getLeftX()));
  }
//Bigfoot is real, he kissed my feet
  public Command getAutonomousCommand() {
    drivebase.zeroGyro();
SmartDashboard.putBoolean("AUTOTESTPATH", true);
    return drivebase.getAutonomousCommand("SimulationTestAuto");
//    return Commands.print("No autonomous command configured");
    // return drivebase.driveCommand(
    //     () -> -0.5,
    //     () -> 0,
    //     () -> 0,
    //     () -> false,
    //     () -> false).withTimeout(2).andThen(drivebase.driveCommand(
    //         () -> 0,
    //         () -> 0,
    //         () -> 0,
    //         () -> false,
    //           () -> false));
              //.andThen(new ElevatorCommand(elevatorbase,
            // () -> false,
            // () -> false,
            // ()-> false,
            // () -> true,
            // () -> false,
            // ()->EndEfctBase.sensorTriggerd())).withTimeout(2).andThen(new EndEffectorCommand(EndEfctBase,
            // m_driverController.leftBumper(), m_driverController.rightBumper(), ()-> elevatorbase.getLeftEncoder())).withTimeout(1);
  }
}
//I love cowboy butts