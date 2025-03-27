// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

import swervelib.SwerveDrive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Meter;
public class SwerveSubsystem extends SubsystemBase {
  double tx;
  double ty;
  public double maximumSpeed = Units.feetToMeters(16);
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  /** Creates a new SwerveSubsystem. */
public SwerveSubsystem() {
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));

    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    zeroGyro();

    // config is based on pathplanner settings
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(0.0020645, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.00338225, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();

}

  @Override
public void periodic() {
    // This method will be called once per scheduler run
}

public SwerveDrive getSwerveDrive() {
    return swerveDrive;
}

public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
}

public void zeroGyro() {
    swerveDrive.zeroGyro();
}

public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
}

  // idk what this means but im gonna guess it's useful
public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);

    swerveDrive.getStates();
}

public Rotation2d getHeading() {
    return getPose().getRotation();
}

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
public Pose2d getPose() {
    return swerveDrive.getPose();
}

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    xInput = Math.pow(xInput, 3);
    yInput = Math.pow(yInput, 3);
    return swerveDrive.swerveController.getTargetSpeeds(xInput,
        yInput,
        headingX,
        headingY,
        getHeading().getRadians(),
        maximumSpeed);
}

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
}

public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
}

  // this specific method is not used to drive the robot but its logic is.
public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
}

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother
   *                         controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother
   *                         controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   *                         smoother controls.
   * @return Drive command.
   */

  // Autoalign Variables
  double xAng = LimelightHelpers.getTX("limelight-driver");
  double yAng = LimelightHelpers.getTY("limelight-driver");
  double ta = LimelightHelpers.getTA("limelight-driver");
  double atHeight = .5;// relative to camera in inches
  double AtDistance = 0;
  double LimeLightAngle = 0;
  double angRot;

  public double getAtDirectionsA() {
    xAng = 0;// LimelightHelpers.getTX("limelight-driver");
    if (xAng == 0) {
      tx = 0;
      ty = 0;
      return 0;
    } else {
      if (xAng > 5) {
        return .5;
      } else if (xAng < -5) {
        return -.5;
      } else {
        return 0;
      }
    }

  }

  public boolean aprilTagDetected() {
    if(LimelightHelpers.getTX("limelight-driver") !=0){
      return true;
    }else if(LimelightHelpers.getTX("limelight-coDriver") != 0 ){
      return true;
    }else{
      return false;
    }
  }

  
  public boolean autoButtonUsed(BooleanSupplier y, BooleanSupplier b) {
    return y.getAsBoolean() || b.getAsBoolean();
  }

  public double getSwerveRotation(BooleanSupplier y, BooleanSupplier b, DoubleSupplier angularRotationX) {
    if (autoButtonUsed(y, b) && aprilTagDetected()) {
      if (b.getAsBoolean()) {
        angRot = bAutoRoation(angularRotationX);
      } else {
        return 0;
      }
    } else {
      angRot = -Math.pow(angularRotationX.getAsDouble(), 3) * 6;
    }
    return -angRot;
  }

public double bAutoRoation(DoubleSupplier angularRotationX) {
    return getAtDirectionsA();
  }

  // returns the X translation of the robot based on the controller input or autoalign
public double getSwerveTranslationX(DoubleSupplier translationX, BooleanSupplier y, BooleanSupplier b) {
    xAng = LimelightHelpers.getTX("limelight-driver");
    yAng = LimelightHelpers.getTY("limelight-driver") + LimeLightAngle;
    SmartDashboard.putBoolean("autoButtonUsed", autoButtonUsed(y, b));
    SmartDashboard.putBoolean("aprilTagdetected", aprilTagDetected());

    if (autoButtonUsed(y, b) && aprilTagDetected()) {

      if (b.getAsBoolean()) {
        return 0;
      } else {
        // y button has been pressed
        AtDistance = Math.abs(atHeight / Math.tan(Math.toRadians(yAng)));
        tx = AtDistance;
        ty = (AtDistance * Math.tan(Math.toRadians(xAng)));
        SmartDashboard.putNumber("yAng", yAng);
        SmartDashboard.putNumber("ty2", ty);
        SmartDashboard.putNumber("atd", AtDistance);
        SmartDashboard.putNumber("txx2", tx);
        // This makes tx and ty preportional to each other on a scale of when to
        // simulate controller input. (puts tx and ty on a scale of -1,1 like a
        // controller )
        if (Math.abs(ty) > Math.abs(tx)) {
          tx = tx / ty;
          ty = ty / ty;
        } else {
          ty = ty / tx;
          tx = tx / tx;
        }
        return tx;
      }
    } else {
      tx = getControllerTranslation(translationX);
      return tx;
    }
  }

  public double getControllerTranslation(DoubleSupplier translation) {
    return (Math.pow(translation.getAsDouble() * 1, 3) * 6);
  }

  // returns the Y translation of the robot based on the controller input or autoalign
  public double getSwerveTranslationY(DoubleSupplier translationY, BooleanSupplier y, BooleanSupplier b) {

    xAng = LimelightHelpers.getTX("limelight-driver");
    yAng = LimelightHelpers.getTY("limelight-driver") + LimeLightAngle;

    if (autoButtonUsed(y, b) && aprilTagDetected()) {

      if (b.getAsBoolean()) {
        return 0;
      } else {
        // y button has been pressed
        AtDistance = Math.abs(atHeight / Math.tan(Math.toRadians(yAng)));
        tx = AtDistance;
        ty = (AtDistance * Math.tan(Math.toRadians(xAng)));
        SmartDashboard.putNumber("yAng", yAng);
        SmartDashboard.putNumber("ty2", ty);
        SmartDashboard.putNumber("atd", AtDistance);
        SmartDashboard.putNumber("txx2", tx);
        // This makes tx and ty preportional to each other on a scale of when to
        // simulate controller input. (puts tx and ty on a scale of -1,1 like a
        // controller )
        if (Math.abs(ty) > Math.abs(tx)) {
          tx = tx / ty;
          ty = ty / ty;
        } else {
          ty = ty / tx;
          tx = tx / tx;
        }
        return ty;
      }
    } else {
      ty = getControllerTranslation(translationY);
      return -ty;
    }
  }

public double getSwerveTranslationYFirst(DoubleSupplier translationY, BooleanSupplier y, BooleanSupplier b){
    xAng = LimelightHelpers.getTX("limelight-driver");
    yAng = LimelightHelpers.getTY("limelight-driver") + LimeLightAngle;

    if (autoButtonUsed(y, b) && aprilTagDetected()) {

      if (b.getAsBoolean()) {
        xAng = LimelightHelpers.getTX("limelight-coDriver");
        yAng = LimelightHelpers.getTY("limelight-coDriver") + LimeLightAngle;
        return 0;
      }else if(y.getAsBoolean()){
        xAng = LimelightHelpers.getTX("limelight-driver");
        yAng = LimelightHelpers.getTY("limelight-driver") + LimeLightAngle;
      }
       
    AtDistance = Math.abs(atHeight / Math.tan(Math.toRadians(yAng)));
       
     if (xAng > 5) {
      ty=.5;
      //tx=0;

      SmartDashboard.putNumber("yAng", yAng);
      SmartDashboard.putNumber("ty2", ty);
      SmartDashboard.putNumber("atd", AtDistance);
      SmartDashboard.putNumber("txx2", tx);

      return ty;
     } else if(xAng < -5){
      ty=-.5;
      //tx=0;

      SmartDashboard.putNumber("yAng", yAng);
      SmartDashboard.putNumber("ty2", ty);
      SmartDashboard.putNumber("atd", AtDistance);
      SmartDashboard.putNumber("txx2", tx);
      return ty;
     }else {
      return 0;
     }
    
        // This makes tx and ty preportional to each other on a scale of when to
        // simulate controller input. (puts tx and ty on a scale of -1,1 like a
        // controller )
        
      
    } else {
      ty = getControllerTranslation(translationY);
      return -ty;
    }
    
  }

  public double getSwerveTranslationXLast(DoubleSupplier translationX, BooleanSupplier y, BooleanSupplier b){
    xAng = LimelightHelpers.getTX("limelight-driver");
    yAng = LimelightHelpers.getTY("limelight-driver") + LimeLightAngle;
    SmartDashboard.putBoolean("autoButtonUsed", autoButtonUsed(y, b));
    SmartDashboard.putBoolean("aprilTagdetected", aprilTagDetected());

    if (autoButtonUsed(y, b) && aprilTagDetected()) {

      if (b.getAsBoolean()) {
        xAng = LimelightHelpers.getTX("limelight-coDriver");
        yAng = LimelightHelpers.getTY("limelight-coDriver") + LimeLightAngle;
      } else if (y.getAsBoolean()){
        xAng = LimelightHelpers.getTX("limelight-driver");
        yAng = LimelightHelpers.getTY("limelight-driver") + LimeLightAngle;
      }
        // y button has been pressed
       if(xAng < 5 && xAng > -5 && (yAng < -2)){
        
        //ty=0;
        tx=.5;
        return tx;
        }      
        else {
        return 0;
       }

      
    } else {
      tx = getControllerTranslation(translationX);
      return tx;
    }
  }
  
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX,
      BooleanSupplier y, BooleanSupplier b) {

    return run(() -> {

      SmartDashboard.putNumber("xPostion", getPose().getX());

      // Default Drive Code
      // swerveDrive.drive(
      // new Translation2d((Math.pow(translationX.getAsDouble() * 1, 3) * 6),
      // (-Math.pow(translationY.getAsDouble() * 1, 3) * 6)),
      // Math.pow(angularRotationX.getAsDouble(), 3) * 6,
      // true,
      // false);

      SmartDashboard.putNumber("yagslSpeed", Math.pow(-translationX.getAsDouble() * 1, 3) * 6);
      SmartDashboard.putNumber("yagsly", -Math.pow(translationY.getAsDouble() * 1, 3) * 6);
      SmartDashboard.putBoolean("dpadTest", b.getAsBoolean());

      // acctually moves robot
      // autoalign integrated drive code
      //very shady code here
      if(autoButtonUsed(y, b)){
      swerveDrive.drive(
          new Translation2d(getSwerveTranslationXLast(translationX, y, b), getSwerveTranslationYFirst(translationY, y, b)),
          getSwerveRotation(y, b, angularRotationX), true, false);
      } else {
        swerveDrive.drive(
          new Translation2d(getSwerveTranslationX(translationX, y, b), getSwerveTranslationY(translationY, y, b)),
          getSwerveRotation(y, b, angularRotationX), false, false);
      }

      SmartDashboard.putNumber("atDistance", AtDistance);
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("ty", ty);
    });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {

      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public double getGyroSwerve() {
    getHeading();
    return getHeading().getDegrees();
  }

  public Command getAutonomousCommand(String autoName) {

    return new PathPlannerAuto(autoName);
  }
}