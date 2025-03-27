package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

public class Constants {
    public static final double maxSpeed = Units.feetToMeters(16);
    public static final double ROBOT_MASS = 63.5029;//(148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag


    public class elevatorConstants {
        public static final double setPoint1 = -4;
        public static final double setPoint2 = -11.4;
        public static final double setPoint3 = -22.65;
    }
    public static class OperatorConstants
    {
  
      // Joystick Deadband
      public static final double LEFT_X_DEADBAND  = 0.1;
      public static final double LEFT_Y_DEADBAND  = 0.1;
      public static final double RIGHT_X_DEADBAND = 0.1;
      public static final double TURN_CONSTANT    = 6;
    }
}
