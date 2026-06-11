// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Millimeter;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.1;
  }
  public static class ArmConstants {
    public static final boolean kInverted = true;
  }
  public static class RangefinderConstants {
    public static final double kLocationX = 0; //meters from the center of the robot
    public static final double kLocationY = 0.3; //meters from the center of the robot
    public static final double kAngleDegree = 0; //angle in degrees from robot forward
    public static final double kAngleRadian = edu.wpi.first.math.util.Units.degreesToRadians(kAngleDegree); //angle in radians from robot forward

    public static final Transform2d kRangeFinderOffset = new Transform2d(kLocationX, kLocationY, new Rotation2d(kAngleRadian));

    //
    public static final double kMinDistance = 0.4;
    public static final double kMaxDistance = 2;
    public static final double kMaxUseableAngularSpeed = 1; //radians a second
  
    
  }
  public static class DrivetrainConstants{
    //public static final double TRACK_WIDTH_IN_METERS = 0.155; 

    // Mecanum wheel locations 
    public static final double kFrontLeftLocationX = 0.0;
    public static final double kFrontLeftLocationY = 0.0;

    public static final double kFrontRightLocationX = 0.0;
    public static final double kFrontRightLocationY = 0.0;

    public static final double kBackLeftLocationX = 0.0;
    public static final double kBackLeftLocationY = 0.0;

    public static final double kBackRightLocationX = 0.0;
    public static final double kBackRightLocationY = 0.0;

    public static final Translation2d kFrontLeftLocation = new Translation2d(kFrontLeftLocationX, kFrontLeftLocationY);
    public static final Translation2d kFrontRightLocation = new Translation2d(kFrontRightLocationX, kFrontRightLocationY);
    public static final Translation2d kBackLeftLocation = new Translation2d(kBackLeftLocationX, kBackLeftLocationY);
    public static final Translation2d kBackRightLocation = new Translation2d(kBackRightLocationX, kBackRightLocationY);
    
    //  Odometry
    public static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    public static final double kCountsPerMotorShaftRev = 12.0;
    public static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
    public static final double kWheelDiameterMeter = Units.Meter.convertFrom(48, Millimeter); // 48 mm



    // Wheel Pids (they are put on smart dashboard so they are editable) 
    //You must restart code for changes to apply
    public static final double kP = 0.2;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.1;
    public static final double kV = 2.2;

    public static final double kMaxLinearXSpeedMPS = 0.6; //in meters per second
    public static final double kMaxLinearYSpeedMPS = 0.5; //in meters per second
    public static final double kMaxAngularSpeedRPS = 75.0492; //in radians per second


  
  }
  public static class PathplannerConstants{

    public static RobotConfig config;
    static {
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
      }
    }


    // Pid Constants

    //Translation PID constants
    public static final double kTranslationP = SmartDashboard.getNumber("kTranslationP", 5);
    public static final double kTranslationI = SmartDashboard.getNumber("kTranslationI", 0);
    public static final double kTranslationD = SmartDashboard.getNumber("kTranslationD", 0);

    public static final PIDConstants kTranslationPID = new PIDConstants(kTranslationP,kTranslationI,kTranslationD);
    // Rotation Pid Constants
    public static final double kRotationP = SmartDashboard.getNumber("kRotationP", 5);
    public static final double kRotationI = SmartDashboard.getNumber("kRotationI", 0);
    public static final double kRotationD = SmartDashboard.getNumber("kRotationD", 0);

    public static final PIDConstants kRotationPID = new PIDConstants(kTranslationP,kTranslationI,kTranslationD);

    static{
      SmartDashboard.putNumber("kTranslationP", kTranslationP);
      SmartDashboard.putNumber("kTranslationI", kTranslationI);
      SmartDashboard.putNumber("kTranslationD", kTranslationD);

      SmartDashboard.putNumber("kRotationP", kRotationP);
      SmartDashboard.putNumber("kRotationI", kRotationI);
      SmartDashboard.putNumber("kRotationD", kRotationD);

      SmartDashboard.setPersistent("kTranslationP");
      SmartDashboard.setPersistent("kTranslationI");
      SmartDashboard.setPersistent("kTranslationD");

      SmartDashboard.setPersistent("kRotationP");
      SmartDashboard.setPersistent("kRotationI");
      SmartDashboard.setPersistent("kRotationD");
    }

    
  }
}
