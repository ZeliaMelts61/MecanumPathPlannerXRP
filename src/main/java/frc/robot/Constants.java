// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

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
  }
  public static class ArmConstants {
    public static final boolean kInverted = true;
  }
  public static class DrivetrainConstants{
    public static final double TRACK_WIDTH_IN_METERS = 0.155; 
    public static final double kP = SmartDashboard.getNumber("kP", 0.2);
    public static final double kI = SmartDashboard.getNumber("kI", 0.0);
    public static final double kD = SmartDashboard.getNumber("kD", 0.0);
    public static final double kS = SmartDashboard.getNumber("kS", 0.1);
    public static final double kV = SmartDashboard.getNumber("kV", 2.2);

    static{
      SmartDashboard.putNumber("kP", kP);
      SmartDashboard.putNumber("kI", kI);
      SmartDashboard.putNumber("kD", kD);
      SmartDashboard.putNumber("kS", kS);
      SmartDashboard.putNumber("kV", kV);
    }

  
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
  }
}
