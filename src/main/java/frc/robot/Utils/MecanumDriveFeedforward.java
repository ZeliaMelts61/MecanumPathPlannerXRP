package frc.robot.Utils;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.LinearSystemId;
public class MecanumDriveFeedforward {
  /** Linear forward/backward velocity gain (V per m/s). */
  public final double kVTranslation;

  /** Linear forward/backward acceleration gain (V per m/s²). */
  public final double kATranslation;

  /** Strafe velocity gain (V per m/s). */
  public final double kVStrafe;

  /** Strafe acceleration gain (V per m/s²). */
  public final double kAStrafe;

  /** Rotational velocity gain (V per rad/s). */
  public final double kVRotation;

  /** Rotational acceleration gain (V per rad/s²). */
  public final double kARotation;

  private final MecanumDriveKinematics m_kinematics;

  /**
   * Creates a new MecanumDriveFeedforward with the specified parameters.
   *
   * @param kVTranslation The linear translational velocity gain in volts per (meters per second).
   * @param kATranslation The linear translational acceleration gain in volts per (meters per second squared).
   * @param kVStrafe The linear strafe velocity gain in volts per (meters per second squared).
   * @param kAStrafe The linear strafe acceleration gain in volts per (meters per second squared).
   * @param kVRotation The angular velocity gain in volts per (radians per second squared).
   * @param kARotation The angular acceleration gain in volts per (radians per second squared).
   * @param kinematics The kinimatics for a mecanum drive used for wheel locations.
   */
  public MecanumDriveFeedforward(
    double kVTranslation,
    double kATranslation,
    double kVStrafe,
    double kAStrafe,
    double kVRotation,
    double kARotation,
    MecanumDriveKinematics kinematics) {

    this.kVTranslation = kVTranslation;
    this.kATranslation = kATranslation;

    this.kVStrafe = kVStrafe;
    this.kAStrafe = kAStrafe;

    this.kVRotation = kVRotation;
    this.kARotation = kARotation;

    this.m_kinematics=kinematics;

  }

  /**
   * Creates a new MecanumDriveFeedforward with the specified parameters.
   *
   * @param kVTranslation The linear translational velocity gain in volts per (meters per second).
   * @param kATranslation The linear translational acceleration gain in volts per (meters per second squared).
   * @param kVStrafe The linear strafe velocity gain in volts per (meters per second squared).
   * @param kAStrafe The linear strafe acceleration gain in volts per (meters per second squared).
   * @param kVRotation The angular velocity gain in volts per (radians per second squared).
   * @param kARotation The angular acceleration gain in volts per (radians per second squared).
   * @param frontLeftWheelMeters The location of the front-left wheel relative to the physical
   *     center of the robot.
   * @param frontRightWheelMeters The location of the front-right wheel relative to the physical
   *     center of the robot.
   * @param rearLeftWheelMeters The location of the rear-left wheel relative to the physical center
   *     of the robot.
   * @param rearRightWheelMeters The location of the rear-right wheel relative to the physical
   *     center of the robot.
   */
  public MecanumDriveFeedforward(
    double kVTranslation,
    double kATranslation,
    double kVStrafe,
    double kAStrafe,
    double kVRotation,
    double kARotation,
    Translation2d frontLeftWheelMeters,
    Translation2d frontRightWheelMeters,
    Translation2d rearLeftWheelMeters,
    Translation2d rearRightWheelMeters) {
        
    this(
      kVTranslation,
      kATranslation,
      kVStrafe,
      kAStrafe,
      kVRotation,
      kARotation,
      new MecanumDriveKinematics(
        frontLeftWheelMeters,
        frontRightWheelMeters,
        rearLeftWheelMeters,
        rearRightWheelMeters
        )
    );
  }

  /**
   * Creates a new MecanumDriveFeedforward with the specified parameters.
   *
   * @param kVTranslation The linear translational velocity gain in volts per (meters per second).
   * @param kATranslation The linear translational acceleration gain in volts per (meters per second squared).
   * @param kVStrafe The linear strafe velocity gain in volts per (meters per second squared).
   * @param kAStrafe The linear strafe acceleration gain in volts per (meters per second squared).
   * @param kVRotation The angular velocity gain in volts per (radians per second squared).
   * @param kARotation The angular acceleration gain in volts per (radians per second squared).
   */
  public MecanumDriveFeedforward(
    double kVTranslation,
    double kATranslation,
    double kVStrafe,
    double kAStrafe,
    double kVRotation,
    double kARotation) {

    this(
      kVTranslation,
      kATranslation,
      kVStrafe,
      kAStrafe,
      kVRotation,
      kARotation,

      new Translation2d(0.5, 0.5),
      new Translation2d(0.5, -0.5),
      new Translation2d(-0.5, 0.5),
      new Translation2d(-0.5, -0.5));
  }

  public record MecanumDriveWheelVoltages(
    double frontLeft,
    double frontRight,
    double rearLeft,
    double rearRight
  ) {}

  


  /**
   * Calculates the mecanum drive feedforward inputs given chassis speeds setpoints.
   *
   * @param current The current chassis speed of the robot.
   * @param next The next chassis speed of the robot.
   * @param dtSeconds Discretization timestep.
   * @return A MecanumDriveWheelVoltages object containing the computed feedforward voltages.
   */
  public MecanumDriveWheelVoltages calculateWithoutkA(ChassisSpeeds current, ChassisSpeeds next, double dtSeconds){
    // Add acceleration calculations.
    double ax = (next.vxMetersPerSecond - current.vxMetersPerSecond) / dtSeconds;
    double ay = (next.vyMetersPerSecond - current.vyMetersPerSecond) / dtSeconds;
    double alpha = (next.omegaRadiansPerSecond - current.omegaRadiansPerSecond) / dtSeconds;

    // Calculate feedforward voltages for each chassis axis.
    double translationVoltage =
      kVTranslation * next.vxMetersPerSecond;
      //+ kATranslation * ax;

    double strafeVoltage =
      kVStrafe * next.vyMetersPerSecond;
      //+ kAStrafe * ay;

    double rotationVoltage =
      kVRotation * next.omegaRadiansPerSecond;
      //+ kARotation * alpha;


    // Get wheel space voltages for each axis.
    MecanumDriveWheelSpeeds translation =
      m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(1.0, 0.0, 0.0));

    MecanumDriveWheelSpeeds strafe =
      m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(0.0, 1.0, 0.0));

    MecanumDriveWheelSpeeds rotation =
      m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(0.0, 0.0, 1.0));


    // Add the contributions.
    return new MecanumDriveWheelVoltages(
      translation.frontLeftMetersPerSecond * translationVoltage
        + strafe.frontLeftMetersPerSecond * strafeVoltage
        + rotation.frontLeftMetersPerSecond * rotationVoltage,

      translation.frontRightMetersPerSecond * translationVoltage
        + strafe.frontRightMetersPerSecond * strafeVoltage
        + rotation.frontRightMetersPerSecond * rotationVoltage,

      translation.rearLeftMetersPerSecond * translationVoltage
        + strafe.rearLeftMetersPerSecond * strafeVoltage
        + rotation.rearLeftMetersPerSecond * rotationVoltage,

      translation.rearRightMetersPerSecond * translationVoltage
        + strafe.rearRightMetersPerSecond * strafeVoltage
        + rotation.rearRightMetersPerSecond * rotationVoltage
    );
  }

  /**
   * Calculates the mecanum drive feedforward inputs given chassis speeds setpoints.
   *
   * @param current The current chassis speed of the robot.
   * @param next The next chassis speed of the robot.
   * @param dtSeconds Discretization timestep.
   * @return A MecanumDriveWheelVoltages object containing the computed feedforward voltages.
   */
  public MecanumDriveWheelVoltages calculate(ChassisSpeeds current, ChassisSpeeds next, double dtSeconds){
    // Add acceleration calculations.
    double ax = (next.vxMetersPerSecond - current.vxMetersPerSecond) / dtSeconds;
    double ay = (next.vyMetersPerSecond - current.vyMetersPerSecond) / dtSeconds;
    double alpha = (next.omegaRadiansPerSecond - current.omegaRadiansPerSecond) / dtSeconds;

    // Calculate feedforward voltages for each chassis axis.
    double translationVoltage =
      kVTranslation * next.vxMetersPerSecond
      + kATranslation * ax;

    double strafeVoltage =
      kVStrafe * next.vyMetersPerSecond
      + kAStrafe * ay;

    double rotationVoltage =
      kVRotation * next.omegaRadiansPerSecond
      + kARotation * alpha;


    // Get wheel space voltages for each axis.
    MecanumDriveWheelSpeeds translation =
      m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(1.0, 0.0, 0.0));

    MecanumDriveWheelSpeeds strafe =
      m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(0.0, 1.0, 0.0));

    MecanumDriveWheelSpeeds rotation =
      m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(0.0, 0.0, 1.0));


    // Add the contributions.
    return new MecanumDriveWheelVoltages(
      translation.frontLeftMetersPerSecond * translationVoltage
        + strafe.frontLeftMetersPerSecond * strafeVoltage
        + rotation.frontLeftMetersPerSecond * rotationVoltage,

      translation.frontRightMetersPerSecond * translationVoltage
        + strafe.frontRightMetersPerSecond * strafeVoltage
        + rotation.frontRightMetersPerSecond * rotationVoltage,

      translation.rearLeftMetersPerSecond * translationVoltage
        + strafe.rearLeftMetersPerSecond * strafeVoltage
        + rotation.rearLeftMetersPerSecond * rotationVoltage,

      translation.rearRightMetersPerSecond * translationVoltage
        + strafe.rearRightMetersPerSecond * strafeVoltage
        + rotation.rearRightMetersPerSecond * rotationVoltage
    );
  }
}