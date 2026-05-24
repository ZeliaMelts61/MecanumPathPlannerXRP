// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PathplannerConstants;

public class Drivetrain extends SubsystemBase {
  public double maxAccX=0.0;

  private static final double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  //private final DifferentialDrive m_diffDrive =
      //new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  //Create kinematics

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH_IN_METERS);

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();


  //Create Odometry
  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(), m_rightEncoder.getDistance(),
      new Pose2d(0, 0, new Rotation2d()));

  private Pose2d m_pose = new Pose2d();

  private final Field2d field = new Field2d();

  //pids 
  private final PIDController leftPID = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private final PIDController rightPID = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV);

  private double lastTimeTargetPoseWasUpdated = Timer.getTimestamp();
  private ArrayList<Pose2d> pathPoses = new ArrayList<Pose2d>();
  private boolean hasRemovedTargetPoseAndPath = false;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    //field.getRobotObject().setTrajectory(new PathPlannerTrajectoryState().fieldSpeeds);
    leftPID.setTolerance(0.02);
    rightPID.setTolerance(0.02);
    //SendableRegistry.addChild(m_diffDrive, m_leftMotor);
    //SendableRegistry.addChild(m_diffDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    



    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
            PathplannerConstants.config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  private void updateKinematics(){
    m_leftEncoder.getRate();
    wheelSpeeds = new DifferentialDriveWheelSpeeds(
        Units.inchesToMeters(m_leftEncoder.getRate()),
        Units.inchesToMeters(m_rightEncoder.getRate()));
    chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    
  }
  public void resetAll(){
    resetPose(Pose2d.kZero);
    maxAccX=0.0;
    

  }

  private void updateOdometry(){
    //m_odometry.update(, null)
    m_pose = m_odometry.update(m_gyro.getRotation2d(),
        Units.inchesToMeters(m_leftEncoder.getDistance()),
        Units.inchesToMeters(m_rightEncoder.getDistance()));

    field.setRobotPose(m_pose);
    SmartDashboard.putData("field", field);

  
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
    // Send to Field2d widget
      if (!poses.isEmpty()){
        hasRemovedTargetPoseAndPath=false;
        lastTimeTargetPoseWasUpdated = Timer.getTimestamp();

        pathPoses.addAll(poses);
        field.getObject("active path trajectory").setPoses(pathPoses);
      }
      
      
    });
  
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      //System.out.println(targetPose);
      lastTimeTargetPoseWasUpdated = Timer.getTimestamp();
      field.getObject("targetPose").setPose(targetPose);
    });
    
    
    if (!(lastTimeTargetPoseWasUpdated+2>Timer.getTimestamp())&&!hasRemovedTargetPoseAndPath){

      hasRemovedTargetPoseAndPath=true;
      System.out.println(lastTimeTargetPoseWasUpdated);
      field.getObject("targetPose").setPoses();
      field.getObject("active path trajectory").setPoses();
      pathPoses.clear();
    }
    

  }
    
  
  /* 
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }
  */
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    
    
    
    double maxLinearSpeed = 0.7;
    double maxAngularSpeed = 7.50492;

    ChassisSpeeds speeds = new ChassisSpeeds(
        xaxisSpeed * maxLinearSpeed,
        0.0,
        zaxisRotate * maxAngularSpeed);

    DifferentialDriveWheelSpeeds wheelSpeeds =
        kinematics.toWheelSpeeds(speeds);

    wheelSpeeds.desaturate(maxLinearSpeed);

    setWheelSpeeds(wheelSpeeds);
    
  }

  
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to individual wheel speeds
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    // Apply speeds to your motor controllers (e.g., using velocity PID)
    // Left: wheelSpeeds.leftMetersPerSecond
    // Right: wheelSpeeds.rightMetersPerSecond
    this.setWheelSpeeds(wheelSpeeds);
  }
    
    
  private void setWheelSpeeds(DifferentialDriveWheelSpeeds targetSpeeds) {

    double leftSpeed =
        Units.inchesToMeters(m_leftEncoder.getRate());

    double rightSpeed =
        Units.inchesToMeters(m_rightEncoder.getRate());

    double leftFF =
        feedforward.calculate(targetSpeeds.leftMetersPerSecond);

    double rightFF =
        feedforward.calculate(targetSpeeds.rightMetersPerSecond);

    double leftPIDOut =
        leftPID.calculate(
            leftSpeed,
            targetSpeeds.leftMetersPerSecond);

    double rightPIDOut =
        rightPID.calculate(
            rightSpeed,
            targetSpeeds.rightMetersPerSecond);

    double leftOutput = leftFF + leftPIDOut;
    double rightOutput = rightFF + rightPIDOut;

    leftOutput = MathUtil.clamp(leftOutput, -1.0, 1.0);
    rightOutput = MathUtil.clamp(rightOutput, -1.0, 1.0);

    m_leftMotor.set(leftOutput);
    m_rightMotor.set(rightOutput);
  }
    
  public Pose2d getPose(){
    return m_pose;
  }

  public void resetPose(Pose2d newPose) {

    resetEncoders();

    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        Units.inchesToMeters(m_leftEncoder.getDistance()),
        Units.inchesToMeters(m_rightEncoder.getDistance()),
        newPose);

    m_pose = newPose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return chassisSpeeds;
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the XRP along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the XRP along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the XRP along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the XRP around the X-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the XRP around the Y-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the XRP around the Z-axis.
   *
   * @return The current angle of the XRP in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateKinematics();
    updateOdometry();
  }

  
}
