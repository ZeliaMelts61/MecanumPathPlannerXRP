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
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PathplannerConstants;

public class Drivetrain extends SubsystemBase {
  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  // It also sets motors "3" and "4" to channels 2 and 3 respectively
  private final XRPMotor m_frontLeftMotor  = new XRPMotor(0);
  private final XRPMotor m_frontRightMotor = new XRPMotor(1);
  private final XRPMotor m_backLeftMotor   = new XRPMotor(2);
  private final XRPMotor m_backRightMotor  = new XRPMotor(3);
  




  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  // as well as DIO pins 8/9 and 10/11 for motors 3 and 4
  private final Encoder m_frontLeftEncoder = new Encoder(4, 5);
  private final Encoder m_frontRightEncoder = new Encoder(6, 7);
  private final Encoder m_backLeftEncoder = new Encoder(8, 9);
  private final Encoder m_backRightEncoder = new Encoder(10, 11);

  // Set up the differential drive controller
  private final MecanumDrive m_mecanumDrive = new MecanumDrive(
    m_frontLeftMotor::set, m_frontRightMotor::set,
    m_backLeftMotor::set, m_backRightMotor::set
);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  //Create kinematics
  private final MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
      DrivetrainConstants.kFrontLeftLocation,
      DrivetrainConstants.kFrontRightLocation,
      DrivetrainConstants.kBackLeftLocation,
      DrivetrainConstants.kBackRightLocation
  );

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0, 0, 0);

  private MecanumDriveWheelSpeeds m_wheelSpeeds = new MecanumDriveWheelSpeeds();

  private MecanumDriveWheelPositions m_wheelPositions = new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
      m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
  );


  //Create Odometry
  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      m_wheelPositions
  );

  private Pose2d m_pose = new Pose2d();

  private final Field2d m_field = new Field2d();

  //pids 
  private final PIDController m_drivePID = new PIDController(DrivetrainConstants.kP, DrivetrainConstants.kI, DrivetrainConstants.kD);
  
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DrivetrainConstants.kS, DrivetrainConstants.kV);


  // stuff for displaying path in elastic
  private double m_lastTimeTargetPoseWasUpdated = Timer.getTimestamp();
  private ArrayList<Pose2d> m_pathPoses = new ArrayList<Pose2d>();
  private boolean m_hasRemovedTargetPoseAndPath = false;

  

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    for(String key: SmartDashboard.getKeys()){
      SmartDashboard.clearPersistent(key);
  }

    //this sets the tolerance for the pid controllers so that they don't wiggle at low speeds
    m_drivePID.setTolerance(0.02);


    SendableRegistry.addChild(m_mecanumDrive, m_frontLeftMotor);
    SendableRegistry.addChild(m_mecanumDrive, m_frontRightMotor);
    SendableRegistry.addChild(m_mecanumDrive, m_backLeftMotor);
    SendableRegistry.addChild(m_mecanumDrive, m_backRightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRightMotor.setInverted(true);
    m_backRightMotor.setInverted(true);

    // Use METERS as unit for encoder distances (this is different from sample XRP code)
    double distancePerPulse = (Math.PI * DrivetrainConstants.kWheelDiameterMeter) / DrivetrainConstants.kCountsPerRevolution;
    m_frontLeftEncoder.setDistancePerPulse(distancePerPulse);
    m_frontRightEncoder.setDistancePerPulse(distancePerPulse);
    m_backLeftEncoder.setDistancePerPulse(distancePerPulse);
    m_backRightEncoder.setDistancePerPulse(distancePerPulse);

    resetEncoders();

    //SmartDashboard.putData("Translation PID", DrivetrainConstants.kTranslationPID);


    // configure autobuilder for pathplanner

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> mecanumDriveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            PathplannerConstants.kTranslationPID, // Translation PID constants
            PathplannerConstants.kRotationPID // Rotation PID constants
        ),
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
    m_wheelSpeeds = new MecanumDriveWheelSpeeds(
        m_frontLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_backLeftEncoder.getRate(),
        m_backRightEncoder.getRate()
    );
    m_chassisSpeeds = m_kinematics.toChassisSpeeds(m_wheelSpeeds);
  }


  private void updateOdometry(){
    //m_odometry.update(, null)

    m_wheelPositions = new MecanumDriveWheelPositions(
        m_frontLeftEncoder.getDistance(), m_frontRightEncoder.getDistance(),
        m_backLeftEncoder.getDistance(), m_backRightEncoder.getDistance()
    );
    m_pose = m_odometry.update(m_gyro.getRotation2d(), m_wheelPositions);

    m_field.setRobotPose(m_pose);
    SmartDashboard.putData("field", m_field);
  }


  public ChassisSpeeds getFieldChassisSpeeds(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(m_chassisSpeeds, m_gyro.getRotation2d());
  }

  public ChassisSpeeds getRobotChassisSpeeds(){
    return m_chassisSpeeds;
  }
 

  public void createDashboardWidgets(){
    SmartDashboard.putData("Mecanum", builder -> {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> 45, null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_wheelSpeeds.frontLeftMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> -45, null);
        builder.addDoubleProperty("Front Right Velocity", () -> 8, null);

        builder.addDoubleProperty("Back Left Angle", () -> 135, null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_wheelSpeeds.rearLeftMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> 225, null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_wheelSpeeds.rearRightMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> m_gyro.getRotation2d().getRadians(), null);
    });

    SmartDashboard.putData("Drive Pid", m_drivePID);
  }

  public void updateDashboardWidgets(){
    double velocity = Math.hypot(Math.abs(m_chassisSpeeds.vxMetersPerSecond),Math.abs(m_chassisSpeeds.vyMetersPerSecond));
    SmartDashboard.putNumber("Linear Speed", velocity);
    SmartDashboard.putNumber("Rotational Speed", Math.abs(m_chassisSpeeds.omegaRadiansPerSecond));
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putBoolean("Controller Connected", DriverStation.isJoystickConnected(Constants.OperatorConstants.kDriverControllerPort));
    SmartDashboard.putBoolean("Robot Connected", DriverStation.isDSAttached());
    SmartDashboard.putBoolean("Robot is About to explode", velocity>DrivetrainConstants.kMaxLinearXSpeedMPS);
    RoboRioSim.getVInVoltage();
    SmartDashboard.putNumber("Voltage?",RoboRioSim.getVInVoltage());
    

  }

  /* Updates the Path and TargetPose on the field widget on the dashboard */
  public void updatePathOnDashboard(){

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
    // Send to Field2d widget
      if (!poses.isEmpty()){
        m_hasRemovedTargetPoseAndPath=false;
        m_lastTimeTargetPoseWasUpdated = Timer.getTimestamp();

        m_pathPoses.addAll(poses);
        m_field.getObject("active path trajectory").setPoses(m_pathPoses);
      }
      
      
    });
  
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
      //System.out.println(targetPose);
      m_lastTimeTargetPoseWasUpdated = Timer.getTimestamp();
      m_field.getObject("targetPose").setPose(targetPose);
    });
    
    
    if (!(m_lastTimeTargetPoseWasUpdated+2>Timer.getTimestamp())&&!m_hasRemovedTargetPoseAndPath){
      m_hasRemovedTargetPoseAndPath=true;
      System.out.println(m_lastTimeTargetPoseWasUpdated);
      m_field.getObject("targetPose").setPoses();
      m_field.getObject("active path trajectory").setPoses();
      m_pathPoses.clear();
    }
    SmartDashboard.putData("Drive", m_mecanumDrive);
  }
    
  public void resetAll(){
    resetPose(Pose2d.kZero);
  }
  
  /**
   * Drive Robot Relative using mecanum drive, uses Pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   *
   * @param xAxisSpeed Desired Speed for the robot in the X Axis (-1.0 to 1.0).
   * @param yAxisSpeed Desired Speed for the robot in the Y Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0).
   */
  public void mecanumDriveRobotRelative(double xAxisSpeed, double yAxisSpeed, double zAxisRotate) {
    ChassisSpeeds speeds = new ChassisSpeeds(
        xAxisSpeed * DrivetrainConstants.kMaxLinearXSpeedMPS,
        yAxisSpeed * DrivetrainConstants.kMaxLinearYSpeedMPS,
        zAxisRotate * DrivetrainConstants.kMaxAngularSpeedRPS);
    mecanumDriveRobotRelative(speeds);
  }

  /**
   * Drive Robot Relative using arcade drive, uses Pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   *
   * @param xAxisSpeed Desired Speed for the robot in the X Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0).
   */
  public void arcadeDrive(double xAxisSpeed, double zAxisRotate) {
    ChassisSpeeds speeds = new ChassisSpeeds(
        xAxisSpeed * DrivetrainConstants.kMaxLinearXSpeedMPS,
        0,
        zAxisRotate * DrivetrainConstants.kMaxAngularSpeedRPS);
    mecanumDriveRobotRelative(speeds);
  }
  

  /**
   * Drive Robot Relative using mecanum drive, Uses pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   * @param speeds The ChassisSpeeds you want the robot to move at. 
   */
  public void mecanumDriveRobotRelative(ChassisSpeeds speeds) {
    // Convert ChassisSpeeds to individual wheel speeds
    MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    // Apply speeds to your motor controllers (e.g., using velocity PID)
    wheelSpeeds.desaturate(DrivetrainConstants.kMaxLinearYSpeedMPS);
    this.setWheelSpeeds(wheelSpeeds);
  }


  /**
   * Drive Field Relative using mecanum drive, Uses pids for more accurate control.
   * It will desaturate the wheel speeds automatically
   * @param xAxisSpeed Desired Speed for the robot in the field X Axis (-1.0 to 1.0).
   * @param yAxisSpeed Desired Speed for the robot in the field Y Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0). 
   * @param gyroAngle The rotation 2d of the robot.
   */
  public void mecanumDriveFieldRelative(double xAxisSpeed, double yAxisSpeed, double zAxisRotate, Rotation2d gyroAngle) {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxisSpeed * DrivetrainConstants.kMaxLinearXSpeedMPS,
            yAxisSpeed * DrivetrainConstants.kMaxLinearYSpeedMPS,
            zAxisRotate * DrivetrainConstants.kMaxAngularSpeedRPS,
            gyroAngle);

    mecanumDriveRobotRelative(speeds);
  }

  /**
   * Drive Field Relative using mecanum drive.
   * It uses the built in gyro to calculate robot rotation.
   * Uses pids for more accurate control.
   * It will desaturate the wheel speeds automatically.
   * @param xAxisSpeed Desired Speed for the robot in the field X Axis (-1.0 to 1.0).
   * @param yAxisSpeed Desired Speed for the robot in the field Y Axis (-1.0 to 1.0).
   * @param zAxisRotate Desired rotation Speed for the robot (-1.0 to 1.0). 
   */
  public void mecanumDriveFieldRelative(double xAxisSpeed, double yAxisSpeed, double zAxisRotate) {
    mecanumDriveFieldRelative(xAxisSpeed, yAxisSpeed, zAxisRotate, m_gyro.getRotation2d());
  }
    

  /**
   * Sets the mecanum wheel speeds using closed-loop velocity control.
   * 
   * <p>This method:
   * <ul>
   *   <li>Desaturates wheel speeds to the drivetrain maximum velocity.</li>
   *   <li>Uses PID controllers to correct wheel velocity error.</li>
   *   <li>Uses feedforward to improve velocity tracking accuracy.</li>
   *   <li>Combines PID and feedforward outputs for final motor commands.</li>
   *   <li>Clamps motor outputs to the valid range of -1.0 to 1.0.</li>
   * </ul>
   * 
   * <p>The supplied wheel speeds are expected to be in meters per second.
   * 
   * @param targetSpeeds Desired wheel speeds for each mecanum wheel.
  */
  private void setWheelSpeeds(MecanumDriveWheelSpeeds targetSpeeds) {
    targetSpeeds.desaturate(DrivetrainConstants.kMaxLinearXSpeedMPS);
    double frontLeftPIDOut =
        m_drivePID.calculate(
            m_wheelSpeeds.frontLeftMetersPerSecond,
            targetSpeeds.frontLeftMetersPerSecond);
    double frontRightPIDOut = 
        m_drivePID.calculate(
                m_wheelSpeeds.frontRightMetersPerSecond,
                targetSpeeds.frontRightMetersPerSecond);
    double backLeftPIDOut =
        m_drivePID.calculate(
            m_wheelSpeeds.rearLeftMetersPerSecond,
            targetSpeeds.rearLeftMetersPerSecond);
    double backRightPIDOut =
        m_drivePID.calculate(
            m_wheelSpeeds.rearRightMetersPerSecond,
            targetSpeeds.rearRightMetersPerSecond);
    
    double frontLeftOutput = MathUtil.clamp(m_feedforward.calculate(targetSpeeds.frontLeftMetersPerSecond) + frontLeftPIDOut, -1.0, 1.0);
    double frontRightOutput = MathUtil.clamp(m_feedforward.calculate(targetSpeeds.frontRightMetersPerSecond) + frontRightPIDOut, -1.0, 1.0);
    double backLeftOutput = MathUtil.clamp(m_feedforward.calculate(targetSpeeds.rearLeftMetersPerSecond) + backLeftPIDOut, -1.0, 1.0);
    double backRightOutput = MathUtil.clamp(m_feedforward.calculate(targetSpeeds.rearRightMetersPerSecond) + backRightPIDOut, -1.0, 1.0);

    m_frontLeftMotor.set(frontLeftOutput);
    m_frontRightMotor.set(frontRightOutput);
    m_backLeftMotor.set(backLeftOutput);
    m_backRightMotor.set(backRightOutput);
  }

  /**
   * Stops the drivetrain by setting all motor powers to 0.
   */
  public void stop(){
    m_frontLeftMotor.set(0);
    m_frontRightMotor.set(0);
    m_backLeftMotor.set(0);
    m_backRightMotor.set(0);
  }
    

  /**
   * Gets the current robot pose
   * @return The current pose2d of the robot.
   */
  public Pose2d getPose(){
    return m_pose;
  }

  /**
   * Gets the field from the Drive system
   * @return The Field
   */
  public Field2d getField(){
    return m_field;
  }

  /**
   * Resets the Robot's pose to the supplied pose
   * @param newPose the pose the robot will get set to
   */
  public void resetPose(Pose2d newPose) {
    resetEncoders();
    //updateOdometry();

    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        m_wheelPositions,
        newPose);

    m_pose = newPose;
  }

  /**
   * Resets the Robot's pose to Pose2d.kZero
   */
  public void resetPose() {
    resetPose(Pose2d.kZero);
  }  

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return m_chassisSpeeds;
  }

  /**
   * Resets all the Drive Encoder distances to zero. 
   * Resets all of the Drive Encoder current counts to zero.
   */
  public void resetEncoders() {
    m_frontLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_backLeftEncoder.reset();
    m_backRightEncoder.reset();
  }

  /**
   * Gets the current count of the Front Left Encoder.
   * Returns the current count on the Front Left Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Front Left Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getFrontLeftEncoderCount() {
    return m_frontLeftEncoder.get();
  }

  /**
   * Gets the current count of the Front Right Encoder.
   * Returns the current count on the Front Right Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Front Right Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getFrontRightEncoderCount() {
    return m_frontRightEncoder.get();
  }

  /**
   * Gets the current count of the Back Left Encoder.
   * Returns the current count on the Back Left Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Back Left Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getBackLeftEncoderCount() {
    return m_backLeftEncoder.get();
  }

  /**
   * Gets the current count of the Back Right Encoder.
   * Returns the current count on the Back Right Encoder.
   * This method compensates for the decoding type.
   * @return Current count from the Back Right Encoder adjusted for the 1x, 2x, or 4x scale factor.
   */
  public int getBackRightEncoderCount() {
    return m_backRightEncoder.get();
  }

  
  /**
  * Gets the distance the Front Left wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getFrontLeftDistanceMeters() {
    return m_frontLeftEncoder.getDistance();
  }

  /**
  * Gets the distance the Front Right wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getFrontRightDistanceMeters() {
    return m_frontRightEncoder.getDistance();
  }

  /**
  * Gets the distance the Back Left wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getBackLeftDistanceMeters() {
    return m_backLeftEncoder.getDistance();
  }

  /**
  * Gets the distance the Back Right wheel has driven (in meters) since the last reset.
  * @return The distance in meters driven since the last reset
  */
  public double getBackRightDistanceMeters() {
    return m_backRightEncoder.getDistance();
  }

  /**
  * Gets the average distance the left wheels have driven (in meters) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageLeftDistanceMeters() {
    return (getFrontLeftDistanceMeters() + getBackLeftDistanceMeters()) / 2;
  }

  /**
  * Gets the average distance the left wheels have driven (in inches) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getLeftDistanceInch() {
    return Units.metersToInches(getAverageLeftDistanceMeters());
  }

  /**
  * Gets the average distance the right wheels have driven (in inches) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getRightDistanceInch() {
    return Units.metersToInches(getAverageRightDistanceMeters());
  }

  /**
  * Gets the average distance the right wheels have driven (in meters) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageRightDistanceMeters() {
    return (getFrontRightDistanceMeters() + getBackRightDistanceMeters()) / 2;
  }

  /**
  * Gets the average distance the robot has driven (in meters) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageDistanceMeters() {
    return (getAverageLeftDistanceMeters() + getAverageRightDistanceMeters()) / 2.0;
  }

  /**
  * Gets the average distance the robot has driven (in inches) since the last reset.
  * @return The average distance in meters driven since the last reset
  */
  public double getAverageDistanceInch() {
    return Units.metersToInches(getAverageDistanceMeters());
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
    updatePathOnDashboard();
    updateDashboardWidgets();
  }

  
}
