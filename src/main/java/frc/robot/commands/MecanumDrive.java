// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.*;

public class MecanumDrive extends Command {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_yaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;

  /**
   * Creates a new MecanumDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public MecanumDrive(
      Drivetrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> yaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_yaxisSpeedSupplier = yaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    double xSpeed = MathUtil.applyDeadband(m_xaxisSpeedSupplier.get(), OperatorConstants.kDeadband);
    double ySpeed = MathUtil.applyDeadband(m_yaxisSpeedSupplier.get(), OperatorConstants.kDeadband);
    double rotSpeed = MathUtil.applyDeadband(m_zaxisRotateSupplier.get(), OperatorConstants.kDeadband);
    SmartDashboard.putNumber("JoystickXSpeed", xSpeed);
    SmartDashboard.putNumber("JoystickYSpeed", ySpeed);
    SmartDashboard.putNumber("JoystickZSpeed", rotSpeed);
    //System.out.println(xSpeed);
    m_drivetrain.mecanumDriveRobotRelative(xSpeed, ySpeed, rotSpeed);
    //m_drivetrain.mecanumDriveNoPid(xSpeed, ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
