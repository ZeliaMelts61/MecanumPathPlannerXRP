// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.event.PrintEvent;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.Constants.PathplannerConstants;
import frc.robot.Utils.SwappableController;
import frc.robot.commands.MecanumDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.commands.RslCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Rsl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPOnBoardIO;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import edu.wpi.first.wpilibj2.command.button.

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final XRPOnBoardIO m_onboardIO = new XRPOnBoardIO();
  private final Arm m_arm = new Arm();
  private final Rsl m_rsl = new Rsl(m_onboardIO);

  // Assumes a gamepad plugged into channel 0
  //private final Joystick m_controller = new Joystick(0);
  public final SwappableController m_controller = new SwappableController(0, this::configureButtonBindings);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    m_chooser=AutoBuilder.buildAutoChooser();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new RslCommand(m_rsl);
    //CommandScheduler.getInstance().schedule(new PrintCommand(String.valueOf(m_drivetrain.maxAccX)).repeatedly());
    
    m_arm.setAngle(80);
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getMecanumDriveCommand());

    // Example of how to use the onboard IO
    Trigger userButton = new Trigger(m_onboardIO::getUserButtonPressed);
    userButton
        .onTrue(new PrintCommand("USER Button Pressed"))
        .onFalse(new PrintCommand("USER Button Released"));

    m_controller.y()
        .onTrue(new InstantCommand (() -> m_drivetrain.resetAll()).ignoringDisable(true));

    m_controller.a()
        .whileTrue(Commands.run(()->m_arm.setAngle(45), m_arm))    
        //.onFalse(new InstantCommand(() -> m_arm.setAngle(0.0), m_arm).onlyIf(() -> m_arm.getCurrentCommand()==null));
        ;
    m_controller.b()
        .whileTrue(Commands.run(()->m_arm.setAngle(90), m_arm))
        //.onFalse(new InstantCommand(() -> m_arm.setAngle(0.0), m_arm).onlyIf(() -> m_arm.getCurrentCommand()==null));
        ;
    m_controller.leftTrigger(0.2)
        .whileTrue(Commands.run(()->m_arm.setPosition(m_controller.getLeftTriggerAxis()), m_arm))
        //.onFalse(new InstantCommand(() -> m_arm.setAngle(-45), m_arm).onlyIf(() -> m_arm.getCurrentCommand()==null));
        ;
    m_controller.leftBumper()
        .whileTrue(new InstantCommand(() -> m_arm.printPosition()).repeatedly().ignoringDisable(true));
    

    m_arm.setDefaultCommand(Commands.run(() -> m_arm.setAngle(0.0), m_arm));
    NamedCommands.registerCommand("arm up", Commands.run(() -> m_arm.setAngle(180), m_arm).andThen(Commands.run(()->System.out.println("e"))).repeatedly().ignoringDisable(true));
    //NamedCommands.registerCommand("arm up", Commands.run(() -> m_arm.setAngle(0.0), m_arm));
    // Setup SmartDashboard options
    m_chooser.setDefaultOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.addOption("Auto Routine Time", new AutonomousTime(m_drivetrain));
    //m_chooser.addOption("Pathplanner Test Auto", new PathPlannerAuto("New Auto"));
    //m_chooser.addOption("curve", new PathPlannerAuto("curve"));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getMecanumDriveCommand() {
    return new MecanumDrive(
        m_drivetrain, () -> -m_controller.getLeftY(), () -> -m_controller.getLeftX(), () -> -m_controller.getRightX());
  }
}
