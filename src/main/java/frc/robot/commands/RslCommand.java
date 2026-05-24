// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Rsl;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RslCommand extends Command {
  Rsl m_rslSubsystem;


  /** Creates a new RSLCommand. */
  public RslCommand(Rsl rslSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_rslSubsystem=rslSubsystem;
    //addRequirements(rslSubsystem);
    //m_rslSubsystem.setDefaultCommand(this);
    m_rslSubsystem.turnOn();
    initialize();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    WrapperCommand flashloop = new RepeatCommand(Commands.waitSeconds(0.2).andThen(new InstantCommand(m_rslSubsystem::toggle, m_rslSubsystem))).beforeStarting(m_rslSubsystem::turnOn, m_rslSubsystem).ignoringDisable(false).withName("RSL Blink");
    WrapperCommand rslOn = new InstantCommand(m_rslSubsystem::turnOn, m_rslSubsystem).ignoringDisable(true).withName("RSL solid");
    Trigger robotEnableTrigger = new Trigger(() -> (RobotState.isEnabled() || RobotState.isAutonomous()));
    //Trigger robotDisableTrigger = new Trigger(() -> (RobotState.))
    //CommandScheduler.getInstance().schedule(rslOn);
    robotEnableTrigger
        .onTrue(flashloop)
        .onFalse(rslOn);

    m_rslSubsystem.turnOn();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
