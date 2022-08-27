// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Tank;

public class MoveStraight extends CommandBase {
  Tank m_drive;
  Timer m_timer;
  double m_time;

  /** Creates a new LeaveTarmac. */
  public MoveStraight(Tank m_drive, double m_time) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drive = m_drive;
    this.m_time = m_time;
    this.m_timer = new Timer();

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.drive(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time);
  }
}
