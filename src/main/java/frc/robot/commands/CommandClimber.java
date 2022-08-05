// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class CommandClimber extends CommandBase {
  Climber m_climber;
  double m_goal;
  ProfiledPIDController m_controller = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.1, 0.2));

  /** Creates a new CommandClimber. */
  public CommandClimber(Climber m_climber, double m_goal) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_climber = m_climber;
    this.m_goal = m_goal;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.m_controller.disableContinuousInput();
    this.m_controller.setGoal(m_goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotations = m_climber.getRotations();
    double next = this.m_controller.calculate(rotations);

    this.m_climber.command(next);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_climber.command(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_controller.atGoal();
  }
}
