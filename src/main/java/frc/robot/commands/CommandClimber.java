// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class CommandClimber extends CommandBase {
  Climber m_climber;
  Watchdog m_watchdog;

  double rotations;

  /** Creates a new CommandClimber. */
  public CommandClimber(Climber m_climber, double rotations) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.rotations = rotations;
    this.m_climber = m_climber;

    this.m_watchdog = new Watchdog(5, () -> {
      m_climber.command(0);

      System.out.println("[WATCHDOG]: TERMINATED CLIMBER, TARGET ROTATIONS: " + rotations + ", WATCHDOG LIMIT: "
          + m_watchdog.getTimeout());
    });

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_climber.command(Math.copySign(m_climber.getPowerLimit(), rotations));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_climber.command(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.copySign(m_climber.getRotations(), rotations) >= rotations || m_watchdog.isExpired();
  }
}
