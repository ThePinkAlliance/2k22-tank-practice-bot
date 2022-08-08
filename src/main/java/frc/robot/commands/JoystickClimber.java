// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.core.joystick.JoystickAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class JoystickClimber extends CommandBase {
  Climber m_climber;
  JoystickAxis m_axis;

  /** Creates a new JoystickClimber. */
  public JoystickClimber(Climber m_climber, JoystickAxis m_axis) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_axis = m_axis;
    this.m_climber = m_climber;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.m_climber.getRotations() >= 0.2 && this.m_axis.get() == Math.copySign(this.m_axis.get(), 1)) {
      this.m_climber.command(this.m_axis.get());
    } else if (this.m_climber.getRotations() <= 0.2 && this.m_axis.get() == Math.copySign(this.m_axis.get(), -1)) {
      this.m_climber.command(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_climber.command(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
