// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ThePinkAlliance.core.joystick.JoystickAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.Climber;

public class JoystickClimber extends CommandBase {
  Climber m_climber;
  JoystickAxis m_axis;

  double MAX_POSITION = 260000;

  /** Creates a new JoystickClimber. */
  public JoystickClimber(Climber m_climber, JoystickAxis m_axis) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_axis = m_axis;
    this.m_climber = m_climber;

    addRequirements(m_climber);
  }

  public JoystickClimber(Climber m_climber, JoystickAxis m_axis, double m_limit) {

    this.m_axis = m_axis;
    this.m_climber = m_climber;
    this.MAX_POSITION = m_limit;

    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (Math.abs(m_climber.getTicks()) <= MAX_POSITION
    // || Math.abs(m_climber.getTicks()) >= MAX_POSITION && matchSign(m_axis.get(),
    // 1)
    // || matchSign(m_climber.getTicks(), 1)) {

    // this.m_climber.command(this.m_axis.get());
    // } else {
    // this.m_climber.command(0.55);
    // }

    this.m_climber.command(this.m_axis.get());

    SmartDashboard.putNumber("ticks", this.m_climber.getTicks());
  }

  public boolean matchSign(double input, double sign) {
    return input == Math.copySign(input, sign);
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
