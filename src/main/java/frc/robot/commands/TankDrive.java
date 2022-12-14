// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.ThePinkAlliance.core.joystick.JoystickAxis;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tank;

public class TankDrive extends CommandBase {
  Tank m_tank;
  // Supplier<Double> left_x;
  // Supplier<Double> right_x;

  JoystickAxis left_x;
  JoystickAxis right_x;

  double MAX_POWER = 1;

  /** Creates a new TankDrive. */
  public TankDrive(Tank m_tank, JoystickAxis left_x, JoystickAxis right_x) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.left_x = left_x;
    this.right_x = right_x;
    this.m_tank = m_tank;

    addRequirements(m_tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(left_x.get() + ", " + right_x.get());

    // 0.47
    m_tank.drive(left_x.get() * 0.47, right_x.get() * 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tank.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
