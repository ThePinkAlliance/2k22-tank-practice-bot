// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ThePinkAlliance.core.joystick.Joystick;
import com.ThePinkAlliance.core.joystick.JoystickAxis;
import com.ThePinkAlliance.core.joystick.Joystick.Axis;
import com.ThePinkAlliance.core.joystick.Joystick.Buttons;
import com.ThePinkAlliance.core.pathweaver.PathChooser;
import com.ThePinkAlliance.core.selectable.Selectable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.CommandClimber;
import frc.robot.commands.JoystickClimber;
import frc.robot.commands.TankDrive;
import frc.robot.commands.auto.MoveStraight;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Tank;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick baseJS = new Joystick(0);
  private final Joystick towerJS = new Joystick(1);

  private JoystickAxis left_y = new JoystickAxis(baseJS, Joystick.Axis.LEFT_Y);

  private JoystickAxis right_y = new JoystickAxis(baseJS, Joystick.Axis.RIGHT_Y);
  private JoystickAxis tower_right_x = new JoystickAxis(towerJS, Joystick.Axis.RIGHT_X);

  private SendableChooser<Command> m_selected_auto = new SendableChooser<>();

  private final Tank m_tank = new Tank();

  private final Climber m_climber_left = new Climber(30);
  private final Climber m_climber_right = new Climber(31);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSubsystems();

    // Configure the button bindings
    configureButtonBindings();

    // Configure the dashboard for operators.
    configureDashboard();
  }

  public void configureSubsystems() {
    this.m_climber_right.resetEncoder();
  }

  public void configureDashboard() {
    m_selected_auto.setDefaultOption("Leave Tarmac", new MoveStraight(m_tank, 3.3));
    m_selected_auto.addOption("Do Nothing", new InstantCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_tank.setDefaultCommand(new TankDrive(m_tank,
        baseJS.getAxis(Joystick.Axis.LEFT_Y),
        baseJS.getAxis(Joystick.Axis.RIGHT_Y)));

    m_climber_left.setDefaultCommand(new JoystickClimber(m_climber_left,
        towerJS.getAxis(Axis.LEFT_Y)));
    m_climber_right.setDefaultCommand(new JoystickClimber(m_climber_right,
        towerJS.getAxis(Axis.RIGHT_Y)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Resolves the selected command that will run in autonomous
    return m_selected_auto.getSelected();
  }
}
