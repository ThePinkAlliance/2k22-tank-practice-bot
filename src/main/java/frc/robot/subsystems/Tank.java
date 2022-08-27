// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.core.rev.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tank extends SubsystemBase {
  SparkMax left_front;
  SparkMax left_back;
  SparkMax right_front;
  SparkMax right_back;

  TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(12, 15),
      new TrapezoidProfile.State());

  MotorControllerGroup left;
  MotorControllerGroup right;
  DifferentialDrive differentialDrive;

  /** Creates a new Tank. */
  public Tank() {
    this.left_front = new SparkMax(22, MotorType.kBrushed);
    this.left_back = new SparkMax(23, MotorType.kBrushed);
    this.right_front = new SparkMax(20, MotorType.kBrushed);
    this.right_back = new SparkMax(21, MotorType.kBrushed);

    // this.left_front.setInverted(true);

    this.left = new MotorControllerGroup(left_front, left_back);
    this.right = new MotorControllerGroup(right_back, right_front);

    this.right.setInverted(true);

    // this.differentialDrive = new DifferentialDrive(left, right);
  }

  public void drive(double left, double right) {
    this.left.set(left);
    this.right.set(right);
  }

  public void drive(double pwr) {
    drive(pwr, pwr);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // this.differentialDrive.check();
    // this.differentialDrive.checkMotors();
  }
}
