// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.core.rev.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tank extends SubsystemBase {
  SparkMax left_front;
  SparkMax left_back;
  SparkMax right_front;
  SparkMax right_back;

  TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(12, 15),
      new TrapezoidProfile.State());

  /** Creates a new Tank. */
  public Tank() {
    this.left_front = new SparkMax(0, MotorType.kBrushed);
    this.left_back = new SparkMax(1, MotorType.kBrushed);
    this.right_front = new SparkMax(2, MotorType.kBrushed);
    this.right_back = new SparkMax(3, MotorType.kBrushed);
  }

  public void drive(double left, double right) {
    this.left_back.set(left);
    this.left_front.set(left);
    this.right_back.set(right);
    this.right_front.set(right);
  }

  public void drive(double pwr) {
    drive(pwr, pwr);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
