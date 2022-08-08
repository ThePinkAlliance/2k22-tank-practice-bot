// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ThePinkAlliance.core.ctre.talon.TalonUtils;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  WPI_TalonFX motor;

  double gearRatio = 10.61;

  Matrix<N1, N1> MODEL_ACCURACY = VecBuilder.fill(1);
  Matrix<N1, N1> ENCODER_ACCURACY = VecBuilder.fill(0.01);
  double LOOP_TIME = 0.025;

  LinearSystemSim<N2, N1, N2> sim;
  BatterySim batterySim;

  final double POWER_LIMIT = 0.1;
  final double MAX_ROTATIONS = 100;

  /** Creates a new Climbers. */
  public Climber(int talonId) {
    this.motor = new WPI_TalonFX(talonId);
    this.sim = new LinearSystemSim<>(
        LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500(1), Units.lbsToKilograms(41), gearRatio));
  }

  public void command(double pwr) {
    if (getRotations() < MAX_ROTATIONS || getRotations() >= MAX_ROTATIONS && pwr == Math.copySign(pwr, -1)) {
      this.motor.set(ControlMode.PercentOutput, pwr * POWER_LIMIT);
    } else {
      this.motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public double getRotations() {
    return (this.motor.getSelectedSensorPosition() / TalonUtils.FULL_TALON_ROTATION_TICKS) / gearRatio;
  }

  public double getTicks() {
    return this.motor.getSelectedSensorPosition();
  }

  public double getPowerLimit() {
    return this.POWER_LIMIT;
  }

  public double getMaxRotations() {
    return this.MAX_ROTATIONS;
  }

  @Override
  public void simulationPeriodic() {
    sim.setInput(RobotController.getBatteryVoltage());
  }
}
