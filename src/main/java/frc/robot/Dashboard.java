// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Dashboard {
  private static boolean debugEnabled = true;

  public static ShuffleboardTab debug = Shuffleboard.getTab("debug");

  public static void sendObject(ShuffleboardTab tab, String name, Object object) {
    NetworkTableInstance.getDefault().getTable(tab.getTitle()).getEntry(name).setValue(object);
  }

  public static void setDebugEnabled(boolean mode) {
    debugEnabled = mode;
  }

  public static void motorDebugInfo(WPI_TalonFX motor) {
    if (debugEnabled) {
      SmartDashboard.putNumber(buildName(motor, "Voltage"), motor.getBusVoltage());
      SmartDashboard.putNumber(buildName(motor, "Ticks"), motor.getSelectedSensorPosition());
      SmartDashboard.putNumber(buildName(motor, "Velocity"), motor.getSelectedSensorVelocity());
      SmartDashboard.putNumber(buildName(motor, "Output"), motor.getMotorOutputPercent());
      SmartDashboard.putNumber(buildName(motor, "Current Supply"), motor.getSupplyCurrent());
      SmartDashboard.putNumber(buildName(motor, "Temperature"), motor.getTemperature());
    }
  }

  private static String buildName(WPI_TalonFX motor, String tag) {
    return "TalonFX: [" + motor.getDeviceID() + "]: " + tag;
  }
}
