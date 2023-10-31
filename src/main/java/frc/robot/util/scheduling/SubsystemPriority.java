// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.scheduling;

public enum SubsystemPriority {
  SUPERSTRUCTURE_MANAGER(20),

  // Run autobalance and autorotate before swerve
  AUTOROTATE(11),
  AUTOBALANCE(11),

  WRIST(10),
  SWERVE(10),
  IMU(10),
  INTAKE(10),
  // Run localization after swerve & IMU
  LOCALIZATION(9),

  FMS(0),
  LIGHTS(0),
  RUMBLE_CONTROLLER(0);

  final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }
}
