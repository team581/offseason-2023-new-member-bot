// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class Positions {
  public static final Rotation2d INTAKING = Rotation2d.fromDegrees(180);
  public static final Rotation2d OUTTAKING_LOW = Rotation2d.fromDegrees(0);
  public static final Rotation2d OUTTAKING_MID = Rotation2d.fromDegrees(90);
  public static final Rotation2d STOWED = Rotation2d.fromDegrees(2);

  private Positions() {}
}