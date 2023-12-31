// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class Positions {
  public static final Rotation2d INTAKING = Rotation2d.fromDegrees(190);
  public static final Rotation2d OUTTAKING_STOWED = Rotation2d.fromDegrees(2);
  public static final Rotation2d OUTTAKING_LOW = Rotation2d.fromDegrees(160);
  public static final Rotation2d OUTTAKING_MID = Rotation2d.fromDegrees(105);
  public static final Rotation2d STOWED = Rotation2d.fromDegrees(2);
  public static final Rotation2d SHOOT_ON_HIGH = Rotation2d.fromDegrees(22);
  public static final Rotation2d SHOOT_ON_HIGH_FOLLOW_THRU = Rotation2d.fromDegrees(110);
  public static final Rotation2d SHOOT_ON_MID = Rotation2d.fromDegrees(30);
  public static final Rotation2d SHOOT_ON_MID_FOLLOW_THRU = Rotation2d.fromDegrees(110);

  private Positions() {}
}
