// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoKindWithoutTeam {
  DO_NOTHING(AutoKind.DO_NOTHING, AutoKind.DO_NOTHING),
  MID_1_BALANCE(AutoKind.RED_MID_1_BALANCE, AutoKind.BLUE_MID_1_BALANCE),
  MID_1(AutoKind.RED_MID_1, AutoKind.BLUE_MID_1),
  FLAT_2(AutoKind.RED_FLAT_2, AutoKind.BLUE_FLAT_2),
  // FLAT_3(AutoKind.RED_FLAT_3, AutoKind.BLUE_FLAT_3),
  BUMP_2(AutoKind.RED_BUMP_2, AutoKind.BLUE_BUMP_2),
  BUMP_3(AutoKind.RED_BUMP_3, AutoKind.BLUE_BUMP_3);

  public final AutoKind redVersion;
  public final AutoKind blueVersion;

  private AutoKindWithoutTeam(AutoKind redVersion, AutoKind blueVersion) {
    this.redVersion = redVersion;
    this.blueVersion = blueVersion;
  }
}
