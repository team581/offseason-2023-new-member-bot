// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),
  BLUE_MID_1_BALANCE("BlueMid1Balance", 4, 3, true),
  RED_MID_1_BALANCE("RedMid1Balance", 4, 3, true),

  BLUE_MID_1("BlueMid1", 4, 3,false ),
  RED_MID_1("RedMid1", 4, 3, false),

  BLUE_FLAT_2("BlueFlat2", 4, 3,false ),
  RED_FLAT_2("RedFlat2", 4, 3, false),

  BLUE_FLAT_3("BlueFlat3", 4, 3, false),


  BLUE_BUMP_2("BlueBump2", 4,3, false),


  BLUE_BUMP_3("BlueBump3", 4, 3, false),
  RED_BUMP_3("RedBump3", 4, 3, false);

  public final String pathName;
  public final PathConstraints constraints;
  public final boolean autoBalance;

  private AutoKind(
      String pathName, double maxVelocity, double maxAcceleration, boolean autoBalance) {
    this.pathName = pathName;
    this.constraints = new PathConstraints(maxVelocity, maxAcceleration);
    this.autoBalance = autoBalance;
  }
}
