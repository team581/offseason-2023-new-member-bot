// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveController extends CommandXboxController {
  private boolean slowModeEnabled;

  public DriveController(int port) {
    super(port);
  }

  /** Scale a joystick value. */
  private double joystickScale(double x) {
    if (Math.abs(x) < 0.075) {
      return 0;
    }

    double value = Math.signum(x) * Math.pow(x, 2);

    if (slowModeEnabled) {
      return value / 2.0;
    } else {
      return value;
    }
  }

  public void slowModeToggle(boolean enabled) {
    slowModeEnabled = enabled;
  }

  /** The rotation across the robot's x-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getSidewaysPercentage() {
    return joystickScale(getLeftX());
  }

  /** The translation across the robot's y-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getForwardPercentage() {
    return joystickScale(-1 * getLeftY());
  }

  /** The rotation about the robot's z-axis as a percentage (<code>-1 <= x <= 1</code>) */
  public double getThetaPercentage() {
    return joystickScale(getRightX());
  }
}
