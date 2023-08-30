// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

public class Config {
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;

  public static final String CANIVORE_ID = "581CANivore";

  private Config() {}
}
