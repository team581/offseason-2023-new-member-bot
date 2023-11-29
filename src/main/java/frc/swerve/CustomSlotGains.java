// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;

public class CustomSlotGains extends Slot0Configs {
  public CustomSlotGains(double kP, double kI, double kD, double kV, double kS) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kV = kV;
    this.kS = kS;
  }
}
