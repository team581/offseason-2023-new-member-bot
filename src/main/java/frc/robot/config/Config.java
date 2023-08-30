// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.localization.VisionMode;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModuleConstants;

public class Config {
  private static final String TYKE_SERIAL_NUMBER = "031617f6";

  public static final String SERIAL_NUMBER = System.getenv("serialnum");
  public static final boolean IS_DEVELOPMENT = true;

  public static final double MATCH_DURATION_TELEOP = 135;

  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final String CANIVORE_ID = "581CANivore";

  public static final int PDP_ID = 0;
  public static final ModuleType PDP_TYPE =  ModuleType.kCTRE;

  public static final VisionMode VISION_MODE = VisionMode.ENABLED_UNUSED;

  public static final int PIGEON_ID = 1;

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.84);
  public static final double SWERVE_STEER_GEARING_REDUCTION = 12.8;
  public static final double SWERVE_DRIVE_GEARING_REDUCTION = 8.14;

  public static final Translation2d SWERVE_FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
  public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
  public static final Translation2d SWERVE_BACK_LEFT_LOCATION =
 new Translation2d(-0.381, 0.381);
  public static final Translation2d SWERVE_BACK_RIGHT_LOCATION =
   new Translation2d(-0.381, -0.381);

  public static final int SWERVE_FL_DRIVE_MOTOR_ID = 8;
  public static final int SWERVE_FL_STEER_MOTOR_ID = 9;
  public static final int SWERVE_FL_CANCODER_ID = 13;
  public static final SwerveModuleConstants SWERVE_FL_CONSTANTS =
      new SwerveModuleConstants(
              Rotation2d.fromDegrees(117.19), SwerveCorner.FRONT_LEFT, false, false);
  public static final int SWERVE_FR_DRIVE_MOTOR_ID = 6;
  public static final int SWERVE_FR_STEER_MOTOR_ID = 7;
  public static final int SWERVE_FR_CANCODER_ID = 12;
  public static final SwerveModuleConstants SWERVE_FR_CONSTANTS =
      new SwerveModuleConstants(
              Rotation2d.fromDegrees(32.2), SwerveCorner.FRONT_RIGHT, false, false);
  public static final int SWERVE_BL_DRIVE_MOTOR_ID = 4;
  public static final int SWERVE_BL_STEER_MOTOR_ID = 5;
  public static final int SWERVE_BL_CANCODER_ID = 11;
  public static final SwerveModuleConstants SWERVE_BL_CONSTANTS = new SwerveModuleConstants(
              Rotation2d.fromDegrees(-101.25), SwerveCorner.BACK_LEFT, false, false);
  public static final int SWERVE_BR_DRIVE_MOTOR_ID = 2;
  public static final int SWERVE_BR_STEER_MOTOR_ID = 3;
  public static final int SWERVE_BR_CANCODER_ID = 10;
  public static final SwerveModuleConstants SWERVE_BR_CONSTANTS =
new SwerveModuleConstants(
              Rotation2d.fromDegrees(-75.42), SwerveCorner.BACK_RIGHT, false, false);
  // 104.58
  public static final int ELEVATOR_MOTOR_ID = 14;

  public static final double ELEVATOR_GEARING =16.0;
  public static final double ELEVATOR_MIN_HEIGHT =  0;
  public static final double ELEVATOR_MAX_HEIGHT =   12;

  public static final double ELEVATOR_KV =  0;
  public static final double ELEVATOR_KF =  0;
  public static final double ELEVATOR_KP =  0.8;
  public static final double ELEVATOR_KI =  0;
  public static final double ELEVATOR_KD =   0;
  public static final double ELEVATOR_KS =  0;

  public static final int ELEVATOR_CRUISE_VELOCITY =  15000;
  public static final int ELEVATOR_ACCELERATION =  27500;
  public static final boolean ELEVATOR_INVERTED =  true;

  public static final int LIGHTS_CANDLE_ID = 15;
  public static final int LIGHTS_LED_COUNT = 0;

  public static final int WRIST_MOTOR_ID = 16;
  public static final double WRIST_GEARING = 48.0 * 2;
  public static final int WRIST_KV =  0;
  public static final double WRIST_KP = 0.1;
  public static final int WRIST_KI =  0;
  public static final int WRIST_KD =  0;
  public static final int WRIST_MOTION_CRUISE_VELOCITY =  20000;
  public static final int WRIST_MOTION_ACCELERATION = 50000;
  public static final double WRIST_HOMED_CURRENT =  5;
  public static final Rotation2d WRIST_HOMED_ANGLE = Rotation2d.fromDegrees(0.0);
  public static final double WRIST_HOMING_VOLTAGE =  -0.15;

  public static final int INTAKE_MOTOR_ID = 17;
  public static final boolean INVERTED_INTAKE =  true;

  public static final double SWERVE_STEER_KV =  0.0;
  public static final double SWERVE_STEER_KP =  3.0;
  public static final double SWERVE_STEER_KI =  0.0;
  public static final double SWERVE_STEER_KD =  0.0;
  public static final double SWERVE_STEER_KS =  0.0;

  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_FORWARD_VOLTAGE =  0;
  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_REVERSE_VOLTAGE =  0;
  public static final double SWERVE_DRIVE_CURRENT_LIMIT = 0;
  public static final boolean SWERVE_DRIVE_LIMITS_ENABLE = true;

  public static final double SWERVE_DRIVE_KP =  0.1;
  public static final double SWERVE_DRIVE_KI =  0.0;
  public static final double SWERVE_DRIVE_KD =  0.0;
  public static final double SWERVE_DRIVE_KV =  0.117;
  public static final double SWERVE_DRIVE_KS = 0.0;

  public static final double STEER_MOTOR_LIMITS =  0.0;
  public static final boolean SWERVE_MOTOR_LIMITS_ENABLED =  true;
  public static final PIDConstants SWERVE_TRANSLATION_PID = new PIDConstants(7, 0, 0.05);
  public static final PIDConstants SWERVE_ROTATION_PID = new PIDConstants(4.25, 0, 0.4);
  public static final PIDConstants SWERVE_ROTATION_SNAP_PID =
           new PIDConstants(0, 0, 0); // TODO: Edit these PID constants
  public static final boolean SWERVE_USE_FOC = true;

  public static final double SUPERSTRUCTURE_COLLISION_HEIGHT =  26;
  public static final Rotation2d SUPERSTRUCTURE_WRIST_RANGE = Rotation2d.fromDegrees(13);

  public static final double ROBOT_CENTER_TO_FRONT = Units.inchesToMeters(17.5);

  private Config() {}
}
