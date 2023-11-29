// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.swerve;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.imu.ImuSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class SwerveSubsystem extends LifecycleSubsystem {
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          Config.SWERVE_FRONT_LEFT_LOCATION,
          Config.SWERVE_FRONT_RIGHT_LOCATION,
          Config.SWERVE_BACK_LEFT_LOCATION,
          Config.SWERVE_BACK_RIGHT_LOCATION);

  private static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(1)
          .withSupportsPro(true)
          .withCANbusName("581CANivore");
  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(Config.SWERVE_DRIVE_GEARING_REDUCTION)
          .withSteerMotorGearRatio(Config.SWERVE_STEER_GEARING_REDUCTION)
          .withWheelRadius(Config.WHEEL_DIAMETER / 2.0)
          //  TODO: Ask Saikiran about this
          .withSlipCurrent(800)
          .withSteerMotorGains(
              new CustomSlotGains(
                  Config.SWERVE_STEER_KP,
                  Config.SWERVE_STEER_KI,
                  Config.SWERVE_STEER_KD,
                  Config.SWERVE_STEER_KV,
                  Config.SWERVE_STEER_KS))
          .withDriveMotorGains(
              new CustomSlotGains(
                  Config.SWERVE_DRIVE_KP,
                  Config.SWERVE_DRIVE_KI,
                  Config.SWERVE_DRIVE_KD,
                  Config.SWERVE_DRIVE_KV,
                  Config.SWERVE_DRIVE_KS))
          .withSpeedAt12VoltsMps(
              6) // Theoretical free speed is 10 meters per second at 12v applied output
          .withSteerInertia(0.0001)
          .withDriveInertia(0.001)
          .withFeedbackSource(SwerveModuleSteerFeedbackType.FusedCANcoder)
          // TODO: Ask Saikran about this
          .withCouplingGearRatio(
              0.0) // Every 1 rotation of the azimuth results in couple ratio drive turns
          .withSteerMotorInverted(true);
  private static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          Config.SWERVE_FL_STEER_MOTOR_ID,
          Config.SWERVE_FL_DRIVE_MOTOR_ID,
          Config.SWERVE_FL_CANCODER_ID,
          Rotation2d.fromDegrees(182.021484375).getRotations(),
          Config.SWERVE_FRONT_LEFT_LOCATION.getX(),
          Config.SWERVE_FRONT_LEFT_LOCATION.getY(),
          true);
  private static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          Config.SWERVE_FR_STEER_MOTOR_ID,
          Config.SWERVE_FR_DRIVE_MOTOR_ID,
          Config.SWERVE_FR_CANCODER_ID,
          Rotation2d.fromDegrees(159.521484375).getRotations(),
          Config.SWERVE_FRONT_RIGHT_LOCATION.getX(),
          Config.SWERVE_FRONT_RIGHT_LOCATION.getY(),
          true);
  private static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          Config.SWERVE_BL_STEER_MOTOR_ID,
          Config.SWERVE_BR_DRIVE_MOTOR_ID,
          Config.SWERVE_BL_CANCODER_ID,
          Rotation2d.fromDegrees(2.548828125).getRotations(),
          Config.SWERVE_BACK_LEFT_LOCATION.getX(),
          Config.SWERVE_BACK_LEFT_LOCATION.getY(),
          true);
  private static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          Config.SWERVE_BR_STEER_MOTOR_ID,
          Config.SWERVE_BR_DRIVE_MOTOR_ID,
          Config.SWERVE_BR_CANCODER_ID,
          Rotation2d.fromDegrees(110.390625).getRotations(),
          Config.SWERVE_BACK_RIGHT_LOCATION.getX(),
          Config.SWERVE_BACK_RIGHT_LOCATION.getY(),
          true);

  private static final SwerveDrivetrain drivetrain =
      new SwerveDrivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);

  private static final double MAX_VELOCITY =
      ((6080.0 / 60.0) / Config.SWERVE_DRIVE_GEARING_REDUCTION) * (Config.WHEEL_DIAMETER * Math.PI);
  private static final double MAX_ANGULAR_VELOCITY = 20;

  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  private final SwerveRequest.FieldCentricFacingAngle driveRequestSnaps =
      new SwerveRequest.FieldCentricFacingAngle();
  private final SwerveRequest.SwerveDriveBrake driveRequestX = new SwerveRequest.SwerveDriveBrake();

  private final ImuSubsystem imu;
  private boolean snapToAngle = false;
  private Rotation2d goalAngle = new Rotation2d();
  private boolean xSwerveEnabled;

  public SwerveSubsystem(ImuSubsystem imu) {
    super(SubsystemPriority.SWERVE);
    this.imu = imu;
    drivetrain.registerTelemetry(
        (SwerveDrivetrain.SwerveDriveState state) -> {
          Logger.getInstance().recordOutput("Swerve/FailedDaqs", state.FailedDaqs);
          Logger.getInstance().recordOutput("Swerve/SuccessfulDaqs", state.SuccessfulDaqs);
          Logger.getInstance().recordOutput("Swerve/Pose", state.Pose);
          Logger.getInstance().recordOutput("Swerve/ModuleStates", state.ModuleStates);
          Logger.getInstance().recordOutput("Swerve/OdometryPeriod", state.OdometryPeriod);
        });
  }

  public Command getXSwerveCommand() {
    return runOnce(() -> setXSwerve(false));
  }

  public Command getDriveTeleopCommand(DriveController controller) {
    return Commands.run(
            () -> {
              if (!DriverStation.isTeleopEnabled()) {
                return;
              }

              boolean openLoop = true;

              driveTeleop(
                  -controller.getSidewaysPercentage(),
                  controller.getForwardPercentage(),
                  controller.getThetaPercentage());
            },
            this)
        .withName("SwerveDriveTeleop");
  }

  public void driveTeleop(
      double SidewaysPercentage, double forwardPercentage, double thetaPercentage) {
    setChassisSpeeds(KINEMATICS.toChassisSpeeds(getModuleStates()), false);
  }

  public void setSnapToAngle(Rotation2d angle) {
    goalAngle = angle;
    snapToAngle = true;
  }

  public void disableSnapToAngle() {
    snapToAngle = false;
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds, boolean closedLoop) {
    if (xSwerveEnabled) {
      drivetrain.setControl(driveRequestX);
    } else if (snapToAngle) {
      drivetrain.setControl(
          driveRequestSnaps
              .withIsOpenLoop(!closedLoop)
              .withVelocityX(chassisSpeeds.vxMetersPerSecond)
              .withVelocityY(chassisSpeeds.vyMetersPerSecond)
              .withTargetDirection(goalAngle));
    } else {
      drivetrain.setControl(
          driveRequest
              .withIsOpenLoop(!closedLoop)
              .withVelocityX(chassisSpeeds.vxMetersPerSecond)
              .withVelocityY(chassisSpeeds.vyMetersPerSecond)
              .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
    }
  }

  public void setXSwerve(boolean enabled) {
    this.xSwerveEnabled = enabled;
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      drivetrain.getModule(0).getPosition(true),
      drivetrain.getModule(1).getPosition(true),
      drivetrain.getModule(2).getPosition(true),
      drivetrain.getModule(3).getPosition(true),
    };
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      drivetrain.getModule(0).getCurrentState(),
      drivetrain.getModule(1).getCurrentState(),
      drivetrain.getModule(2).getCurrentState(),
      drivetrain.getModule(3).getCurrentState(),
    };
  }
}
