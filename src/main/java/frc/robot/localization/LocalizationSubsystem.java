// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class LocalizationSubsystem extends LifecycleSubsystem {

  private final SwerveSubsystem swerve;
  private final ImuSubsystem imu;

  private final SwerveDriveOdometry odometry;

  public LocalizationSubsystem(SwerveSubsystem swerve, ImuSubsystem imu) {
    super(SubsystemPriority.LOCALIZATION);

    this.swerve = swerve;
    this.imu = imu;

    odometry =
        new SwerveDriveOdometry(
            SwerveSubsystem.KINEMATICS, imu.getRobotHeading(), swerve.getModulePositions());
  }

  @Override
  public void robotPeriodic() {
    update();

    Logger.getInstance().recordOutput("Localization/CombinedPose", getPose());
    Logger.getInstance().recordOutput("Localization/OdometryPose", odometry.getPoseMeters());
  }

  private void update() {
    try {
      odometry.update(imu.getRobotHeading(), swerve.getModulePositions());
    } catch (Exception e) {
      System.err.println("Odometry threw while adding measurement:");
      e.printStackTrace();
    }
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    imu.setAngle(pose.getRotation());
    odometry.resetPosition(pose.getRotation(), swerve.getModulePositions(), pose);
  }

  public void resetGyro(Rotation2d gyroAngle) {
    Pose2d pose = new Pose2d(getPose().getTranslation(), gyroAngle);
    resetPose(pose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 180 : 0)));
  }

  public Command getZeroAwayCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees(FmsSubsystem.isRedAlliance() ? 0 : 180)));
  }

  public boolean atPose(Pose2d goal) {

    Pose2d pose = getPose();
    double distanceRelative = goal.getTranslation().getDistance(pose.getTranslation());

    Logger.getInstance().recordOutput("Localization/AtPoseGoal", goal);

    Rotation2d rotationDifference = goal.getRotation().minus(pose.getRotation());
    if (distanceRelative < 0.2 && Math.abs(rotationDifference.getDegrees()) < 5) {
      Logger.getInstance().recordOutput("Localization/AtPose", true);
      return true;
    } else {
      Logger.getInstance().recordOutput("Localization/AtPose", false);
      return false;
    }
  }
}
