// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final double WRIST_TOLERANCE = 3;
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pid;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(0);
  private Rotation2d goalAngle = new Rotation2d();
  private HomingState homingState = HomingState.NOT_HOMED;

  public WristSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.WRIST);

    pid = motor.getPIDController();
    this.motor = motor;
    encoder = motor.getEncoder();
    pid.setP(Config.WRIST_KP);
    pid.setI(Config.WRIST_KI);
    pid.setD(Config.WRIST_KD);
    pid.setSmartMotionMaxAccel(Config.WRIST_MOTION_MAX_ACCELERATION, 0);
    pid.setSmartMotionMaxVelocity(Config.WRIST_MOTION_MAX_VELOCITY, 0);
    encoder.setPositionConversionFactor(Config.WRIST_GEARING);
  }

  @Override
  public void enabledPeriodic() {
    double rawCurrent = motor.getOutputCurrent();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    if (homingState == HomingState.NOT_HOMED) {
      // set the motor to 0
      motor.set(0);
    }
    if (homingState == HomingState.HOMING) {
      // set the motor to run at like 10% ish
      motor.set(0.1);

      if (filteredCurrent > 5) {
        motor.set(0);
        encoder.setPosition(0);
        set(Rotation2d.fromDegrees(0));
        homingState = HomingState.HOMED;
      }
    } else if (homingState == HomingState.HOMED) {
      // read the goal angle, set the motor to go to that position
      pid.setReference(goalAngle.getRotations(), ControlType.kSmartMotion);
    }
  }

  // when the robot is enabled and homing state is HOMED, go to the stored goal angle
  public HomingState getHomingState() {
    return homingState;
  }

  // when the robot is enabled and homing state is HOMING, run the homing code (run motor at 10%),
  // stop when current is above 10A
  public void startHoming() {
    homingState = HomingState.HOMING;
  }

  // when the robot is enabled and homing state is NOT_HOMED, set the voltage to 0
  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
    motor.set(0);
  }

  // when the robot is powered on, log a few values related to the wrist
  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Velocity", encoder.getVelocity());
    Logger.getInstance().recordOutput("Wrist/Angle", getWristAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/DutyCycleOutput", motor.getAppliedOutput());
    Logger.getInstance().recordOutput("Wrist/StatorCurrent", motor.getOutputCurrent());
    Logger.getInstance().recordOutput("Wrist/Homing", homingState.toString());
  }

  // current angle, goal angle
  private Rotation2d getWristAngle() {
    return Rotation2d.fromRotations(encoder.getPosition());
  }

  // add a method to set the goal angle
  public void set(Rotation2d angle) {
    goalAngle = angle;
  }

  // add a method to check if we're at a given angle (within 3 degrees of it)
  public boolean atAngle(Rotation2d angle) {
    double encoderAngle = getWristAngle().getDegrees();
    return Math.abs(encoderAngle - angle.getDegrees()) < WRIST_TOLERANCE;
  }

  // add a method to return a command which sets the goal angle to something and then exits the
  // command once at that angle
  public Command goToAngle(Rotation2d toGoalAngle) {
    return run(() -> {
          set(toGoalAngle);
        })
        .until(() -> atAngle(goalAngle));
  }

  public Command getHomeCommand() {
    return runOnce(() -> startHoming())
        .andThen(Commands.waitUntil(() -> getHomingState() == HomingState.HOMED));
  }

  public Rotation2d getGoalAngle() {
    return goalAngle;
  }
}
