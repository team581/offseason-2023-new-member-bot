// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends LifecycleSubsystem {
  // seven is a placeholder not tuned.
  private final LinearFilter voltageFilter = LinearFilter.movingAverage(7);
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(7);
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private IntakeState goalState = IntakeState.STOPPED;
  private boolean holdingCube = false;

  public IntakeSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.INTAKE);
    this.motor = motor;
    encoder = motor.getEncoder();
  }

  @Override
  public void enabledPeriodic() {
    if (goalState == IntakeState.STOPPED) {
      motor.set(0);
    } else if (goalState == IntakeState.INTAKING) {
      motor.set(0.3);
    } else {
      motor.set(-0.3);
    }

    double motorVelocity = velocityFilter.calculate(encoder.getVelocity());
    double intakeVoltage = voltageFilter.calculate(motor.getAppliedOutput()) * 12.0;
    double theoreticalSpeed = intakeVoltage * (5700.0 / 12.0); // Neo Max is 5700
    double threshold = theoreticalSpeed * 0.5;

    if (goalState == IntakeState.INTAKING && motorVelocity < threshold) {
      holdingCube = true;
    } else if (goalState == IntakeState.OUTTAKING && motorVelocity > threshold) {
      holdingCube = false;
    }
  }

  public void setGoalState(IntakeState intakeState) {
    goalState = intakeState;
  }

  public boolean atGoal(IntakeState intakeState) {
    if (goalState != intakeState) {
      return false;
    }

    if (goalState == IntakeState.STOPPED) {
      return true;
    }

    if (goalState == IntakeState.OUTTAKING) {
      return !hasCube();
    }

    if (goalState == IntakeState.INTAKING) {
      return hasCube();
    }

    return false;
  }

  public Command setState(IntakeState intakeState) {
    return run(() -> {
          setGoalState(intakeState);
        })
        .until(() -> atGoal(intakeState));
  }

  public boolean hasCube() {
    return holdingCube;
  }
}
