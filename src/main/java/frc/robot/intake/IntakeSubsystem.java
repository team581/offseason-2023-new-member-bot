// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  // seven is a placeholder not tuned.
  private final LinearFilter voltageFilter = LinearFilter.movingAverage(7);
  private final LinearFilter velocityFilter = LinearFilter.movingAverage(7);
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private IntakeState goalState = IntakeState.STOPPED;
  private boolean holdingCube = false;
  private final Timer intakeTimer = new Timer();

  public IntakeSubsystem(CANSparkMax motor) {
    super(SubsystemPriority.INTAKE);
    this.motor = motor;
    encoder = motor.getEncoder();

    motor.setInverted(Config.INVERTED_INTAKE);

    encoder.setPositionConversionFactor(Config.INTAKE_GEARING);
    encoder.setVelocityConversionFactor(Config.INTAKE_GEARING);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/IntakeState", goalState.toString());
    Logger.getInstance().recordOutput("Intake/HasCube", hasCube());
  }

  @Override
  public void enabledPeriodic() {
    if (goalState == IntakeState.OUTTAKING) {
      motor.set(-0.5);
    } else if (holdingCube) {
      motor.set(0.1);
    } else if (goalState == IntakeState.INTAKING) {
      motor.set(0.5);
    } else {
      motor.set(0);
    }

    // Game piece detection
    double motorVelocity = velocityFilter.calculate(encoder.getVelocity());
    double intakeVoltage = voltageFilter.calculate(motor.getAppliedOutput()) * 12.0;
    double theoreticalSpeed = intakeVoltage * (5700.0 / 12.0); // Neo Max is 5700
    double threshold = theoreticalSpeed * 0.5;
    Logger.getInstance().recordOutput("Intake/MotorVelocity", motorVelocity);
    Logger.getInstance().recordOutput("Intake/IntakeVoltage", intakeVoltage);
    Logger.getInstance().recordOutput("Intake/TheoreticalSpeed", theoreticalSpeed);
    Logger.getInstance().recordOutput("Intake/Threshold", threshold);
    // if (intakeTimer.hasElapsed(0.5)) {
    //   if (motorVelocity < threshold && goalState == IntakeState.INTAKING) {
    //     holdingCube = true;
    //   } else if (motorVelocity > threshold && goalState == IntakeState.OUTTAKING) {
    //     holdingCube = false;
    //   }
    // }
  }

  public void setGoalState(IntakeState newState) {
    if (goalState != newState) {
      if (newState == IntakeState.INTAKING) {
        holdingCube = false;
      }

      if (newState == IntakeState.INTAKING || newState == IntakeState.OUTTAKING) {
        intakeTimer.reset();
        intakeTimer.start();
      }
    }
    goalState = newState;
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

  public Command setStateCommand(IntakeState intakeState) {
    return run(() -> {
          setGoalState(intakeState);
        })
        .until(() -> atGoal(intakeState));
  }

  public boolean hasCube() {
    return holdingCube;
  }

  public void setHasCube(boolean newHasCube) {
    holdingCube = newHasCube;
  }
}
