// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPRamseteController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.fms.FmsSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.wrist.Positions;
import frc.robot.wrist.WristSubsystem;
import java.lang.ref.WeakReference;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static Command wrapAutoEvent(String commandName, Command command) {
    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static Map<String, Command> wrapAutoEventMap(Map<String, Command> eventMap) {
    Map<String, Command> wrappedMap = new HashMap<>();
    for (Map.Entry<String, Command> entry : eventMap.entrySet()) {
      wrappedMap.put(
          entry.getKey(), wrapAutoEvent("AutoEvent_" + entry.getKey(), entry.getValue()));
    }
    return wrappedMap;
  }

  private final LoggedDashboardChooser<AutoKindWithoutTeam> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final Map<AutoKind, WeakReference<Command>> autosCache = new EnumMap<>(AutoKind.class);

  private final LocalizationSubsystem localization;

  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final WristSubsystem wrist;
  private final Autobalance autobalance;

  public Autos(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      WristSubsystem wrist,
      Autobalance autobalance) {

    this.localization = localization;
    this.swerve = swerve;
    this.intake = intake;
    this.wrist = wrist;
    this.autobalance = autobalance;

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("[COMMANDS] Starting command " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> System.out.println("[COMMANDS] Cancelled command " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> System.out.println("[COMMANDS] Finished command " + command.getName()));

    for (AutoKindWithoutTeam autoKind : EnumSet.allOf(AutoKindWithoutTeam.class)) {
      autoChooser.addOption(autoKind.toString(), autoKind);
    }



    Logger.getInstance().recordOutput("Autos/CurrentTrajectory", new Trajectory());
    Logger.getInstance().recordOutput("Autos/TargetPose", new Pose2d());
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/X", 0);
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/Y", 0);
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/Omega", 0);
    Logger.getInstance().recordOutput("Autos/TranslationError", new Pose2d());
    Logger.getInstance().recordOutput("Autos/RotationError", 0);

  }

  private Command buildAutoCommand(AutoKind auto) {
    WeakReference<Command> ref = autosCache.get(auto);
    if (ref != null && ref.get() != null) {
      Command autoCommand = ref.get();

      if (autoCommand != null) {
        return autoCommand;
      }
    }

    String autoName = "Auto" + auto.toString();
    Command autoCommand = Commands.runOnce(() -> swerve.driveTeleop(0, 0, 0, true, true), swerve);

    if (auto == AutoKind.DO_NOTHING) {
      return autoCommand
          .andThen(localization.getZeroAwayCommand())
          .andThen(wrist.getHomeCommand())
          .withName(autoName);
    }

    List<PathPlannerTrajectory> pathGroup = Paths.getInstance().getPath(auto);

    // todo: update
    // autoCommand = autoCommand.andThen(autoBuilder.fullAuto(pathGroup));

    if (auto.autoBalance) {
      autoCommand = autoCommand.andThen(this.autobalance.getCommand());
    }

    autoCommand = autoCommand.withName(autoName);

    autosCache.put(auto, new WeakReference<>(autoCommand));

    return autoCommand;
  }

  public Command getAutoCommand() {
    AutoKindWithoutTeam rawAuto = autoChooser.get();

    if (rawAuto == null) {
      rawAuto = AutoKindWithoutTeam.MID_1_BALANCE;
    }

    AutoKind auto = FmsSubsystem.isRedAlliance() ? rawAuto.redVersion : rawAuto.blueVersion;

    return buildAutoCommand(auto);
  }

  public void clearCache() {
    autosCache.clear();
    Paths.getInstance().clearCache();
  }
}
