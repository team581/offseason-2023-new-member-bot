// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.Autobalance;
import frc.robot.autos.Autos;
import frc.robot.config.Config;
import frc.robot.controller.DriveController;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.AutoRotate;
import frc.robot.util.scheduling.LifecycleSubsystemManager;
import frc.robot.wrist.Positions;
import frc.robot.wrist.WristSubsystem;
import frc.swerve.SwerveSubsystem;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  // Enables power distribution logging
  private final PowerDistribution pdpLogging = new PowerDistribution(0, ModuleType.kCTRE);

  private final DriveController driveController = new DriveController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final RumbleControllerSubsystem rumbleController =
      new RumbleControllerSubsystem(new XboxController(1));



  private final FmsSubsystem fmsSubsystem = new FmsSubsystem();
  private final ImuSubsystem imu = new ImuSubsystem(new Pigeon2(Config.PIGEON_ID));
  private final SwerveSubsystem swerve = new SwerveSubsystem(imu);
  private final IntakeSubsystem intake =
      new IntakeSubsystem(new CANSparkMax(Config.INTAKE_MOTOR_ID, MotorType.kBrushless));
  private final WristSubsystem wrist =
      new WristSubsystem(new CANSparkMax(Config.WRIST_MOTOR_ID, MotorType.kBrushless));
  private final LocalizationSubsystem localization = new LocalizationSubsystem(swerve, imu);
  private final AutoRotate autoRotate = new AutoRotate(swerve);
  private final Autobalance autobalance = new Autobalance(swerve, imu);

  private final Autos autos = new Autos(localization, swerve, intake, wrist, autobalance);

  private Command autoCommand;

  public Robot() {
    // Log to a USB stick
    Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1/"));
    if (Config.IS_DEVELOPMENT) {
      // Publish data to NetworkTables
      Logger.getInstance().addDataReceiver(new NT4Publisher());
    }

    // Record metadata
    Logger.getInstance().recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.getInstance().recordMetadata("RoborioSerialNumber", Config.SERIAL_NUMBER);
    Logger.getInstance().recordMetadata("RobotConfig", "Offseason bot 1");
    Logger.getInstance().recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.getInstance().recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.getInstance().recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.getInstance().recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.getInstance().recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.getInstance().recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.getInstance().recordMetadata("GitDirty", "Unknown");
        break;
    }

    Logger.getInstance().start();

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.getInstance().ready();

    configureButtonBindings();

    enableLiveWindowInTest(false);

    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {}

  private void configureButtonBindings() {
    swerve.setDefaultCommand(swerve.getDriveTeleopCommand(driveController));

    // intake
    driveController
        .leftTrigger(0.5)
        .onTrue(
            intake
                .setStateCommand(IntakeState.INTAKING)
                .alongWith(wrist.goToAngle(Positions.INTAKING).alongWith(Commands.print("hghgh")))
                .until(() -> intake.hasCube())
                .andThen(wrist.goToAngle(Positions.STOWED).withName("TeleopIntakeCommand")))
        // .alongWith(intake.setStateCommand(IntakeState.STOPPED))))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));
    // .alongWith(Commands.runOnce(() -> intake.setHasCube(true))));
    // outtake
    driveController
        .rightTrigger(0.5)
        .onTrue(intake.setStateCommand(IntakeState.OUTTAKING))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));
    // shoot
    driveController
        .rightBumper()
        .onTrue(intake.setStateCommand(IntakeState.SHOOTING))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));
    // snaps
    driveController.b().onTrue(autoRotate.getCommand(() -> AutoRotate.getLeftAngle()));
    driveController.x().onTrue(autoRotate.getCommand(() -> AutoRotate.getRightAngle()));
    driveController.a().onTrue(autoRotate.getCommand(() -> AutoRotate.getForwardAngle()));
    driveController.y().onTrue(autoRotate.getCommand(() -> AutoRotate.getBackwardsAngle()));

    // get x swerve
    driveController.start().onTrue(swerve.getXSwerveCommand());

    // reset gyroscope
    driveController.back().onTrue(localization.getZeroCommand());

    new Trigger(() -> driveController.getThetaPercentage() == 0)
        .onFalse(autoRotate.getDisableCommand());

    new Trigger(
            () ->
                driveController.getSidewaysPercentage() == 0
                    && driveController.getForwardPercentage() == 0
                    && driveController.getThetaPercentage() == 0);

    // operator home wrist
    operatorController.back().onTrue(wrist.getHomeCommand());
    // operator stow
    operatorController.x().onTrue(wrist.goToAngle(Positions.STOWED));
    // set the superstructure low
    operatorController
        .a()
        .onTrue(wrist.goToAngle(Positions.OUTTAKING_LOW))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));
    // set the superstructure mid
    operatorController
        .b()
        .onTrue(wrist.goToAngle(Positions.OUTTAKING_MID))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));

    operatorController
        .povUp()
        .onTrue(
            wrist
                .goToAngle(Positions.SHOOT_ON_HIGH_FOLLOW_THRU)
                .alongWith(
                    Commands.waitUntil(() -> wrist.pastAngle(Positions.SHOOT_ON_HIGH))
                        .andThen(intake.setStateCommand(IntakeState.SHOOTING))))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));

    operatorController
        .povDown()
        .onTrue(
            wrist
                .goToAngle(Positions.SHOOT_ON_MID_FOLLOW_THRU)
                .alongWith(
                    Commands.waitUntil(() -> wrist.pastAngle(Positions.SHOOT_ON_MID))
                        .andThen(intake.setStateCommand(IntakeState.SHOOTING))))
        .onFalse(
            wrist
                .goToAngle(Positions.STOWED)
                .alongWith(intake.setStateCommand(IntakeState.STOPPED)));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    LifecycleSubsystemManager.getInstance().log();

    Logger.getInstance()
        .recordOutput("Intake/requestingIntake", driveController.leftTrigger(0.5).getAsBoolean());
  }

  @Override
  public void autonomousInit() {
    autoCommand = autos.getAutoCommand();
    CommandScheduler.getInstance().schedule(autoCommand);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
      autoCommand = null;
    }

    autos.clearCache();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // Keep selected auto in cache to avoid loading it when auto starts
    autos.getAutoCommand();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
