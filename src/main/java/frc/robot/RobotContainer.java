// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.LoadAlgae1;
import frc.robot.commands.LoadAlgae2;
import frc.robot.commands.LockHeadingOnAprilTag;
import frc.robot.commands.LockXOnAlgae;
import frc.robot.commands.LockXOnAprilTag;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Lighting;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.KeyboardController;
import swervelib.SwerveInputStream;

public class RobotContainer {



  // Controllers
  final CommandXboxController driver = new CommandXboxController(0);
  RumbleState driverRumbleState = RumbleState.TARGET_NONE;
  double driverRumbleIntensity = 0;
  final KeyboardController operatorController = new KeyboardController(0);

  // Subsystems
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
      new File(Filesystem.getDeployDirectory(), "swerve-2025"));
  private final AlgaeCollectorSubsystem algaeCollectorSubsystem = new AlgaeCollectorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final Lighting lighting = new Lighting();

  // Auto Chooser
    private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDriveSubsystem.getSwerveDrive(),
      () -> MathUtil.applyDeadband(driver.getLeftY() * -1, 0.05),
      () -> MathUtil.applyDeadband(driver.getLeftX() * -1, 0.05))
      .withControllerRotationAxis(() -> MathUtil.applyDeadband(driver.getRightX(), 0.05))
      .deadband(0.1)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(() -> MathUtil.applyDeadband(driver.getRightX(), 0.05),
    () -> MathUtil.applyDeadband(driver.getRightY(), 0.05))
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  public RobotContainer() {
    NamedCommands.registerCommand("Algae L2", CompositeCommands.elevatorAlgaeL2(elevator, wristSubsystem));
    NamedCommands.registerCommand("Collect Algae",algaeCollectorSubsystem.autoCollect());
    NamedCommands.registerCommand("Collect Algae (timeout version)",algaeCollectorSubsystem.autoCollectWithTimeout());
    NamedCommands.registerCommand("Barge Back", CompositeCommands.elevatorBargeBack(elevator, wristSubsystem));
    NamedCommands.registerCommand("Release Algae", algaeCollectorSubsystem.autoRelease());
    NamedCommands.registerCommand("Stop algae collector", algaeCollectorSubsystem.stop());
    NamedCommands.registerCommand("Storage", CompositeCommands.elevatorStorage(elevator, wristSubsystem));
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

    algaeCollectorSubsystem.setDefaultCommand(new LoadAlgae2(algaeCollectorSubsystem));
  }

  private void configureBindings() {
    configureDriverController();
    //configureOperatorController();
    configureWPIOperatorController();
  }

  public void configureDriverController() {
    Command driveRobotOrientedAngularVelocity = swerveDriveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngle = swerveDriveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveNew = swerveDriveSubsystem.driveFieldOriented(driveAngularVelocity);
    

    // swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    // swerveDriveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);
    swerveDriveSubsystem.setDefaultCommand(driveNew);


    //driver.y().whileTrue(Commands.runOnce(() -> swerveDriveSubsystem.pathPlannerStartPose()));
    driver.x().whileTrue(Commands.runOnce(() -> swerveDriveSubsystem.getSwerve().lockPose()));


    driver.start().onTrue((Commands.runOnce(swerveDriveSubsystem::zeroGyro)));
    /*
    driver.rightBumper().whileTrue(new LockHeadingOnAprilTag(swerveDriveSubsystem,
        () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.05),
        () -> MathUtil.applyDeadband(-driver.getLeftX(), 0.05),
        new ControllerRumbleCallback() {
          @Override
          public void update(RumbleState rumbleState) {
            driverRumbleState = rumbleState;
          }
        }));
        */


    Command lockOn = new LockXOnAprilTag(
      swerveDriveSubsystem,
      () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.05),
      () -> 0);
 
    driver.rightBumper()
      .onTrue(new InstantCommand(() -> {
        SmartDashboard.putBoolean("Target Left Reef", false);
      }))
      .whileTrue(lockOn);

    driver.leftBumper()
      .onTrue(new InstantCommand(() -> {
        SmartDashboard.putBoolean("Target Left Reef", true);
      }))
      .whileTrue(lockOn);

    driver.a().onTrue(new InstantCommand(swerveDriveSubsystem::zeroGyro, swerveDriveSubsystem));

    Command lockOnAlgae = new LockXOnAlgae(
      swerveDriveSubsystem,
      () -> MathUtil.applyDeadband(-driver.getLeftY(), 0.05),
      () -> 0);
    driver.leftTrigger().whileTrue(lockOnAlgae);

    driver.rightTrigger().onTrue(driveRobotOrientedAngularVelocity).onFalse(driveNew);

    // TODO - remove these, test out the motor on the climber
    //driver.b().whileTrue(climber.testPositive()).onFalse(climber.testStop());
    //driver.x().whileTrue(climber.testNegative()).onFalse(climber.testStop());
  }

  public void configureWPIOperatorController() {

    //// Manual Wrist Control
    // Rotate Up
    operatorController.wpiOperatorController()
      .rotateWristUp()
      .whileTrue(wristSubsystem.up());

    // Rotate Down
    operatorController.wpiOperatorController()
      .rotateWristDown()
      .whileTrue(wristSubsystem.down());

    //// Wrist Setpoints
    // Storage
    operatorController.wpiOperatorController()
      .wristToStorage()
      .onTrue(wristSubsystem.algaeStorage());

    // Hold Algae
    operatorController.wpiOperatorController()
      .wristToHold()
      .onTrue(wristSubsystem.algaeHeld());

    // Reef
    operatorController.wpiOperatorController()
      .wristToReef()
      .onTrue(wristSubsystem.algaeL2());

    // Ground
    operatorController.wpiOperatorController()
      .wristToGround()
      .onTrue(wristSubsystem.algaeGround());

    //// Manual Algae Collector Control
    // Collect
    operatorController.wpiOperatorController()
      .algaeCollect()
      .whileTrue(algaeCollectorSubsystem.in2())
      .onFalse(algaeCollectorSubsystem.stop());

    // Release
    operatorController.wpiOperatorController()
      .algaeRelease()
      .whileTrue(algaeCollectorSubsystem.out())
      .onFalse(algaeCollectorSubsystem.stop());

    //// Climber Control
    // climb1 - DEPLOY
    operatorController.wpiOperatorController()
      .climb1()
      .onTrue(climber.deploy());

    // climb2 - CLIMB
    operatorController.wpiOperatorController()
      .climb2()
      .onTrue(climber.climb2());

    //// Manual Elevator Control
    // Up
    operatorController.wpiOperatorController()
      .elevatorUp()
      .onTrue(elevator.up())
      .onFalse(elevator.stop());

    // Down
    operatorController.wpiOperatorController()
      .elevatorDown()
      .onTrue(elevator.down())
      .onFalse(elevator.stop());

    //// Elevator / Wrist Setpoints
    // Barge Back 

    // TODO - need to insure that the elevator is at a height where it can go back
    operatorController.wpiOperatorController()
      .bargeBack()
      .onTrue(CompositeCommands.elevatorBargeBack(elevator, wristSubsystem));

    // Barge Front
    operatorController.wpiOperatorController()
      .bargeFront()
      .onTrue(CompositeCommands.elevatorBargeFront(elevator, wristSubsystem));

    // Algae L3
    operatorController.wpiOperatorController()
      .algaeL3()
      .onTrue(CompositeCommands.elevatorAlgaeL3(elevator, wristSubsystem));

    // Algae L2
    operatorController.wpiOperatorController()
      .algaeL2()
      .onTrue(CompositeCommands.elevatorAlgaeL2(elevator, wristSubsystem));

    // Ground
    operatorController.wpiOperatorController()
      .elevatorGround()
      .onTrue(CompositeCommands.elevatorAlgaeGround(elevator, wristSubsystem));

    // Processor
    operatorController.wpiOperatorController()
      .elevatorProcessor()
      .onTrue(CompositeCommands.elevatorAlgaeProcessor(elevator, wristSubsystem));

    // Storage
    operatorController.wpiOperatorController()
      .elevatorStorage()
      .onTrue(CompositeCommands.elevatorStorage(elevator, wristSubsystem));

    operatorController.wpiOperatorController()
      .forceElevatorDown()
      .onTrue(Commands.sequence(elevator.turnLimitsOff(), elevator.downOverrideLimit()))
      .onFalse(Commands.sequence(elevator.stop(), elevator.turnLimitsOn()));

    operatorController.wpiOperatorController()
      .forceElevatorUp()
      .onTrue(Commands.sequence(elevator.turnLimitsOff(), elevator.upOverrideLimit()))
      .onFalse(Commands.sequence(elevator.stop(), elevator.turnLimitsOn()));
  }

  public void setSwerveOdometry() {
    Pose2d initialPose = swerveDriveSubsystem.getPose();
    swerveDriveSubsystem.resetOdometry(initialPose.rotateBy(new Rotation2d(Math.PI)));
  }

  public Command getAutonomousCommand() {
    //return Commands.run(() -> swerveDriveSubsystem.getSwerve().drive(new ChassisSpeeds(1, 0, 0)),swerveDriveSubsystem).withTimeout(1.75); 
    /*
    var alliance = DriverStation.getAlliance();
    if  (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        return Commands.run(() -> swerveDriveSubsystem.getSwerve().drive(new ChassisSpeeds(1, 0, 0)),swerveDriveSubsystem).withTimeout(3); 
    } else {
      return Commands.run(() -> swerveDriveSubsystem.getSwerve().drive(new ChassisSpeeds(-1, 0, 0)),swerveDriveSubsystem).withTimeout(3); 
    }
      */
    return autoChooser.getSelected();
  }
}
