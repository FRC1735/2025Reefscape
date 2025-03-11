// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.LoadAlgae;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.LockHeadingOnAprilTag;
import frc.robot.commands.LockXOnAprilTag;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubystem;
import frc.robot.subsystems.ElevatorSubsystem;
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
      new File(Filesystem.getDeployDirectory(), "swerve-2025"),
      new ControllerRumbleCallback() {
        @Override
        public void update(RumbleState rumbleState) {
          driverRumbleState = rumbleState;
        }
      });
  private final CoralSubystem coralSubystem = new CoralSubystem();
  private final AlgaeCollectorSubsystem algaeCollectorSubsystem = new AlgaeCollectorSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

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
    configureBindings();

    coralSubystem.setDefaultCommand(new LoadCoral(coralSubystem));
    algaeCollectorSubsystem.setDefaultCommand(new LoadAlgae(algaeCollectorSubsystem));
  }

  private void configureBindings() {
    configureDriverController();
    configureOperatorController();
  }

  public void configureDriverController() {
    Command driveRobotOrientedAngularVelocity = swerveDriveSubsystem.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngle = swerveDriveSubsystem.driveFieldOriented(driveDirectAngle);

    // swerveDriveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    swerveDriveSubsystem.setDefaultCommand(driveRobotOrientedAngularVelocity);

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
      () -> 0,
      new ControllerRumbleCallback() {
        @Override
        public void update(RumbleState rumbleState) {
          // TODO
        }
      });

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
    /*
     * driver
     * .a()
     * .onTrue(new InstantCommand(coralSubystem::shoot, coralSubystem))
     * .onFalse(new InstantCommand(coralSubystem::stop, coralSubystem));
     * 
     * driver
     * .b()
     * .onTrue(new InstantCommand(coralSubystem::returnToFunnel, coralSubystem))
     * .onFalse(new InstantCommand(coralSubystem::stop, coralSubystem));
     */

    /*
     * driver.a().onTrue(new InstantCommand(wristSubsystem::testPositionControl,
     * wristSubsystem));
     * 
     * driver.x().onTrue(new InstantCommand(wristSubsystem::testPositionControl2,
     * wristSubsystem));
     * 
     * 
     * driver.y().onTrue(new InstantCommand(wristSubsystem::testPositionControl3,
     * wristSubsystem));
     */

    // driver.b().onTrue(new InstantCommand(wristSubsystem::stop, wristSubsystem));

  }

  public void configureOperatorController() {
    //////////  
    // Coral Shooter
    operatorController.coralCollector().score().whileTrue(coralSubystem.shoot()).onFalse(coralSubystem.stop());
    operatorController.coralCollector().reverse().whileTrue(coralSubystem.reverse()).onFalse(coralSubystem.stop());
    //operatorController.coralCollector().load().onTrue(coralSubystem.load()).onFalse(coralSubystem.stop());
    //////////  
    // Elevator
    
    //// Manual Controls
    operatorController.elevator().up().and(coralSubystem.isSafeForElevator()).onTrue(elevator.up()).onFalse(elevator.stop());
    operatorController.elevator().down().and(coralSubystem.isSafeForElevator()).onTrue(elevator.down()).onFalse(elevator.stop());

    //// Algae Delivery Setpoints
    operatorController.elevator().algaeBarge().and(coralSubystem.isSafeForElevator())
      .onTrue(
          CompositeCommands.elevatorAlgaeBarge(elevator, wristSubsystem)
      );
    // Algae L3
    operatorController.elevator().algaeL3().and(coralSubystem.isSafeForElevator())
      .onTrue(
        CompositeCommands.elevatorAlgaeL3(elevator, wristSubsystem)
      );
    // Algae L2
    operatorController.elevator().algaeL2().and(coralSubystem.isSafeForElevator())
      .onTrue(
        CompositeCommands.elevatorAlgaeL2(elevator, wristSubsystem)
      );
    // Algae Processor
    operatorController.elevator().algaeProcessor().and(coralSubystem.isSafeForElevator())
      .onTrue(
        CompositeCommands.elevatorAlgaeProcessor(elevator, wristSubsystem)
      );
    
    //// Coral Delivery Setpoints
    // Coral L4
    operatorController.elevator().coralL4().and(coralSubystem.isSafeForElevator()).onTrue(CompositeCommands.elevatorCoralL4(elevator, wristSubsystem));
    // Coral L3
    operatorController.elevator().coralL3().and(coralSubystem.isSafeForElevator()).onTrue(CompositeCommands.elevatorCoralL3(elevator, wristSubsystem));
    // Coral L2
    operatorController.elevator().coralL2().and(coralSubystem.isSafeForElevator()).onTrue(CompositeCommands.elevatorCoralL2(elevator, wristSubsystem));
    // Coral L1
    operatorController.elevator().coralL1().and(coralSubystem.isSafeForElevator()).onTrue(CompositeCommands.elevatorCoralL1(elevator, wristSubsystem));
    
    //// Storage
    // TODO - this needs to be expanded to include the AlgaeCollector position, verify that CoralCollector is safe, etc
    operatorController.elevator().storage().and(coralSubystem.isSafeForElevator())
      .onTrue(
        CompositeCommands.elevatorStorage(elevator, wristSubsystem)
      );

    //////////  
    // Wrist

    //// Manual Controls
    operatorController.wrist().rotateDown().whileTrue(wristSubsystem.down());
    operatorController.wrist().rotateUp().whileTrue(wristSubsystem.up());
      
    //// Algae Collector Setpoints
    // Storage
    operatorController.algaeCollector().storage().onTrue(wristSubsystem.algaeStorage());
    // Ground
    operatorController.algaeCollector().ground().onTrue(wristSubsystem.algaeGround());
    // Reef
    operatorController.algaeCollector().reef().onTrue(wristSubsystem.algaeL2());
    // Algae in Posession
    operatorController.algaeCollector().held().onTrue(wristSubsystem.algaeHeld());

    //////////  
    // Algae Collector
    operatorController.algaeCollector().collect().whileTrue(algaeCollectorSubsystem.in()).onFalse(algaeCollectorSubsystem.stop());
    operatorController.algaeCollector().release().whileTrue(algaeCollectorSubsystem.out()).onFalse(algaeCollectorSubsystem.stop());
  }

  public void setRumble(double val) {
    driver.setRumble(RumbleType.kRightRumble, 1);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void rumblePeriodic() {
    if (DriverStation.isTeleopEnabled()) {
      switch (driverRumbleState) {
        case TARGET_FOUND:
          if (driverRumbleIntensity > 0.5) {
            driverRumbleIntensity = 0.2;
          } else {
            driverRumbleIntensity += 0.01;
          }
          break;

        case TARGET_LOCKED_ON:
          driverRumbleIntensity = 1;
          break;

        case TARGET_NONE:
        default:
          driverRumbleIntensity = 0;
          break;
      }
    } else {
      driverRumbleState = RumbleState.TARGET_NONE;
    }

    driver.setRumble(RumbleType.kBothRumble, driverRumbleIntensity);
  }
}
