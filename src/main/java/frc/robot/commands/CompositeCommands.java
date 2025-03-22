// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeCollectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.utils.KeyboardController.CoralCollector;
import frc.robot.utils.KeyboardController.Wrist;

/** Add your docs here. */
public class CompositeCommands {

    // General

    public static Command elevatorStorage(ElevatorSubsystem elevator, WristSubsystem wrist) {
        // if algae is held, set wrist to algaeHeld, otherwise algaeStorage
        // in either case elevator should not move until we know that the wrist is safe

        return new SequentialCommandGroup(
            wrist.algaeHeld(), // This command knows wether to go to "held" or "storage" based on algae hold state
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.storage()
        );
    }

    // Algae specific
    public static Command elevatorAlgaeProcessor(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new SequentialCommandGroup(
            wrist.algaeProcessor(),
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.algaeProcessor()
        );
    }

    public static Command elevatorAlgaeL2(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new SequentialCommandGroup(
            wrist.algaeL2(),
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.algaeL2()
        );
    }

    public static Command elevatorAlgaeL3(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new SequentialCommandGroup(
            wrist.algaeL3(),
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.algaeL3()
        );
    }

    public static Command elevatorBargeFront(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new SequentialCommandGroup(
            wrist.algaeBargeFront(),
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.algaeBarge()
        );
    }

    public static Command elevatorBargeBack(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new SequentialCommandGroup(
            wrist.algaeHeld(),
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.algaeBarge(),
            Commands.waitUntil(elevator.safeForBargeBack()),
            wrist.algaeBargeBack()
        );
    }

    public static Command elevatorAlgaeGround(ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new SequentialCommandGroup(
            wrist.algaeHeld(),
            Commands.waitUntil(wrist.safeForElevatorDownMovement()),
            elevator.storage(),
            wrist.algaeGround()
        );
    }
 

}
