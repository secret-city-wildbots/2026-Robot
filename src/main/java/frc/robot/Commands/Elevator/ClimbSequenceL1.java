package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;

public class ClimbSequenceL1 extends SequentialCommandGroup {

    public ClimbSequenceL1(ElevatorLift lift) {

        addCommands(

            // 1. Full extend
            new ExtendLiftCommand(lift),

            // 2. Pull down to handoff
            new RetractLiftCommand(lift, true)
        );
    }
    
}
