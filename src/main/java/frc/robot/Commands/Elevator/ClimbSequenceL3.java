package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Actors.Subsystems.Elevator.ElevatorLift;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Actors.Subsystems.Elevator.ElevatorHook;

public class ClimbSequenceL3 extends SequentialCommandGroup {

    public ClimbSequenceL3(ElevatorLift lift, ElevatorHook hook) {

        addCommands(

            // 1. Full extend
            new ExtendLiftCommand(lift),

            // 2. Pull down to handoff
            new RetractLiftCommand(lift, true),

            // 3. Rotate hooks out
            new RotateHookToPositionCommand(hook, ElevatorConstants.hookDeployedPosition),

            // 4. Full extend
            new ExtendLiftCommand(lift),

            // 5. Pull down AND rotate hooks safe (parallel)
            new ParallelCommandGroup(
                new RetractLiftCommand(lift, true),
                new RotateHookToPositionCommand(hook, ElevatorConstants.hookSafePosition)
            ),

            // 6. Rotate hooks out
            new RotateHookToPositionCommand(hook, ElevatorConstants.hookDeployedPosition),

            // 7. Full extend
            new ExtendLiftCommand(lift),

            // 8. Pull down to handoff
            new ParallelCommandGroup(
                new RetractLiftCommand(lift, true),
                new RotateHookToPositionCommand(hook, ElevatorConstants.hookSafePosition))
        );
    }
    
}
