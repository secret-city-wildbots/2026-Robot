package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Actors.Subsystems.Shooter;
import frc.robot.Commands.Shooter.ShootCommand;
import frc.robot.Commands.Shooter.HoodCommand;

public class HoodAndShootSequence extends ParallelCommandGroup {

    public HoodAndShootSequence(Shooter shooter, double shooterRPS, double hoodAngle_deg) {

        addCommands(

            // 1. Full extend
            new ShootCommand(shooter, shooterRPS, hoodAngle_deg)

            // 2. Pull down to handoff
            // new HoodCommand(shooter, hoodAngle_deg)
        );
    }

}