// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Actors.Subsystems.Indexer;
import frc.robot.Actors.Subsystems.Shooter;
import frc.robot.Actors.Subsystems.Turret;
import frc.robot.Commands.Shooter.AimAtHubCommand;

public class RobotContainer {

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final Turret turret;

    public final Shooter shooter;

    public final Indexer indexer;

    public RobotContainer() {
        this.turret = new Turret();
        this.indexer = new Indexer();
        this.shooter = new Shooter();
        configureBindings();
    }

    private void configureBindings() {
        //turret.setDefaultCommand(new JoystickAimCommand(turret, joystick)); //use for tuning turret
        shooter.setDefaultCommand(new AimAtHubCommand(shooter, turret, null, null));

        joystick.rightTrigger(0.6).whileTrue(
                new ParallelCommandGroup(
                        new frc.robot.Commands.Indexer.ShootCommand(indexer),
                        new frc.robot.Commands.Shooter.ShootCommand(shooter, 30.0)
                ));
    }
}
