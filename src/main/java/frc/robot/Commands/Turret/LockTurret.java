package frc.robot.Commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.Shooter.Turret;

public class LockTurret extends Command {
    public Turret turret;

    public LockTurret(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        this.turret.setTargetAngle(this.turret.getTurretAngle());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Do not end the command
        return false;
    }
}
