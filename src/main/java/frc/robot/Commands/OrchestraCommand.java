package frc.robot.Commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

public class OrchestraCommand extends Command {

    private final Orchestra _orchestra = new Orchestra();
    private int _timeToPlayLoops = 0;

    public OrchestraCommand() {
        TalonFX[] motors = new TalonFX[] {
            new TalonFX(47, "rio"),
            new TalonFX(48, "rio")
        };

        for (TalonFX motor : motors) {
            _orchestra.addInstrument(motor, 0);
        }
    }

    @Override
    public void initialize() {
        _orchestra.loadMusic("Tune.chrp");
        System.out.println("[Orchestra] loadMusic called");

        /* Schedule play after a delay — gives Orchestra time to parse the chirp file.
           Calling play() immediately after loadMusic can result in an invalid action error. */
        _timeToPlayLoops = 10;
    }

    @Override
    public void execute() {
        if (_timeToPlayLoops > 0) {
            --_timeToPlayLoops;
            if (_timeToPlayLoops == 0) {
                var status = _orchestra.play();
                System.out.println("[Orchestra] play() status: " + status + ", isPlaying: " + _orchestra.isPlaying());
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[Orchestra] end(), interrupted: " + interrupted);
        _orchestra.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
