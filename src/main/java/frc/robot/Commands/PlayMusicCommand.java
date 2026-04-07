package frc.robot.Commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

// Import WPILib Commands

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Actors.Subsystems.CommandSwerveDrivetrain;
// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Spindexer.Spindexer;
import frc.robot.Actors.Subsystems.Spindexer.Transfer;

public class PlayMusicCommand extends Command {

    // Initialize the subsystems
    private final CommandSwerveDrivetrain drivetrain;
    private Orchestra orchestra;


    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param spindexer The subsystem to be controlled by the command ({@link Spindexer})
     * @param transferRPS The rps for the transfer
     * @param spindexerRPS The rps for the spindexer
     */
    public PlayMusicCommand(
        CommandSwerveDrivetrain drivetrain
        ) {
        // Set the subystems
        this.drivetrain = drivetrain;

        // Add subsystem requirements
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        System.out.println("[PlayMusic] initialize() called");

        // Create a fresh orchestra each time to avoid duplicate instrument accumulation
        orchestra = new Orchestra("Drivetrain");

        // Load music FIRST — loadMusic resets instrument assignments in CTRE Orchestra
        var loadStatus = orchestra.loadMusic("Tune.chrp");
        System.out.println("[PlayMusic] loadMusic status: " + loadStatus);

        TalonFX[] motors = new TalonFX[] {
            // drivetrain.getModule(0).getDriveMotor(),
            // drivetrain.getModule(1).getDriveMotor(),
            // drivetrain.getModule(2).getDriveMotor(),
            // drivetrain.getModule(3).getDriveMotor(),
            // drivetrain.getModule(0).getSteerMotor(),
            // drivetrain.getModule(1).getSteerMotor(),
            // drivetrain.getModule(2).getSteerMotor(),
            // drivetrain.getModule(3).getSteerMotor()
            new TalonFX(47, "rio"),
            new TalonFX(48, "rio")
        };

        String[] motorNames = new String[] {
            "Drive[0]", "Drive[1]", "Drive[2]", "Drive[3]",
            "Steer[0]", "Steer[1]", "Steer[2]", "Steer[3]"
        };

        // All motors on track 0 — same note simultaneously = louder output
        for (int i = 0; i < motors.length; i++) {
            var addStatus = orchestra.addInstrument(motors[i], 0);
            System.out.println("[PlayMusic] " + motorNames[i] + " (deviceID=" + motors[i].getDeviceID() + ") -> track 0 | status: " + addStatus);
        }

        var playStatus = orchestra.play();
        System.out.println("[PlayMusic] play() status: " + playStatus + ", isPlaying: " + orchestra.isPlaying());


    }

    @Override
    public void execute() {
        System.out.println("[PlayMusic] isPlaying: " + orchestra.isPlaying());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[PlayMusic] end() called, interrupted: " + interrupted);
        orchestra.stop();
    }

    @Override
    public boolean isFinished() {
        // whileTrue handles cancellation on button release; no need to self-finish
        return false;
    }
    
}
