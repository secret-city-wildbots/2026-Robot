package frc.robot.Commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
// Import WPILib Commands
import edu.wpi.first.wpilibj2.command.Command;

// Import Actors, Utils & Constants
import frc.robot.Actors.Subsystems.Indexer.Indexer;
import frc.robot.Actors.Subsystems.Indexer.Transfer;
import frc.robot.Actors.Subsystems.Intake.Intake;
import frc.robot.Actors.Subsystems.Shooter.Shooter;

public class ClearTransferCommand extends Command {

    // Initialize the subsystems
    private final Transfer transfer;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private Timer timer;

    /**
     * Creates and sets up the SpinFuelCommand
     * 
     * @param transfer The subsystem to be controlled by the command ({@link Transfer})
     * @param indexer The subsystem to be controlled by the command ({@link Indexer})
     * @param transferRPS The rps for the transfer
     * @param indexerRPS The rps for the indexer
     */
    public ClearTransferCommand(
        Transfer transfer,
        Indexer indexer,
        Intake intake,
        Shooter shooter
        ) {
        // Set the subystems
        this.transfer = transfer;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.timer = new Timer();

        // Add subsystem requirements
        addRequirements(transfer, indexer, intake, shooter);
    }

    @Override
    public void initialize() {
        // Start the transfer motor and reset the timer
        //?
        transfer.setRPS(-30.0);
        indexer.setRPS(0.0, -30.0);
        intake.set(-0.4);
        shooter.setRPS(-10);
        shooter.setHoodAngle(10.0);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (timer.get() > 0.2) {
            shooter.setRPS(30);
            transfer.setRPS(50.0);
            indexer.setRPS(60.0, -30.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        transfer.setRPS(0.0);
        indexer.setRPS(0.0, 0.0);
        intake.stop();
        shooter.setRPS(0.0);
        shooter.setHoodAngle(10.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
