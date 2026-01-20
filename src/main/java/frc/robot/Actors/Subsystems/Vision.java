package frc.robot.Actors.Subsystems;

import frc.robot.Constants.VisionConstants;
import frc.robot.Utils.Limelight;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final List<Limelight> limelights = new ArrayList<>();

    public Vision() {
        for (String limelightID : VisionConstants.limelightNames) {
            limelights.add(new Limelight(limelightID));
        }
        
    }
     
}
