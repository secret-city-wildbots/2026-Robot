package frc.robot;

// Import Network Tables
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// Import Network Tables Publisher Types
import edu.wpi.first.networktables.DoubleArrayPublisher;

public class Dashboard {
    public NetworkTableInstance inst;
    public NetworkTable table;
    public DoubleArrayPublisher desiredSpeeds_rPs;
    public DoubleArrayPublisher actualSpeeds_rPs;

    public Dashboard() {
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("SmartDashboard");
        desiredSpeeds_rPs = table.getDoubleArrayTopic("desiredSpeeds_rPs").publish();
        actualSpeeds_rPs = table.getDoubleArrayTopic("actualSpeeds_rPs").publish();
    }
}
