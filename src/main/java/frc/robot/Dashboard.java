package frc.robot;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
