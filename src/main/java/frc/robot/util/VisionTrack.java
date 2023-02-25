package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionTrack {
    private NetworkTableInstance ntwrkInst;

    public VisionTrack(NetworkTableInstance ntwrkInst) {
        this.ntwrkInst = ntwrkInst;
    }

    public double[] getPose() {
        double[] empty = {0, 0, 0};
        return ntwrkInst.getTable("SmartDashboard").getEntry("pose").getDoubleArray(empty);
    }
}
