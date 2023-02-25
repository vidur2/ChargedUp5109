package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionTrack {
    private NetworkTableInstance ntwrkInst;
    double[] empty = {0, 0, 0};

    public VisionTrack(NetworkTableInstance ntwrkInst) {
        this.ntwrkInst = ntwrkInst;
    }

    public double[] getPose() {
        return ntwrkInst.getTable(Constants.kTableInstance).getEntry(Constants.kTableEntryPose).getDoubleArray(empty);
    }

    public Translation3d[] getCubePoses() throws Exception
    {
        double[] doubleArray;
        doubleArray = ntwrkInst.getTable(Constants.kTableInstance).getEntry(Constants.kTableEntryCubePoses).getDoubleArray(empty);
        ArrayList<Translation3d> al= ArrayParser.getObjectPoses(doubleArray);
        Translation3d[] transArray = new Translation3d[al.size()];
        al.toArray(transArray);

        return transArray;
    }
}
