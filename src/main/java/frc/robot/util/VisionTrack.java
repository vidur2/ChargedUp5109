package frc.robot.util;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;

public class VisionTrack {
    private NetworkTableInstance ntwrkInst;
    double[] empty = {0, 0, 0};
    private SwerveDrivePoseEstimator m_poseEstimator;
    private DoubleArraySubscriber m_subscriber;
    private long lastUpdate = 0;

    public VisionTrack(NetworkTableInstance ntwrkInst) {
        this.ntwrkInst = ntwrkInst;
    }

    public VisionTrack(NetworkTableInstance ntwrkInst, SwerveDrivePoseEstimator poseEstimator) {
        m_poseEstimator = poseEstimator;
        this.ntwrkInst = ntwrkInst;
        DoubleArraySubscriber subscriber = ntwrkInst.getTable(Constants.kTableInstance).getDoubleArrayTopic(Constants.kTableEntryPose).subscribe(empty);
        m_subscriber = subscriber;
    }

    public double[] getPose() {
        return ntwrkInst.getTable(Constants.kTableInstance).getEntry(Constants.kTableEntryPose).getDoubleArray(empty);
    }

    public Pose2d getPose2d(AHRS navX) {
        double[] pose = getPose();

        return new Pose2d(new Translation2d(pose[0], pose[1]), navX.getRotation2d());
    }

    public void updateVision(AHRS navX) {
        TimestampedDoubleArray arr = m_subscriber.getAtomic();
        if (arr.timestamp != 0) {
            Pose2d pose = new Pose2d(new Translation2d(arr.value[0], arr.value[1]), navX.getRotation2d());
            m_poseEstimator.addVisionMeasurement(pose, arr.timestamp);
        }
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
