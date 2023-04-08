package frc.robot.drivetrain.vision;

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
import frc.robot.drivetrain.Drivetrain;
import frc.robot.util.ArrayParser;
import frc.robot.util.Constants;

public class VisionTrack {
    private NetworkTableInstance ntwrkInst;
    double[] empty = {0, 0, 0};
    public SwerveDrivePoseEstimator m_poseEstimator;
    private DoubleArraySubscriber m_subscriber;
    private long lastUpdate = 0;
    private AHRS navX;

    public VisionTrack(NetworkTableInstance ntwrkInst) {
        this.ntwrkInst = ntwrkInst;
    }

    public VisionTrack(NetworkTableInstance ntwrkInst, SwerveDrivePoseEstimator poseEstimator, AHRS navX) {
        m_poseEstimator = poseEstimator;
        this.ntwrkInst = ntwrkInst;
        this.navX = navX;
        DoubleArraySubscriber subscriber = ntwrkInst.getTable(Constants.kTableInstance).getDoubleArrayTopic(Constants.kTableEntryPose).subscribe(empty);
        m_subscriber = subscriber;
    }

    public double[] getPose() {
        return ntwrkInst.getTable(Constants.kTableInstance).getEntry(Constants.kTableEntryPose).getDoubleArray(empty);
    }

    public Pose2d getPose2d() {
        double[] pose = getPose();

        return new Pose2d(new Translation2d(pose[1], pose[0]), navX.getRotation2d());
    }

    public void updateVision(Drivetrain m_swerve, boolean isNull) {
        TimestampedDoubleArray arr = m_subscriber.getAtomic();
        // System.out.printf("%b, %b, %b\n", isNull, arr.timestamp != 0, arr.timestamp != lastUpdate);
        if (arr.timestamp != 0 && arr.timestamp != lastUpdate && m_poseEstimator != null) {
            Pose2d pose = new Pose2d(new Translation2d(arr.value[1], arr.value[0]), navX.getRotation2d());
            m_poseEstimator.addVisionMeasurement(pose, arr.timestamp * Math.pow(10, -6));
            lastUpdate = arr.timestamp;
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
