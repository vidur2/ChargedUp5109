// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Constants;
import frc.robot.util.VisionTrack;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {

 
  public double m_speedMultiplier = 1;
  public double m_maxSpeed = m_speedMultiplier * Constants.kMaxSpeed; // 3 meters per second
  public static final double kMaxAngularSpeed = 3 * Math.PI; // 1/2 rotation per second

  // Network Table instantiation
  private final NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();
  public VisionTrack visionTrack;

  // public NetworkTable ballAlignmentValues = ntwrkInst.getTable("ballAlignment");

  private final float balanceP = .172f;
  private final float balanceI = 0.0f;
  private final float balanceD = 1.249f;

  // Bot measurements
  private final Translation2d m_frontLeftLocation = new Translation2d(0.2921, 0.2921);
  private final Translation2d m_frontRightLocation = new Translation2d(0.2921, -0.2921);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.2921, 0.2921);
  private final Translation2d m_backRightLocation = new Translation2d(-0.2921, -0.2921);
  private static final DriverStation.Alliance alliance = DriverStation.getAlliance();

  // Swerve Module instantiation
  public SwerveModule m_frontLeft;
  public SwerveModule m_frontRight;
  public SwerveModule m_backLeft;
  public SwerveModule m_backRight;
  public AHRS navX = new AHRS(SPI.Port.kMXP);
  public SwerveDrivePoseEstimator m_poseEstimator;

  // Holonomic drive controlling stuff
  HolonomicDriveController m_holonomicDriveController = new HolonomicDriveController(new PIDController(1, 0, 0), new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(2 * Math.PI, Math.PI)));

  // Shooter Range
  public double shooterRangeCm; // Enter shooter distance here (cm)

  // Swerve drive library instantiation
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, navX.getRotation2d(), getPositions());

  /**
   * Constructor for the swerve drivetrain
   * 
   * @param shooterRange           The optimal shooting distance in cm for the
   *                               robot (used for auto aligment with limelight)
   * @param swerveFrontLeftMotors  The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   * @param swerveFrontRightMotors The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   * @param swerveBackLeftMotors   The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   * @param swerveBackRightMotors  The CAN ids for the swerve modules. The first
   *                               element in the array should be the drive motor
   *                               id, the second should be the turning motor id
   */
  public Drivetrain(int[] swerveFrontLeftMotors, int[] swerveFrontRightMotors,
      int[] swerveBackLeftMotors, int[] swerveBackRightMotors) {
    navX.reset();
    navX.resetDisplacement();
    m_frontLeft = new SwerveModule(swerveFrontLeftMotors[0], swerveFrontLeftMotors[1]);
    m_frontRight = new SwerveModule(swerveFrontRightMotors[0], swerveFrontRightMotors[1]);
    m_backLeft = new SwerveModule(swerveBackLeftMotors[0], swerveBackLeftMotors[1]);
    m_backRight = new SwerveModule(swerveBackRightMotors[0], swerveBackRightMotors[1]);
    visionTrack = new VisionTrack(ntwrkInst, m_poseEstimator, navX);
    m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, navX.getRotation2d(), getPositions(), visionTrack.getPose2d());
    m_holonomicDriveController.setTolerance(new Pose2d(0.2, 0.2, Rotation2d.fromDegrees(1)));
  }

  private float prevPitch = 0.0f;
  private float integral = 0.0f;
  public boolean autoBalance(double scalar) // auto balance for the charging station
  {
    boolean retVal = false;
    float pitch = navX.getRoll() - Constants.kNavXOffsetAlign; //pitch is offset by 2
    integral += pitch*0.01f;
    //System.out.println("Current Pitch: " + pitch);
    
    float deadzone = .2f; //degrees (2.5 is max allowed on docs)
    integral = Math.max(integral, 0.1f);

    if (Math.abs(pitch) > deadzone)
    {
      float speed =  balanceP * pitch + balanceI * integral + balanceD * (pitch - prevPitch);
      SmartDashboard.putNumber("speedOutput", speed);
      System.out.printf("(%f, %f, %f, %f)\n", speed, pitch, prevPitch, pitch - prevPitch);
      drive(-scalar * speed, 0, 0, Constants.kFieldRelative);
      retVal = false;
    }
    else
    {
      System.out.printf("else: (%f, %f)\n", pitch, prevPitch);
      drive(0, 0, 0, Constants.kFieldRelative);
      integral = 0.0f;
      prevPitch = 0.0f;
      retVal = true;
    }
    prevPitch = pitch;

    return retVal;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    Rotation2d navXVal = new Rotation2d((-navX.getAngle() % 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navXVal)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);

    // for (SwerveModuleState state: swerveModuleStates) {
    // System.out.println(state);
    // }
    // System.out.println("1");
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // System.out.println("2");
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    // System.out.println("3");
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    // System.out.println("4");
    m_backRight.setDesiredState(swerveModuleStates[3]);
    // System.out.println(swerveModuleStates[0].speedMetersPerSecond);
  }

  public void driveChassisSpeed(ChassisSpeeds wanted) {
    this.driveChassisSpeed(wanted, Constants.kFieldRelative);
  }

  public void driveChassisSpeed(ChassisSpeeds wanted, boolean fieldRelative) {
    Rotation2d navXVal = new Rotation2d((navX.getAngle()% 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(wanted.vxMetersPerSecond, wanted.vyMetersPerSecond,
                wanted.omegaRadiansPerSecond, navXVal)
            : wanted);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, m_maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public Pose2d updateOdometry() {
    return m_poseEstimator.update(
        new Rotation2d(navX.getAngle()* 180 / Math.PI),
        getPositions());
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState(),
    };

    return positions;
  }

  public void resetEncoders() {
    m_backRight.m_turningEncoderRelative.setPosition(0);
    m_backLeft.m_turningEncoderRelative.setPosition(0);
    m_frontLeft.m_turningEncoderRelative.setPosition(0);
    m_frontRight.m_turningEncoderRelative.setPosition(0);
  }


  public void setMaxSpeed(double speed) {
    m_maxSpeed = speed;
  }

  public boolean driveTo(Pose2d target) {
    ChassisSpeeds speed = m_holonomicDriveController.calculate(m_poseEstimator.getEstimatedPosition(), target, 0, target.getRotation());
    driveChassisSpeed(speed);
    return m_holonomicDriveController.atReference();
  }

  public void align() {
    m_backLeft.alignWheel();
    m_backRight.alignWheel();
    m_frontLeft.alignWheel();
    m_frontRight.alignWheel();
  }
}
