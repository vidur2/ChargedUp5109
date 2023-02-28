// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.Constants;
import frc.robot.util.VisionTrack;

import com.kauailabs.navx.frc.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;


/** Represents a swerve drive style drivetrain. */
public class Drivetrain {

 

  public static final double kMaxSpeed = 10; // 3 meters per second
  public static final double kMaxAngularSpeed = 3 * Math.PI; // 1/2 rotation per second

  // Network Table instantiation
  private final NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();
  public VisionTrack visionTrack = new VisionTrack(ntwrkInst);

  // public NetworkTable ballAlignmentValues = ntwrkInst.getTable("ballAlignment");

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
  public SwerveDriveOdometry m_odometry;
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
  public Drivetrain(double shooterRange, double[] swerveFrontLeftMotors, double[] swerveFrontRightMotors,
      double[] swerveBackLeftMotors, double[] swerveBackRightMotors) {
    navX.reset();
    navX.resetDisplacement();
        
    // ntwrkInst.startClientTeam(5109);
    m_frontLeft = new SwerveModule((int) swerveFrontLeftMotors[0], (int) swerveFrontLeftMotors[1]);
    m_frontRight = new SwerveModule((int) swerveFrontRightMotors[0], (int) swerveFrontRightMotors[1]);
    m_backLeft = new SwerveModule((int) swerveBackLeftMotors[0], (int) swerveBackLeftMotors[1]);
    m_backRight = new SwerveModule((int) swerveBackRightMotors[0], (int) swerveBackRightMotors[1]);

    m_odometry = new SwerveDriveOdometry(m_kinematics, navX.getRotation2d(), getPositions());
  }
  private float prevPitch = 0.0f;
  private float integral = 0.0f;
  public boolean autoBalance() // auto balance for the charging station
  {
    boolean retVal = false;
    float pitch = navX.getPitch() - Constants.kNavXOffsetAlign; //pitch is offset by 2
    integral += pitch*0.01f;
    //System.out.println("Current Pitch: " + pitch);
    float P = 0.125f;
    float I = -0.0f;
    float D = 0.0f;
    float deadzone = 2.5f; //degrees (2.5 is max allowed on docs)
    integral = Math.max(integral, -P);

    if (pitch > deadzone || pitch < -deadzone)
    {
      float speed =  P * pitch + I * integral + D * (pitch - prevPitch)/0.02f;
      System.out.println("Speed: " + speed + " At Pitch: " + pitch);

      drive(-speed, 0, 0, Constants.kFieldRelative);
      retVal = false;
    }
    else
    {
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
    Rotation2d navXVal = new Rotation2d((-navX.getYaw() % 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navXVal)
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

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
    Rotation2d navXVal = new Rotation2d((navX.getYaw() % 360) * Math.PI / 180);
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(wanted.vxMetersPerSecond, wanted.vyMetersPerSecond,
                wanted.omegaRadiansPerSecond, navXVal)
            : wanted);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public Pose2d updateOdometry() {
    return m_odometry.update(
        new Rotation2d(navX.getYaw() * 180 / Math.PI),
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

}
