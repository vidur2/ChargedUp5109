// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;


import com.kauailabs.navx.frc.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.*;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {

 

  public static final double kMaxSpeed = 2.5; // 3 meters per second
  public static final double kMaxAngularSpeed = 3 * Math.PI; // 1/2 rotation per second

  // Network Table instantiation
  private final NetworkTableInstance ntwrkInst = NetworkTableInstance.getDefault();
  public NetworkTable ballAlignmentValues = ntwrkInst.getTable("ballAlignment");

  private RamseteController cRamseteController = new RamseteController();

  // Map to store velocities of robot and time
  private Translation2d lastKnownVelocity = new Translation2d(0, 0);
  private double lastKnownTime = 0;

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
  private Rotation2d initialMeasurement = new Rotation2d((navX.getYaw() % 360) * Math.PI / 180);

  // Shooter Range
  public double shooterRangeCm; // Enter shooter distance here (cm)

  // Swerve drive library instantiation
  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  // public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, initialMeasurement);

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
    m_frontLeft = new SwerveModule((int) swerveFrontLeftMotors[0], (int) swerveFrontLeftMotors[1],
        (int) swerveFrontLeftMotors[2], swerveFrontLeftMotors[3]);
    m_frontRight = new SwerveModule((int) swerveFrontRightMotors[0], (int) swerveFrontRightMotors[1],
        (int) swerveFrontRightMotors[2], swerveFrontRightMotors[3]);
    m_backLeft = new SwerveModule((int) swerveBackLeftMotors[0], (int) swerveBackLeftMotors[1],
        (int) swerveBackLeftMotors[2], swerveBackLeftMotors[3]);
    m_backRight = new SwerveModule((int) swerveBackRightMotors[0], (int) swerveBackRightMotors[1],
        (int) swerveBackRightMotors[2], swerveBackRightMotors[3]);
  }

  public void autoBalance() // auto balance for the charging station
  {
    float pitch = navX.getPitch();
    System.out.println(pitch);
    float speed = 0.5f; // m/s
    float deadzone = 2.5f; //degrees
    /*drive(speed, 0, 0, true);*/ 
    System.out.println(pitch);
    if (pitch > deadzone || pitch < -deadzone)
    {
      speed *= -pitch;
      drive(speed, 0, 0, true);
    }
    else
    {
      drive(0, 0, 0, true);
    } 
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
    this.driveChassisSpeed(wanted, true);
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
  // public void updateOdometry() {
  //   m_odometry.update(
  //       new Rotation2d(navX.getYaw() * 180 / Math.PI),
  //       m_frontLeft.getState(),
  //       m_frontRight.getState(),
  //       m_backLeft.getState(),
  //       m_backRight.getState());
  // }

  public void resetEncoders() {
    m_backRight.m_turningEncoderRelative.setPosition(0);
    m_backLeft.m_turningEncoderRelative.setPosition(0);
    m_frontLeft.m_turningEncoderRelative.setPosition(0);
    m_frontRight.m_turningEncoderRelative.setPosition(0);
  }


  // // Function to fake odometry calc
  // public Pose2d getRobotPose() {

  //   // Gets the current time 
  //   double currentTime = Timer.getFPGATimestamp();

  //   // Iterates through velocity HashMap backwards
  //   Double[] velocityArray = velocityMap.keySet().toArray(Double[]::new);
  //   for (int i = velocityArray.length - 1; i > 0; i--) {

  //     // Multiplies velocity * time to get distance
  //     double time = velocityArray[i];
  //     Translation2d velocityComp = velocityMap.get(time);
  //     double vx = velocityComp.getX();
  //     double vy = velocityComp.getY();
  //     globalX += (vx * (currentTime - time));
  //     globalY += (vy * (currentTime - time));
  //     currentTime = time;
  //   }

  //   // Adds the displacement from acceleration to constant velocity calculation
  //   double dispX = navX.getDisplacementX();
  //   double dispY = navX.getDisplacementY();

  //   if (Math.abs(dispX) > odometryLimiter) {
  //     globalY -= dispX;
  //   } 

  //   if (Math.abs(dispY) > odometryLimiter) {
  //     globalX -= dispY;
  //   }

  //   // Resets displacement and velocityMap
  //   navX.resetDisplacement();
  //   velocityMap.clear();

  //   // Returns RobotPose
  //   return new Pose2d(new Translation2d(globalX, globalY), new Rotation2d(-navX.getYaw()));
  // }

  public Translation2d getRobotPoseNavX() {
    Translation2d lastKnownCoord = new Translation2d(-navX.getDisplacementY(), -navX.getDisplacementX());
    double currentTime = Timer.getFPGATimestamp();
    Translation2d deltaTranslation  = lastKnownVelocity.times(Math.abs(currentTime-lastKnownTime));
    Translation2d finalTranslation = lastKnownCoord.plus(deltaTranslation);
    return finalTranslation;
  }

  /**
   * Initializes the listener for aligining to balls using the jetson nano program
   * Drives everytime the network tables is updated
   * 
   * @return Returns the id of the listener
   */
  // public int initBallListener() {
  //   NetworkTableEntry allianceEntry = ballAlignmentValues.getEntry("alliance");
  //   allianceEntry.setString(alliance.toString());
  //   int listenerHandle = ballAlignmentValues.addEntryListener("tVelocity", (table, key, entry, value, flags) -> {
  //     double[] velocity = value.getDoubleArray();
  //     double xVel = velocity[0];
  //     double yVel = velocity[1];

  //     if (xVel != 0 && yVel != 0) {
  //       drive(xVel, yVel, 0, true);
  //     }
  //   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
  //   return listenerHandle;
  // }

  public void customAutonAlign() {

    // double testCounter = 0;
    // double degreeOffset = 1;

    // double e_frontLeftPos = m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // double e_frontRightPos = m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    // double e_backRightPos = m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    // double e_backLeftPos = m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // // boolean outuput =Math.abs(e_backLeftPos) <= degreeOffset &&
    // // Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <=
    // // degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    // double startTime = Timer.getFPGATimestamp();

    // while (testCounter == 0) {
    //   double currTime = Timer.getFPGATimestamp();
    //   System.out.println(currTime - startTime);
    //   if (Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset
    //       && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset) {
    //     testCounter = 1;
    //   }
    //   e_frontLeftPos = m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    //   e_frontRightPos = m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    //   e_backRightPos = m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    //   e_backLeftPos = m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();

    //   // boolean output = Math.abs(e_backLeftPos) <= degreeOffset &&
    //   // Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <=
    //   // degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    //   // System.out.println(-1 * e_frontLeftPos/180);
    //   // System.out.println(-1 * e_frontRightPos/180);
    //   // System.out.println(-1 * e_backRightPos/180);
    //   // System.out.println(-1 * e_frontLeftPos/180);

    //   m_frontLeft.m_turningMotor.set(-1 * e_frontLeftPos / 180);
    //   m_backLeft.m_turningMotor.set(-1 * e_backLeftPos / 180);
    //   m_frontRight.m_turningMotor.set(-1 * e_frontRightPos / 180);
    //   m_backRight.m_turningMotor.set(-1 * e_backRightPos / 180);

    //   if (currTime - startTime > 3) {
    //     break;
    //   }
    // }

    // System.out.println("out of init loop");
  }

  public void customTeleopAlign() {

    // double testCounter = 0;
    // double degreeOffset = 10;

    // double e_frontLeftPos = m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // double e_frontRightPos = m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    // double e_backRightPos = m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    // double e_backLeftPos = m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    // // boolean outuput =Math.abs(e_backLeftPos) <= degreeOffset &&
    // // Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <=
    // // degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    // double startTime = Timer.getFPGATimestamp();

    // while (testCounter == 0) {
    //   double currTime = Timer.getFPGATimestamp();
    //   System.out.println(currTime - startTime);
    //   if (Math.abs(e_backLeftPos) <= degreeOffset && Math.abs(e_frontLeftPos) <= degreeOffset
    //       && Math.abs(e_frontRightPos) <= degreeOffset && Math.abs(e_backRightPos) <= degreeOffset) {
    //     testCounter = 1;
    //   }
    //   e_frontLeftPos = m_frontLeft.m_turningEncoderAbsolute.getAbsolutePosition();
    //   e_frontRightPos = m_frontRight.m_turningEncoderAbsolute.getAbsolutePosition();
    //   e_backRightPos = m_backRight.m_turningEncoderAbsolute.getAbsolutePosition();
    //   e_backLeftPos = m_backLeft.m_turningEncoderAbsolute.getAbsolutePosition();

    //   // boolean output = Math.abs(e_backLeftPos) <= degreeOffset &&
    //   // Math.abs(e_frontLeftPos) <= degreeOffset && Math.abs(e_frontRightPos) <=
    //   // degreeOffset && Math.abs(e_backRightPos) <= degreeOffset;

    //   // System.out.println(-1 * e_frontLeftPos/180);
    //   // System.out.println(-1 * e_frontRightPos/180);
    //   // System.out.println(-1 * e_backRightPos/180);
    //   // System.out.println(-1 * e_frontLeftPos/180);

    //   m_frontLeft.m_turningMotor.set(-1 * e_frontLeftPos / 180);
    //   m_backLeft.m_turningMotor.set(-1 * e_backLeftPos / 180);
    //   m_frontRight.m_turningMotor.set(-1 * e_frontRightPos / 180);
    //   m_backRight.m_turningMotor.set(-1 * e_backRightPos / 180);

    //   if (currTime - startTime > 3) {
    //     break;
    //   }
    // }

    // System.out.println("out of init loop");
  }

  public Optional<Pose2d> followTrajectory(Trajectory trajectory, double startTime, Pose2d prevPose) {
    // double startTime = Timer.getFPGATimestamp();

    double timeDiff = Timer.getFPGATimestamp() - startTime;

    // Pose2d prevPose = trajectory.getInitialPose();

    if (trajectory.getTotalTimeSeconds() > timeDiff) {
      State state = trajectory.sample(timeDiff);

      // Might be opposite depending on orientation of bot
      Translation2d dirVector = state.poseMeters.getTranslation().minus(prevPose.getTranslation());
      Rotation2d deltTheta = state.poseMeters.getRotation().minus(prevPose.getRotation());
      driveTo(state.velocityMetersPerSecond, deltTheta, dirVector, state.curvatureRadPerMeter);
      // prevPose = state.poseMeters;
      return Optional.of(state.poseMeters);
    }  else {
      return Optional.empty();
    }
  }


  public Optional<State> followTrajectory(Trajectory trajectory, double startTime, State prevState) {
    double timeDiff = Timer.getFPGATimestamp() - startTime;

    if (trajectory.getTotalTimeSeconds() > timeDiff) {
      State goal = trajectory.sample(timeDiff);
      ChassisSpeeds adjustedSpeeds = cRamseteController.calculate(prevState.poseMeters, goal);
      drive(adjustedSpeeds.vxMetersPerSecond, adjustedSpeeds.vyMetersPerSecond, adjustedSpeeds.omegaRadiansPerSecond, true);
      return Optional.of(goal);
    } else {
      drive(0, 0, 0, true);
      return Optional.empty();
    }
  }

  public void driveTo(double speed, Rotation2d rotationVector, Translation2d dirVector, double curvatureRadPerMeter) {
    double xRatio = dirVector.getX()/dirVector.getNorm();
    double yRatio = dirVector.getY()/dirVector.getNorm();

    drive(-yRatio * speed, -xRatio * speed, curvatureRadPerMeter * speed, false);
  }

  // public void autoAlignToGoalUsingLimelight() {
  //   OptionalDouble posRes = limelight.getXOffset();
  //   if (posRes.isPresent()) {
  //     double res = posRes.getAsDouble();
  //     auto.rotateTo(Rotation2d.fromDegrees(navX.getYaw() + res));
  //   } else {
  //     System.out.println("No object detected.");
  //   }

  // }
}
