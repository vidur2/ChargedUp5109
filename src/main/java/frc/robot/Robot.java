// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.arm.Arm;
import frc.robot.arm.TargetExtension;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.util.ButtonState;
import frc.robot.util.Constants;
import frc.robot.util.ControllerState;
import frc.robot.util.DriverModes;
import frc.robot.util.TeleopMethods;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";

  private static final String kKevinMode = "kKevinMode";
  private static final String kWillMode = "kWillMode";
  
  private String m_autoSelected;

  // private Gripper m_gripper;
  private Arm m_arm = new Arm(11, 22, 0, true);
  
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_teleopChooser = new SendableChooser<>();
  private DriverModes m_driveMode = DriverModes.kWillMode;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  private final XboxController xController = new XboxController(0);

  private static final int[] frontLeftIds = { 15, 14 };// 100}; // back right?
  private static final int[] frontRightIds = { 12, 13 };// (180 - 55) - 360}; // back left?
  private static final int[] backLeftIds = { 18, 19 };// -5}; //front right?
  private static final int[] backRightIds = { 16, 17 };// (180 + 40) - 360}; //front left?y

  public final Drivetrain m_swerve = new Drivetrain(frontLeftIds, frontRightIds, backLeftIds,
  backRightIds);
  
  private ControllerState controllerState = new ControllerState();
  
  // Auton variables
  private int m_autoCounter = 0;

  private void driveWithJoystick(boolean fieldRelative) {
    /**
     * Get desired X speed of chassis.
     * Inverted since Xbox joysticks return flipped values.
     * 
     **/
    final double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftY(), 0.08))
        * m_swerve.m_maxSpeed;
    /**
     * Get desired Y (strafe/sideways) speed of chassis.
     * Positive = left, negative = right.
     * XBox controllers return flipped values.
     **/
    final double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftX(), 0.12))
        * m_swerve.m_maxSpeed;
    /**
     * Get desired rotation speed of chassis.
     * Positive = left, negative = right.
     * Xbox returns positive when holding right by default.
     **/
    SmartDashboard.putNumber("driveX", xSpeed);
    SmartDashboard.putNumber("driveY", ySpeed);
    SmartDashboard.putBoolean("isClamping", m_arm.getClamping());
    final double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xController.getRightX(), 0.12))
        * frc.robot.drivetrain.Drivetrain.kMaxAngularSpeed; 
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Constants.kNavXOffsetAlign = m_swerve.navX.getRoll();
    // m_swerve.m_frontLeft.m_driveEncoder.setPosition(0);
    // m_swerve.m_frontRight.m_driveEncoder.setPosition(0);
    // m_swerve.m_backRight.m_driveEncoder.setPosition(0);
    // m_swerve.m_backLeft.m_driveEncoder.setPosition(0);
    // m_swerve.navX.calibrate();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    SmartDashboard.putBoolean("isClamping", m_arm.getClamping());
    m_chooser.addOption("My Auto", kCustomAuto);
    m_teleopChooser.setDefaultOption("kWillMode", kWillMode);
    m_teleopChooser.addOption("kKevinMode", kKevinMode);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putData("Teleop Choices", m_teleopChooser);
    SmartDashboard.putNumber("rotatorPosition", m_arm.m_rotatorEncoder.getPosition());
    SmartDashboard.putNumber("driveX", 0);
    SmartDashboard.putNumber("driveY", 0);
    SmartDashboard.putNumber("yaw", m_swerve.navX.getAngle());
    SmartDashboard.putNumber("speedOutput", 0);
    SmartDashboard.putNumber("roll", m_swerve.navX.getRoll() - Constants.kNavXOffsetAlign);
    SmartDashboard.putNumber("kevinMultiplier", 0.2);
    SmartDashboard.putNumber("speedOutput", 0);
    SmartDashboard.putNumber("autoCounter", m_autoCounter);
    SmartDashboard.putNumber("extenderPosition", m_arm.m_extenderEncoder.getPosition());
    // SmartDashboard.putNumber("fLeftEnc", m_swerve.m_frontLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("fRightEnc", m_swerve.m_frontRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bLeftEnc", m_swerve.m_backRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bRightEnc", m_swerve.m_backLeft.m_driveEncoder.getPosition());
    double[] pose = m_swerve.visionTrack.getPose();
    SmartDashboard.putNumber("x", pose[0]);
    SmartDashboard.putNumber("y", pose[1]);
    SmartDashboard.putNumber("z", pose[2]);
    m_swerve.navX.reset();
    SmartDashboard.putBoolean("reachedPosition", false);
    // m_swerve.m_odometry.resetPosition(m_swerve.navX.getRotation2d(), m_swerve.getPositions(), new Pose2d(new Translation2d(0, 0), m_swerve.navX.getRotation2d()));
    Pose2d position = m_swerve.m_poseEstimator.update(m_swerve.navX.getRotation2d(), m_swerve.getPositions());
    SmartDashboard.putNumber("x_odom", position.getX()*55);
    SmartDashboard.putNumber("y_odom", position.getY()*55);
    controllerState.addMethod(TeleopMethods.AutoBalance);
  
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_swerve.visionTrack.updateVision();
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("yaw", m_swerve.navX.getAngle());
    SmartDashboard.putNumber("fLeftEnc", m_swerve.m_frontLeft.m_driveEncoder.getPosition());
    SmartDashboard.putNumber("fRightEnc", m_swerve.m_frontRight.m_driveEncoder.getPosition());
    SmartDashboard.putNumber("bLeftEnc", m_swerve.m_backRight.m_driveEncoder.getPosition());
    SmartDashboard.putNumber("bRightEnc", m_swerve.m_backLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("pitch", m_swerve.navX.getPitch() - Constants.kNavXOffset);
    double[] pose = m_swerve.visionTrack.getPose();
    SmartDashboard.putNumber("x", pose[0]);
    SmartDashboard.putNumber("y", pose[1]);
    SmartDashboard.putNumber("z", pose[2]);
    SmartDashboard.putNumber("rotatorPosition", m_arm.m_rotatorEncoder.getPosition());
    SmartDashboard.putNumber("pitch", m_swerve.navX.getPitch());
    SmartDashboard.putNumber("roll", m_swerve.navX.getRoll()- Constants.kNavXOffsetAlign);
    SmartDashboard.putNumber("extenderPosition", m_arm.m_extenderEncoder.getPosition());
    Pose2d position = m_swerve.m_poseEstimator.update(m_swerve.navX.getRotation2d(), m_swerve.getPositions());
    SmartDashboard.putNumber("x_odom", position.getX());
    SmartDashboard.putNumber("y_odom", position.getY());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    Constants.kNavXOffsetAlign = m_swerve.navX.getRoll();
    m_autoSelected = m_chooser.getSelected();
    m_swerve.navX.setAngleAdjustment(180);
    m_autoCounter = 0;
    m_arm.initAuto();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        switch (m_autoCounter) {
          case 0:
            m_swerve.drive(3, 0, 0, Constants.kFieldRelative);
            if (Math.abs(m_swerve.navX.getRoll() - Constants.kNavXOffsetAlign) > 7) {
              m_swerve.drive(0, 0, 0, Constants.kFieldRelative);
              m_autoCounter++;
            }
            break;
          case 1:
            if (m_swerve.autoBalance(-1)) {
              m_autoCounter++;
            }
            break;
        }
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }

    SmartDashboard.putNumber("autoCounter", m_autoCounter);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_arm.initAuto();
    // m_swerve.navX.setAngleAdjustment(0);
    switch (m_teleopChooser.getSelected()) {
      case kKevinMode:
        m_driveMode = DriverModes.kKevinMode;
        break;
      case kWillMode:
        m_driveMode = DriverModes.kWillMode;
        break;
    }
    m_swerve.resetEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(Constants.kFieldRelative);
    handleDebounce(xController.getXButton(), TeleopMethods.AutoBalance);
    m_swerve.setMaxSpeed(m_driveMode.checkState(xController));
    // if (xController.getAButton()) {
    //   m_swerve.autoBalance(-1);
    // }

    if (xController.getXButton())
    {
      m_arm.release();
    }
    else
    {
      m_arm.grip();
    }
    if (xController.getYButton() == false && prevYButton == true)
    {
      m_arm.pickup(Rotation2d.fromRadians(-Math.PI));
    }
    if (xController.getBButton() == false && prevBButton == true)
    {
      m_arm.reset();
    }
    if (xController.getAButton() == false && prevAButton == true)
    {
      m_arm.place(TargetExtension.kHigh);
    }


    prevYButton = xController.getYButton();
    prevBButton = xController.getBButton();
    prevAButton = xController.getAButton();
    // m_swerve.drive(1, 0, 0, true);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    m_swerve.align();
  }
  boolean m_reached = false;
  
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_swerve.resetEncoders();
    // m_reached = false;
    // m_swerve.navX.setAngleAdjustment(0);
    // m_swerve.m_poseEstimator.resetPosition(m_swerve.navX.getRotation2d(), m_swerve.getPositions(), new Pose2d());
    m_arm.initAuto();
    // m_arm.m_rotatorEncoder.setPosition(0);
    // m_arm.m_extenderController.setReference(-0.6, com.revrobotics.CANSparkMax.ControlType.kPosition);
    // m_arm.m_rotatorController.setReference(0, com.revrobotics.CANSparkMax.ControlType.kPosition);
    Timer.delay(0.1);
    // m_arm.pickup(Rotation2d.fromRadians(-Math.PI));
  }

  boolean prevYButton, prevBButton, prevAButton = false;

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // m_arm.test(xController.getAButton());
    // m_arm.m_gripper.test(xController.getAButton());
    // if (xController.getAButton()) {
    //   m_arm.m_extender.set(-0.3);
    // } else {
    //   m_arm.m_extender.set(0);
    // }
    if (xController.getXButton())
    {
      m_arm.release();
    }
    else
    {
      m_arm.grip();
    }
    if (xController.getYButton() == false && prevYButton == true)
    {
      m_arm.pickup(Rotation2d.fromRadians(-Math.PI));
    }
    if (xController.getBButton() == false && prevBButton == true)
    {
      m_arm.reset();
    }
    if (xController.getAButton() == false && prevAButton == true)
    {
      m_arm.place(TargetExtension.kHigh);
    }

    prevYButton = xController.getYButton();
    prevBButton = xController.getBButton();
    prevAButton = xController.getAButton();
  }

  public void handleDebounce(boolean buttonPress, TeleopMethods method) {
    ButtonState state = controllerState.getMethodState(method);

    if (!state.pressed) {
      if (buttonPress && state.state) {
        m_swerve.autoBalance(1);
      } else if (xController.getXButton()) {
        controllerState.changeMethodState(TeleopMethods.AutoBalance);
      }

      state.pressed = true;
    } else {
      state.pressed = false;
    }
  }
}
