// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.ConstantBootstraps;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.ButtonState;
import frc.robot.util.Constants;
import frc.robot.util.ControllerState;
import frc.robot.util.TeleopMethods;
import frc.robot.util.VisionTrack;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  private final XboxController xController = new XboxController(0);

  private static final double[] frontLeftIds = { 15, 14 };// 100}; // back right?
  private static final double[] frontRightIds = { 12, 13 };// (180 - 55) - 360}; // back left?
  private static final double[] backLeftIds = { 18, 19 };// -5}; //front right?
  private static final double[] backRightIds = { 16, 17 };// (180 + 40) - 360}; //front left?y

  public final Drivetrain m_swerve = new Drivetrain(0, frontLeftIds, frontRightIds, backLeftIds,
  backRightIds);
  
  private ControllerState controllerState = new ControllerState();



  private void driveWithJoystick(boolean fieldRelative) {
    /**
     * Get desired X speed of chassis.
     * Inverted since Xbox joysticks return flipped values.
     * 
     **/
    final double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftY(), 0.08))
        * frc.robot.Drivetrain.kMaxSpeed;
    /**
     * Get desired Y (strafe/sideways) speed of chassis.
     * Positive = left, negative = right.
     * XBox controllers return flipped values.
     **/
    final double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftX(), 0.08))
        * frc.robot.Drivetrain.kMaxSpeed;
    /**
     * Get desired rotation speed of chassis.
     * Positive = left, negative = right.
     * Xbox returns positive when holding right by default.
     **/
    final double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(xController.getRightX(), 0.12))
        * frc.robot.Drivetrain.kMaxAngularSpeed; 
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Constants.kNavXOffsetAlign = m_swerve.navX.getPitch();
    // m_swerve.m_frontLeft.m_driveEncoder.setPosition(0);
    // m_swerve.m_frontRight.m_driveEncoder.setPosition(0);
    // m_swerve.m_backRight.m_driveEncoder.setPosition(0);
    // m_swerve.m_backLeft.m_driveEncoder.setPosition(0);
    // m_swerve.navX.calibrate();
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("yaw", m_swerve.navX.getAngle());
    // SmartDashboard.putNumber("fLeftEnc", m_swerve.m_frontLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("fRightEnc", m_swerve.m_frontRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bLeftEnc", m_swerve.m_backRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bRightEnc", m_swerve.m_backLeft.m_driveEncoder.getPosition());
    double[] pose = m_swerve.visionTrack.getPose();
    SmartDashboard.putNumber("x", pose[0]);
    SmartDashboard.putNumber("y", pose[1]);
    SmartDashboard.putNumber("z", pose[2]);
    m_swerve.navX.reset();
    // m_swerve.m_odometry.resetPosition(m_swerve.navX.getRotation2d(), m_swerve.getPositions(), new Pose2d(new Translation2d(0, 0), m_swerve.navX.getRotation2d()));
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
    SmartDashboard.putData("Auto choices", m_chooser);
    SmartDashboard.putNumber("yaw", m_swerve.navX.getYaw());
    // SmartDashboard.putNumber("fLeftEnc", m_swerve.m_frontLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("fRightEnc", m_swerve.m_frontRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bLeftEnc", m_swerve.m_backRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bpRightEnc", m_swerve.m_backLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("pitch", m_swerve.navX.getPitch() - Constants.kNavXOffset);
    double[] pose = m_swerve.visionTrack.getPose();
    SmartDashboard.putNumber("x", pose[0]);
    SmartDashboard.putNumber("y", pose[1]);
    SmartDashboard.putNumber("z", pose[2]);
    SmartDashboard.putNumber("pitch", m_swerve.navX.getPitch());
    SmartDashboard.putNumber("roll", m_swerve.navX.getRoll());
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_swerve.resetEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    driveWithJoystick(Constants.kFieldRelative);
    handleDebounce(xController.getXButton(), TeleopMethods.AutoBalance);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_swerve.resetEncoders();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (xController.getAButton()) {
      m_swerve.autoBalance();
    }
    else if (xController.getBButton()) {
      m_swerve.drive(1, 0, 0, Constants.kFieldRelative);
    }
    else {
      driveWithJoystick(Constants.kFieldRelative);
    }
  }

  public void handleDebounce(boolean buttonPress, TeleopMethods method) {
    ButtonState state = controllerState.getMethodState(method);

    if (!state.pressed) {
      if (buttonPress && state.state) {
        m_swerve.autoBalance();
      } else if (xController.getXButton()) {
        controllerState.changeMethodState(TeleopMethods.AutoBalance);
      }

      state.pressed = true;
    } else {
      state.pressed = false;
    }
  }
}
