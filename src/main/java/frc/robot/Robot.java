// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.arm.Arm;
import frc.robot.arm.TargetExtension;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.vision.ScoringController;
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

  private final LightController m_lightController = new LightController(0, 50+73);
  
  private String m_autoSelected;

  // private Gripper m_gripper;
  private Arm m_arm = new Arm(11, 22, 0, true);
  
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_teleopChooser = new SendableChooser<>();
  private DriverModes m_driveMode = DriverModes.kWillMode;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1);

  private boolean m_reached = true;

  private final XboxController xController = new XboxController(0);
  private final Joystick jStick = new Joystick(1);

  private static final int[] frontLeftIds = { 15, 14 };// 100}; // back right?
  private static final int[] frontRightIds = { 12, 13 };// (180 - 55) - 360}; // back left?
  private static final int[] backLeftIds = { 18, 19 };// -5}; //front right?
  private static final int[] backRightIds = { 16, 17 };// (180 + 40) - 360}; //front left?y

  public final Drivetrain m_swerve = new Drivetrain(frontLeftIds, frontRightIds, backLeftIds,
  backRightIds);
  
  private boolean m_aligning = false;

  private ControllerState controllerState = new ControllerState();

  private Notifier m_placeNotifier;
  private Notifier m_resetNotifier;
  private Notifier m_pickupNotifier;
  private Notifier m_coneNotifier;
  private Notifier m_midNotifier;
  private Notifier m_rotateNotifier;

  private HashMap<Integer, Pose2d> m_scoringMap = new HashMap<>();
  private ScoringController m_scoringController = new ScoringController(m_scoringMap);
  
  // Auton variables
  private int m_autoCounter = 0;

  private void driveWithJoystick(boolean fieldRelative) {
    /**
     * Get desired X speed of chassis.
     * Inverted since Xbox joysticks return flipped values.
     * 
     **/
    final double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftY(), 0.2))
        * m_swerve.m_maxSpeed;
    /**
     * Get desired Y (strafe/sideways) speed of chassis.
     * Positive = left, negative = right.
     * XBox controllers return flipped values.
     **/
    final double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(xController.getLeftX(), 0.2))
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
    m_placeNotifier = new Notifier(new Runnable() {
      public void run() {
        m_arm.place();
      }
    });

    SmartDashboard.putNumber("autonThing", 1);

    m_pickupNotifier = new Notifier(new Runnable() {
      public void run() {
        m_arm.pickup();
      }
    });

    m_resetNotifier = new Notifier(new Runnable() {
      public void run() {
        m_arm.reset();
      }
    });

    m_coneNotifier = new Notifier(
      new Runnable() 
      {
        public void run() 
        {
          m_arm.pickupCone();
        }
      }
    );

    m_rotateNotifier = new Notifier(
      new Runnable() 
      {
        public void run() 
        {
          m_swerve.rotateToZero();
        }
      }
    );

    m_midNotifier = new Notifier(new Runnable() {
      public void run() {
        m_arm.place(TargetExtension.kMid);
      }
    });
    // m_swerve.m_frontLeft.m_driveEncoder.setPosition(0);
    // m_swerve.m_frontRight.m_driveEncoder.setPosition(0);
    // m_swerve.m_backRight.m_driveEncoder.setPosition(0);
    // m_swerve.m_backLeft.m_driveEncoder.setPosition(0);
    // m_swerve.navX.calibrate();
    // CameraServer.startAutomaticCapture();
    // CvSink cvSink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("Blur", 680, 420);
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
    SmartDashboard.putNumber("ff", m_arm.m_armFeedForward.calculate(Math.PI, 0));
    SmartDashboard.putNumber("autoCounter", m_autoCounter);
    SmartDashboard.putNumber("extenderPosition", m_arm.m_extenderEncoder.getPosition());
    // SmartDashboard.putNumber("fLeftEnc", m_swerve.m_frontLeft.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("fRightEnc", m_swerve.m_frontRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bLeftEnc", m_swerve.m_backRight.m_driveEncoder.getPosition());
    // SmartDashboard.putNumber("bRightEnc", m_swerve.m_backLeft.m_driveEncoder.getPosition());
    double[] pose = m_swerve.visionTrack.getPose();
    SmartDashboard.putNumber("x", pose[1]);
    SmartDashboard.putNumber("y", pose[0]);
    SmartDashboard.putNumber("z", pose[2]);
    m_swerve.navX.reset();
    SmartDashboard.putBoolean("reachedPosition", false);
    // m_swerve.m_odometry.resetPosition(m_swerve.navX.getRotation2d(), m_swerve.getPositions(), new Pose2d(new Translation2d(0, 0), m_swerve.navX.getRotation2d()));
    Pose2d position = m_swerve.m_poseEstimator.update(Rotation2d.fromDegrees(m_swerve.navX.getAngle()), m_swerve.getPositions());
    SmartDashboard.putNumber("x_odom", position.getX());
    SmartDashboard.putNumber("y_odom", position.getY());
    controllerState.addMethod(TeleopMethods.AutoBalance);
    m_scoringMap.put(6, new Pose2d(6.991, -3.8681, Rotation2d.fromDegrees(0)));
    m_scoringMap.put(7, new Pose2d(6.991, -2.5685, Rotation2d.fromDegrees(0)));
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
    SmartDashboard.putNumber("x", pose[1]);
    SmartDashboard.putNumber("y", pose[0]);
    SmartDashboard.putNumber("z", pose[2]);
    SmartDashboard.putNumber("rotatorPosition", m_arm.m_rotatorEncoder.getPosition());
    SmartDashboard.putNumber("pitch", m_swerve.navX.getPitch());
    SmartDashboard.putNumber("roll", m_swerve.navX.getRoll()- Constants.kNavXOffsetAlign);
    SmartDashboard.putNumber("extenderPosition", m_arm.m_extenderEncoder.getPosition());
    Pose2d position = m_swerve.m_poseEstimator.update(Rotation2d.fromDegrees(m_swerve.navX.getAngle()), m_swerve.getPositions());
    SmartDashboard.putNumber("x_odom", position.getX());
    SmartDashboard.putNumber("y_odom", position.getY());
    // m_swerve.ntwrkInst.getTable(Constants.kTableInstance).putValue("angle", NetworkTableValue.makeDouble(Rotation2d.fromDegrees().getRadians()));
  }

  Pose2d target;

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
    m_lightController.setAllianceColor(DriverStation.getAlliance());
    m_autoSelected = m_chooser.getSelected();
    m_swerve.navX.reset();
    m_swerve.navX.setAngleAdjustment(180);
    m_swerve.m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_swerve.navX.getAngle()), m_swerve.getPositions(), m_swerve.visionTrack.getPose2d());
    m_autoCounter = 0;
    target = new Pose2d(new Translation2d(0, -2), new Rotation2d(0));
    m_swerve.resetEncoders();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    Timer.delay(2);
    m_arm.initAuto();
    m_arm.m_gripper.grip();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    switch (m_autoSelected) {
      case kCustomAuto:
        SmartDashboard.putNumber("autonThing", 0);
        // Put custom auto code here
        switch (m_autoCounter) {
          case 0:
          m_arm.m_extenderController.setReference(Units.inchesToMeters(40), ControlType.kPosition);
          Timer.delay(0.5);
          m_autoCounter++;
          break;
          case 1:
          m_arm.place(TargetExtension.kHigh);
          m_swerve.drive(-1, 0, 0, true);
          Timer.delay(1);
          m_swerve.drive(0, 0, 0, true);
          Timer.delay(0.5);
          m_autoCounter++;
          break;
          case 2:
          m_arm.m_gripper.release();
          Timer.delay(2);
          m_autoCounter++;
          break;
          case 3:
            m_arm.m_gripper.grip();
            m_swerve.drive(2, 0, 0, true);
            Timer.delay(3);
            m_swerve.drive(0, 0, 0, true);
            m_arm.reset();
            m_autoCounter++;
            break;  
        //   case 0:
        // m_arm.m_extenderController.setReference(Units.inchesToMeters(40), ControlType.kPosition);
        // Timer.delay(0.5);
        // m_autoCounter++;
        // break;
        // case 1:
        // m_arm.place(TargetExtension.kHigh);
        // m_swerve.drive(-1, 0, 0, true);
        // Timer.delay(1);
        // m_swerve.drive(0, 0, 0, true);
        // Timer.delay(0.5);
        // m_autoCounter++;
        // break;
        // case 2:
        // m_arm.m_gripper.release();
        // Timer.delay(0.5);
        // m_autoCounter++;
        // break;
        // case 3:
        //   m_arm.m_gripper.grip();
        //   m_swerve.drive(2, 0, 0, true);
        //   m_arm.reset();
        //   m_autoCounter++;
        //   break;
        //   case 4:
        //   if (Math.abs(m_swerve.navX.getRoll())  - Constants.kNavXOffsetAlign > 4)  {
        //     m_swerve.drive(0, 0, 0, true);
        //     m_autoCounter++;
        //   }  
        //   break;
        //   case 5:
        //   boolean balanced = m_swerve.autoBalance();

        //   if (balanced) {
        //     m_autoCounter++;
        //   }
        //   break;
        //   case 6:
        //   m_lightController.setBalanceColor();
        }
        break;
      case kDefaultAuto:
      default:
      switch(m_autoCounter) {
        case 0:
        m_arm.m_extenderController.setReference(Units.inchesToMeters(40), ControlType.kPosition);
        Timer.delay(0.5);
        m_autoCounter++;
        break;
        case 1:
        m_arm.place(TargetExtension.kHigh);
        m_swerve.drive(-1, 0, 0, true);
        Timer.delay(1);
        m_swerve.drive(0, 0, 0, true);
        Timer.delay(0.5);
        m_autoCounter++;
        break;
        case 2:
        m_arm.m_gripper.release();
        Timer.delay(2);
        m_autoCounter++;
        break;
        case 3:
          m_arm.m_gripper.grip();
          m_swerve.drive(2, 0, 0, true);
          Timer.delay(3);
          m_swerve.drive(0, 0, 0, true);
          m_arm.reset();
          m_autoCounter++;
          break;
      }
    }

    SmartDashboard.putNumber("autoCounter", m_autoCounter);
  }

  boolean prevYButton, prev2Button, prevAButton, prevRightBumper = false;

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_reached = true;
    m_aligning = false;

    // Comment out these two lines
    m_arm.initAuto();
    m_swerve.navX.reset();  

    m_swerve.navX.setAngleAdjustment(180);
    switch (m_teleopChooser.getSelected()) {
      case kKevinMode:
        m_driveMode = DriverModes.kKevinMode;
        break;
      case kWillMode:
        m_driveMode = DriverModes.kWillMode;
        break;
    }
    m_swerve.m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_swerve.navX.getAngle()), m_swerve.getPositions(), m_swerve.visionTrack.getPose2d());
    // m_swerve.m_poseEstimator.resetPosition(Rotation2d.fromDegrees(m_swerve.navX.getAngle()), m_swerve.getPositions(), new Pose2d());
    m_swerve.resetEncoders();
  }
//-2.8

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_lightController.setKevinsBuggedOutBug((int)(-m_swerve.navX.getAngle()/2), false);
    m_swerve.setMaxSpeed(m_driveMode.checkState(xController));
    if (xController.getRightTriggerAxis() > 0.5)
    {
      m_arm.release();
    }
    else
    {
      m_arm.grip();
    }
    if (jStick.getRawButtonPressed(3))
    {
      m_pickupNotifier.startSingle(0.1);
    }
    if (jStick.getRawButtonPressed(2))
    {
      m_resetNotifier.startSingle(0.1);
    }
    if (jStick.getTriggerPressed())
    {
      m_placeNotifier.startSingle(0.1);
    }

    if (jStick.getRawButtonPressed(4)) {
      m_coneNotifier.startSingle(0.1);
    }
    if (xController.getYButtonPressed()) {
      m_rotateNotifier.startSingle(0.1);
    }
    if (jStick.getRawButtonPressed(5)) {
      m_midNotifier.startSingle(0.1);
    }
    if (m_scoringController.registerInput(jStick)) {
      // m_swerve.rotateToZero();
      m_reached = false;
    }

    if (xController.getAButtonPressed()) {
      m_reached = true;
    }
    if (!m_reached) {
      m_reached = m_swerve.driveTo(m_scoringController.continueInput());
    }else {
      driveWithJoystick(true);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // m_swerve.align();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_swerve.resetEncoders();
    m_lightController.setAllianceColor(DriverStation.getAlliance());
    m_arm.initAuto();
    // System.out.println(Units.inchesToMeters(49));
    // m_arm.m_extenderController.setReference(Units.inchesToMeters(49), ControlType.kPosition);
    // m_arm.m_rotatorController.setReference(0, ControlType.kPosition);
    m_swerve.navX.reset();
    m_swerve.navX.setAngleAdjustment(180);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    if (xController.getBButton()) {
      m_arm.m_extender.set(-0.3);
    } else if (xController.getAButton()) {
      m_arm.m_extender.set(0.3);
    } else {
      m_arm.m_extender.set(0);
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

  private boolean pickupStation() {
    boolean reached = m_swerve.driveTo(new Pose2d(new Translation2d(16/2, 6/2), new Rotation2d(Math.PI)));

    return reached;
  }
}
