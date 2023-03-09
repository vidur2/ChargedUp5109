package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.IInit;
import frc.robot.util.ITest;

public class Arm implements ITest, IInit {
    public Gripper m_gripper;

    public CANSparkMax m_extender;
    public SparkMaxPIDController m_extenderController;
    public RelativeEncoder m_extenderEncoder;

    private CANSparkMax m_rotator;
    public SparkMaxPIDController m_rotatorController;
    public RelativeEncoder m_rotatorEncoder;


    private final double kExtenderGearRatio = 100;
    private final double kRotatorGearRatio = 4*4*3*1.857;
    private final double kPulleyRadiusInitial = .02;
    private final double kArmMinExtension = Units.inchesToMeters(38);
    private final double kArmMidExtension = 1.5;
    private final double kArmMaxExtension = Units.inchesToMeters(41);
    private final double kArmMaxRotation = 0;
    private final Translation2d kAxisofRotation = new Translation2d(0, 1.3335);
    private final double kConeHeight = 15.92/100;
    private final double kTreyHeight = 1;

    public Arm(int extenderChannel, int rotatorChannel, int gripperChannelForward, boolean isClamping) {
        m_gripper = new Gripper(gripperChannelForward, isClamping);
        m_extender = new CANSparkMax(extenderChannel, MotorType.kBrushless);
        m_extender.setInverted(true);
        m_extenderController = m_extender.getPIDController();
        setExtenderPid();
        m_extenderEncoder = m_extender.getEncoder();

        m_rotator = new CANSparkMax(rotatorChannel, MotorType.kBrushless);
        m_rotatorController = m_rotator.getPIDController();
        setRotatorPid();
        m_rotatorEncoder = m_rotator.getEncoder();

        m_extender.setIdleMode(IdleMode.kBrake);
        m_rotator.setIdleMode(IdleMode.kBrake);

        m_extenderEncoder.setPositionConversionFactor(kPulleyRadiusInitial * 2 * Math.PI/kExtenderGearRatio);
        m_rotatorEncoder.setPositionConversionFactor(2 * Math.PI/kRotatorGearRatio);
    }

    // Change later
    private void setExtenderPid() {
        m_extenderController.setP(1.6 * 4);
        m_extenderController.setI(0);
        m_extenderController.setD(0);
    }
    

    private void setRotatorPid() {
        m_rotatorController.setP(0.8);
        m_rotatorController.setI(0);
        m_rotatorController.setD(0);
    }

    public void reset() {
        // m_gripper.grip();
        m_extenderController.setReference(Units.inchesToMeters(38), ControlType.kPosition);
        m_rotatorController.setReference(-Math.PI/2, ControlType.kPosition);
    }

    public void pickupCone(double extension, TargetExtension target, boolean forward) {
        Translation2d targetPose = new Translation2d();
        switch (target) {
            case kHigh:
                targetPose = new Translation2d(Math.sqrt(Math.pow(extension, 2) - Math.pow(kConeHeight, 2)), kConeHeight + kTreyHeight).minus(kAxisofRotation);
                break;
            case kLow:
                targetPose = new Translation2d(Math.sqrt(Math.pow(extension, 2) - Math.pow(kConeHeight, 2)), kConeHeight).minus(kAxisofRotation);
                break;
            case kMid:
                return;
            default:
                return;
        }


        if (!forward) {
            targetPose = new Translation2d(-targetPose.getX(), targetPose.getY());
        }

        m_extenderController.setReference(extension, ControlType.kPosition);
        m_rotatorController.setReference(targetPose.getAngle().getRadians(), ControlType.kPosition);
    }

    public void pickup(Rotation2d theta) {
        // m_gripper.grip();
        m_rotatorController.setReference(theta.getRadians(), ControlType.kPosition);
        m_extenderController.setReference(Units.inchesToMeters(41), ControlType.kPosition);
        // m_gripper.release();
    }

    public Translation3d getGripperPosition(Translation2d robotPosition) {
        Translation2d gripperRelPose = new Translation2d(m_extenderEncoder.getPosition(), Rotation2d.fromRadians(m_rotatorEncoder.getPosition()));
        gripperRelPose = gripperRelPose.plus(kAxisofRotation);

        return new Translation3d(robotPosition.getX() + gripperRelPose.getX(), robotPosition.getY(), gripperRelPose.getY());
    }

    public void place(TargetExtension target) {
        m_gripper.grip();
        m_rotatorController.setReference(0, ControlType.kPosition);
        switch (target) {
            case kHigh:
                m_extenderController.setReference(Units.inchesToMeters(41), ControlType.kPosition);
                break;
            case kMid:
                m_extenderController.setReference(1.5, ControlType.kPosition);
                break;
            case kLow:
                break;
        }
        m_gripper.release();
    }

    @Override
    public void test(boolean buttonPress) {
        // if (buttonPress)
        // {
        //     place(TargetExtension.kHigh);
        // }
        // else
        // {
        //     reset();
        // }
        if (buttonPress) {
            m_rotator.set(0.3);
        } else {
            m_rotator.set(0);
        }
    }

    @Override
    public void initAuto() {

        m_rotatorEncoder.setPosition(-Math.PI/2);
        m_extenderEncoder.setPosition(kArmMinExtension);
        
        m_gripper.initAuto();
    }

    @Override
    public void initTeleop() {
        m_gripper.initTeleop();
        return;
    }

    public void grip() {
        m_gripper.grip();
    }

    public void release() {
        m_gripper.release();
    }

    public boolean getClamping() {
        return m_gripper.getClamping();
    }
}
