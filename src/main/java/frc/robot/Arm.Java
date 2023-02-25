package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.TargetExtension;

public class Arm {
    private Gripper m_gripper;

    private CANSparkMax m_extender;
    private SparkMaxPIDController m_extenderController;
    private RelativeEncoder m_extenderEncoder;

    private CANSparkMax m_rotator;
    private SparkMaxPIDController m_rotatorController;
    private RelativeEncoder m_rotatorEncoder;


    private static final double kExtenderGearRatio = 1;
    private static final double kRotatorGearRatio = 4*4*3*1.857;
    private static final double kPulleyRadiusInitial = .02;
    private static final double kArmMinExtension = 1;
    private static final double kArmMidExtension = 1.5;
    private static final double kArmMaxExtension = 2;
    private static final double kArmMaxRotation = Math.PI/3;
    private static final Translation2d kAxisofRotation = new Translation2d(0, 1.3335);

    public Arm(int extenderChannel, int rotatorChannel, int gripperChannelForward, int gripperChannelReverse, boolean isClamping) {
        m_gripper = new Gripper(gripperChannelForward, gripperChannelReverse, isClamping);
        m_extender = new CANSparkMax(extenderChannel, MotorType.kBrushless);
        m_extenderController = m_extender.getPIDController();
        setExtenderPid();
        m_extenderEncoder = m_extender.getEncoder();

        m_rotator = new CANSparkMax(rotatorChannel, MotorType.kBrushless);
        m_rotatorController = m_rotator.getPIDController();
        setRotatorPid();
        m_rotatorEncoder = m_rotator.getEncoder();
    
        // Converts to meters?
        m_extenderEncoder.setPositionConversionFactor(kExtenderGearRatio * kPulleyRadiusInitial * 2 * Math.PI);
        m_rotatorEncoder.setPositionConversionFactor(kRotatorGearRatio * 2 * Math.PI);

        m_rotatorEncoder.setPosition(-Math.PI/2);
        m_extenderEncoder.setPosition(kArmMinExtension);
    }

    public Translation3d getGripperPosition(Translation2d robotPosition) {
        Translation2d gripperRelPose = new Translation2d(m_extenderEncoder.getPosition(), Rotation2d.fromRadians(m_rotatorEncoder.getPosition()));
        gripperRelPose = gripperRelPose.plus(kAxisofRotation);

        return new Translation3d(robotPosition.getX() + gripperRelPose.getX(), robotPosition.getY(), gripperRelPose.getY());
    }

    public void place(TargetExtension target) {
        m_rotatorController.setReference(kArmMaxRotation, ControlType.kPosition);
        switch (target) {
            case kHigh:
                m_extenderController.setReference(kArmMaxExtension, ControlType.kPosition);
                break;
            case kMid:
                m_extenderController.setReference(kArmMidExtension, ControlType.kPosition);
                break;
            case kLow:
                break;
        }
        m_gripper.release();
    }

    public void reset() {
        m_extenderController.setReference(kArmMinExtension, ControlType.kPosition);
        m_rotatorController.setReference(-Math.PI/2, ControlType.kPosition);
    }

    public void pickup(Rotation2d theta) {
        m_rotatorController.setReference(theta.getRadians(), ControlType.kPosition);
        m_extenderController.setReference(kArmMaxExtension, ControlType.kPosition);
        m_gripper.grip();
        reset();
    }

    // Change later
    private void setExtenderPid() {
        m_extenderController.setP(0.1);
        m_extenderController.setI(0);
        m_extenderController.setD(0);
    }
    

    private void setRotatorPid() {
        m_extenderController.setP(0.1);
        m_extenderController.setI(0);
        m_extenderController.setD(0);
    }
}