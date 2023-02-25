package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Gripper {

    // fields of class
    private boolean isClamping;
    private DoubleSolenoid m_gripperSol;

    public Gripper(int mGripperPortForward, int mGripperPortReverse, boolean isClamping) {
        // Instantiate stuff
        m_gripperSol = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, mGripperPortForward, mGripperPortReverse);

        if (isClamping) {
            m_gripperSol.set(Value.kForward);
        } else {
            m_gripperSol.set(Value.kReverse);
        }
    }

    public boolean getClamping() {
        return this.isClamping;
    }

    public void release() {
        m_gripperSol.set(Value.kReverse);
        isClamping = false;
    }

    public void grip() {
        m_gripperSol.set(Value.kForward);
        isClamping = true;
    }
}