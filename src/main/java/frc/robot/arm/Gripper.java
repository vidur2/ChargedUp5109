package frc.robot.arm;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.util.IInit;
import frc.robot.util.ITest;

public class Gripper implements ITest, IInit {

    // fields of class
    private boolean isClamping;
    private Solenoid m_gripperSol;

    public Gripper(int mGripperPortForward, boolean isClamping) {
        // Instantiate stuff
        m_gripperSol = new Solenoid(PneumaticsModuleType.CTREPCM, mGripperPortForward);
        this.isClamping = isClamping;
    }

    public boolean getClamping() {
        return this.isClamping;
    }

    public void release() {
        m_gripperSol.set(false);
        isClamping = false;
    }

    public void grip() {
        m_gripperSol.set(true);
        isClamping = true;
    }

    @Override
    public void test(boolean buttonPress) {
        if (buttonPress) {
            grip();
        } else {
            release();
        }
    }

    @Override
    public void initAuto() {
        System.out.println(isClamping);
        m_gripperSol.set(isClamping);
    }

    @Override
    public void initTeleop() {
        return;
        
    }
}