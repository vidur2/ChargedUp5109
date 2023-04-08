package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public enum DriverModes {
    kKevinMode,
    kWillMode;

    private double m_kevinMultiplier = 0.2;
    private boolean m_willToggled = false;
    private double m_willMultiplier = 0.3;
    public boolean m_pressed = false;

    public double checkState(XboxController xController) {
        switch (this) {
            case kKevinMode:
                if (xController.getRightBumper() && m_kevinMultiplier < 1 && !m_pressed) {
                    m_kevinMultiplier += 0.2;
                    m_pressed = true;
                } else if (xController.getLeftBumper() && m_kevinMultiplier > 0.2 && !m_pressed) {
                    m_kevinMultiplier -= 0.2;
                    m_pressed = true;
                } else {
                    m_pressed = false;
                }
                if (m_kevinMultiplier < 0.2) {
                    m_kevinMultiplier = 0.2;
                }
                SmartDashboard.putNumber("kevinMultiplier", m_kevinMultiplier);
                return m_kevinMultiplier * Constants.kMaxSpeed;
            case kWillMode:
                SmartDashboard.putBoolean("willToggle", m_willToggled);
                if (xController.getLeftBumper() && !m_pressed) {
                    return Constants.kMaxSpeed;
                } else if (xController.getLeftTriggerAxis() >= 0.5) {
                    return 15;
                }

                return m_willMultiplier * Constants.kMaxSpeed;
        }

        return m_willMultiplier * Constants.kMaxSpeed;
    }
}
