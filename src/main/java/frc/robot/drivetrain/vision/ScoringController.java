package frc.robot.drivetrain.vision;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;

public class ScoringController {
    private HashMap<Integer, Pose2d> scoringPoints;
    public int prevPress;

    public ScoringController(HashMap<Integer, Pose2d> scoringPoints) {
        this.scoringPoints = scoringPoints;
    }

    public boolean registerInput(Joystick jStick) {
        Pose2d scoringPose = null;
        if (jStick.getRawButtonPressed(1)) {
            prevPress = 1;
            scoringPose = scoringPoints.get(1);
        } else if (jStick.getRawButtonPressed(2)) {
            prevPress = 2;
            scoringPose = scoringPoints.get(2);
        } else if (jStick.getRawButtonPressed(3)) {
            prevPress = 3;
            scoringPose = scoringPoints.get(3);
        } else if (jStick.getRawButtonPressed(4)) {
            prevPress = 4;
            scoringPose = scoringPoints.get(4);
        } else if (jStick.getRawButtonPressed(5)) {
            prevPress = 5;
            scoringPose = scoringPoints.get(5);
        }  else if (jStick.getRawButtonPressed(6)) {
            prevPress = 6;
            scoringPose = scoringPoints.get(6);
        }  else if (jStick.getRawButtonPressed(7)) {
            prevPress = 7;
            scoringPose = scoringPoints.get(7);
        }  else if (jStick.getRawButtonPressed(8)) {
            prevPress = 8;
            scoringPose = scoringPoints.get(8);
        }  else if (jStick.getRawButtonPressed(9)) {
            prevPress = 9;
            scoringPose = scoringPoints.get(9);
        }

        return scoringPose != null;
    }
    
    public Pose2d continueInput() {
        return scoringPoints.get(prevPress);
    }
}
