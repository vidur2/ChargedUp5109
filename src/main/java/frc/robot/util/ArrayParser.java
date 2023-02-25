package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation3d;

public class ArrayParser
{
    public static ArrayList<Translation3d> getObjectPoses(double[] poses) throws Exception
    {
        if (poses.length % 3 != 0)
        {
            throw new Exception("poses is not divisible by 3");
        }
        ArrayList<Translation3d> posesRet = new ArrayList<>();
        for (int i = 0; i < poses.length; i += 3)
        {
            posesRet.add(new Translation3d(poses[i - 2], poses[i - 1], poses[i]));
        }

        return posesRet;
    }
}