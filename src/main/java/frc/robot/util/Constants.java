package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Constants 
{
    public static class DriveConstants
    {
        public static final double maximumSpeed = 4.81; //meters per second
        public static final double maxAngular = 6.28; //radians per second
        public static final Pose2d blue_speaker = new Pose2d(0, 5.56, Rotation2d.fromDegrees(0));
        public static final Pose2d red_speaker = new Pose2d(16.46, 5.56, Rotation2d.fromDegrees(180));
        public static final Pose2d blue_amp = new Pose2d(1.91, 7.59, Rotation2d.fromDegrees(90));
        public static final Pose2d red_amp = new Pose2d(14.66, 7.59, Rotation2d.fromDegrees(180));
        

    }

    public static final class LimelightConstants
    {
        public static final String limelight_1_name = "limelight-front";
        public static final String limelight_2_name = "limelight-rear";

    }

}
