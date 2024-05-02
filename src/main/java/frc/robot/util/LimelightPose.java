package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.util.Constants.LimelightConstants;

import java.util.function.Supplier;




public class LimelightPose
{
    static String LL1 = LimelightConstants.limelight_1_name;
    static String LL2 = LimelightConstants.limelight_2_name;

    public LimelightPose()
    {

    }
    public static void evaluate(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds speeds)
    {
        evaluate_single_limelight(LL1, poseEstimator, speeds);
        evaluate_single_limelight(LL2, poseEstimator, speeds);
    }

    private static void evaluate_single_limelight(String LL, SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds speeds)
    {
        LimelightHelpers.SetRobotOrientation(LL, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        double stdDev = 0.5;
        

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

        double poseDiff = mt2.pose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation());

        if(poseDiff > 1 || mt2.tagCount == 0 || Math.abs(speeds.omegaRadiansPerSecond) > Units.degreesToRadians(180))
        {
            return;
        }

        if(mt2.avgTagDist < 4)
        {
            stdDev -= 0.2;
        }
        if(mt2.avgTagDist > 8)
        {
            stdDev += 0.2;
        }

        stdDev /= mt2.tagCount;

        poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            VecBuilder.fill(stdDev,stdDev,9999999));
    }

}
