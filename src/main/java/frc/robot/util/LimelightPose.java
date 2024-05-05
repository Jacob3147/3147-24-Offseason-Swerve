package frc.robot.util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.util.Constants.LimelightConstants;

import java.util.Optional;
import java.util.function.Supplier;




public class LimelightPose
{
    static String LL1 = LimelightConstants.limelight_1_name;
    static String LL2 = LimelightConstants.limelight_2_name;



    public LimelightPose()
    {

    }
    public static void evaluate(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds speeds, Field2d field)
    {
        /*
        This snippet of code looks scary but is actually pretty nice
        An Optional<T> object holds a value of type T that may or may not exist, while avoiding null pointer exceptions
        In this case the evaluate limelight returns an Optional<Pose2d> because it may not see any targets

        One method of an optional is Optional<T>.ifPresent(Consumer<T>)
        If the optional is present, the Consumer function is run and is passed the value of the optional
        Since we know the type T, whatever parameter I use is assumed to be that type. So I can make a lambda with (p) -> and it is known to be the Pose2d
        To add a pose with a name to a Field2d we do field.getObject(String "name").setPose(Pose2d pose)
        */
        evaluate_single_limelight(LL1, poseEstimator, speeds).ifPresent((p) -> field.getObject(LL1 + "_pose").setPose(p));
        evaluate_single_limelight(LL2, poseEstimator, speeds).ifPresent((p) -> field.getObject(LL2 + "_pose").setPose(p));

    }

    private static Optional<Pose2d> evaluate_single_limelight(String LL, SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds speeds)
    {
        LimelightHelpers.SetRobotOrientation(LL, poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);

        double stdDev = 0.5;
        

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

        double poseDiff = mt2.pose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation());

        if(poseDiff > 1 || mt2.tagCount == 0 || Math.abs(speeds.omegaRadiansPerSecond) > Units.degreesToRadians(180))
        {
            return Optional.empty();
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
        
        return Optional.of(mt2.pose);
    }

}
