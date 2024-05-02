package frc.robot.util;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class PhotonPose 
{
    static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    static Translation3d zero = new Translation3d(0,0,0);

    static PhotonCamera cam1 = new PhotonCamera("frontCam");
    static PhotonCamera cam2 = new PhotonCamera("rearCam");

    static Transform3d cam1ToBot = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));
    static Transform3d cam2ToBot = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    static PhotonPoseEstimator cam1PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1ToBot);
    static PhotonPoseEstimator cam2PoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam2ToBot);

    static Optional<EstimatedRobotPose> cam1BotposeOptional;
    static Optional<EstimatedRobotPose> cam2BotposeOptional;

    static EstimatedRobotPose cam1Botpose;
    static EstimatedRobotPose cam2Botpose;

    public PhotonPose()
    {

    }

    public static void evaluate(SwerveDrivePoseEstimator poseEstimator, ChassisSpeeds speeds, Field2d field)
    {
        /*
        This snippet of code looks scary but is actually pretty nice
        An Optional<T> object holds a value of type T that may or may not exist, while avoiding null pointer exceptions
        In this case the evaluate photon returns an Optional<Pose2d> because it may not see any targets

        One method of an optional is Optional<T>.ifPresent(Consumer<T>)
        If the optional is present, the Consumer function is run and is passed the value of the optional
        Since we know the type T, whatever parameter I use is assumed to be that type. So I can make a lambda with (p) -> and it is known to be the Pose2d
        To add a pose with a name to a Field2d we do field.getObject(String "name").setPose(Pose2d pose)
        */
        evaluate_single_photoncamera(cam1PoseEstimator, cam1BotposeOptional, cam1Botpose, poseEstimator, speeds)
            .ifPresent((p) -> field.getObject("cam1_pose").setPose(p));

        evaluate_single_photoncamera(cam2PoseEstimator, cam2BotposeOptional, cam2Botpose, poseEstimator, speeds)
            .ifPresent((p) -> field.getObject("cam2_pose").setPose(p));
    }
    
    private static Optional<Pose2d>  evaluate_single_photoncamera(
                PhotonPoseEstimator PhotonEst, 
                Optional<EstimatedRobotPose> OptionalPose, 
                EstimatedRobotPose ActualPose, 
                SwerveDrivePoseEstimator poseEstimator,
                ChassisSpeeds speeds)
    {
        double stdDev = 0.5;
        double targetCount = 0;
        double distance;
        double closestDistance = 1000;
        Pose2d estimatedPose2d;
        double poseDiff;

        OptionalPose = PhotonEst.update();
        if(OptionalPose.isPresent())
        {
            ActualPose = OptionalPose.get();
            estimatedPose2d = ActualPose.estimatedPose.toPose2d();
            
            for(PhotonTrackedTarget target : ActualPose.targetsUsed)
            {
                targetCount++;

                distance = target.getBestCameraToTarget().getTranslation().getDistance(zero);
                if(distance < closestDistance)
                {
                    closestDistance = distance;
                }

            }

            poseDiff = estimatedPose2d.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation());

            if(poseDiff > 1 || Math.abs(speeds.omegaRadiansPerSecond) > Units.degreesToRadians(180))
            {
                return Optional.empty();
            } 

            
            if(closestDistance < 4)
            {
                stdDev -= 0.2;
            }
            if(closestDistance > 10)
            {
                stdDev += 0.3;
            }

            stdDev /= targetCount;

            if(stdDev < 0.1) stdDev = 0.1;

            poseEstimator.addVisionMeasurement(estimatedPose2d, ActualPose.timestampSeconds, VecBuilder.fill(stdDev,stdDev,9999999));

            return Optional.of(estimatedPose2d);
        }
        return Optional.empty();
    }







}
