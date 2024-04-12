package frc.robot.Utility;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Utility.Constants.LimelightConstants;

import java.util.function.Supplier;




public class Vision extends SubsystemBase
{
    SwerveSubsystem drive;
    String LL1 = LimelightConstants.limelight_1_name;
    String LL2 = LimelightConstants.limelight_2_name;

    boolean trust;
    double confidence;
    ChassisSpeeds speeds;
    double linearspeed;
    double angularspeed;

    boolean LL_1_hastarget;
    boolean LL_2_hastarget;

    Supplier<Double> yawRateSupplier;

    Supplier<SwerveDrivePoseEstimator> odometry;
    SwerveDrivePoseEstimator poseEstimator;

    public Vision(Supplier<Double> yawRate, Supplier<SwerveDrivePoseEstimator> odometry)
    {
        this.drive = drive;
        yawRateSupplier = yawRate;
        this.odometry = odometry;
    }

    @Override
    public void periodic() 
    {
        poseEstimator = odometry.get();
        
        if(DriverStation.isTeleop())
        {
            LL_1_hastarget = Evaluate_Limelight(LL1);
            LL_2_hastarget = Evaluate_Limelight(LL2);
        }

        SmartDashboard.putBoolean("LL front valid", LL_1_hastarget);
        SmartDashboard.putBoolean("LL rear valid", LL_2_hastarget);

        LimelightHelpers.SetRobotOrientation("limelight", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    public boolean Evaluate_Limelight(String LL)
    {
        boolean doRejectUpdate = false;
        double confidence = 0.7;
        

        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL);

        double poseDiff = mt2.pose.getTranslation().getDistance(poseEstimator.getEstimatedPosition().getTranslation());
        //field.getObject("LL").setPose(mt2.pose);
        SmartDashboard.putNumber("Posediff " + LL, poseDiff);
        SmartDashboard.putNumber("tagcount " + LL, mt2.tagCount);
        SmartDashboard.putNumber("yaw rate", Math.abs(yawRateSupplier.get()));
        if(Math.abs(yawRateSupplier.get()) > 180) // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        {
            doRejectUpdate = true;
        }
        if(poseDiff > 0.5)
        {
            confidence -= 0.2;
        }
        if(poseDiff > 1)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount > 2)
        {
            confidence /= 2;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(confidence,confidence,9999999));
            poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }

        return !doRejectUpdate;

    }

}
