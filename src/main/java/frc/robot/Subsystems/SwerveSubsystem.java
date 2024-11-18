package frc.robot.Subsystems;

import static frc.robot.util.Constants.DriveConstants.*;

import java.io.File;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;
import frc.robot.util.LimelightPose;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

//1 - BR
//2 - FR
//3 - FL
//4 - BL
public class SwerveSubsystem extends SubsystemBase
{
    private SwerveDrive swerveDrive;
    LimelightPose limelights;

    Field2d field;
    Alliance m_alliance = null;
    
    /**
    * Initialize {@link SwerveDrive} with the directory provided.
    *
    * @param directory Directory of swerve drive config files.
    */
    public SwerveSubsystem(File directory)
    {
        try
        {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        
        } catch (Exception e)
        {
            throw new RuntimeException(e);
        }

        

        swerveDrive.setCosineCompensator(true);

        //0 is disabled. Try 0.1 and -0.1
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

        swerveDrive.getModules()[0].setAntiJitter(true);
        swerveDrive.getModules()[1].setAntiJitter(true);
        swerveDrive.getModules()[2].setAntiJitter(true);
        swerveDrive.getModules()[3].setAntiJitter(true);

        swerveDrive.pushOffsetsToEncoders();

        swerveDrive.setChassisDiscretization(true, 0.02);
    
        swerveDrive.setMotorIdleMode(true);

        field = new Field2d();
        
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        configurePathPlanner();

        
    }

    

    

    public void driveFieldOriented(ChassisSpeeds speeds)
    {
        swerveDrive.driveFieldOriented(speeds);
    }

    /**
    * Configure Path Planner following config
    */
    public void configurePathPlanner()
    {
        AutoBuilder.configureHolonomic(
            poseSupplier,
            poseResetter,
            speedsRobotRelative,
            driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0, 0), //translation
                new PIDConstants(5, 0, 0), //rotation
                swerveDrive.getMaximumVelocity(),
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                new ReplanningConfig()
            ),
            () -> { var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false; },
            this
        );
    }

    /**
     * Pathfind to pose
     * @param pose target {@link Pose2d} to go to
     * @return pathfinding command
     */
    public Command driveToPose(Pose2d pose)
    {
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumVelocity(), 4,
            swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720)
        );

        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            0,
            0
        );
    }
    

    /**
     *  X-lock the wheels
     */
    public void lock() { swerveDrive.lockPose(); }

    //Getters and setters
    public Supplier<Pose2d> poseSupplier = () -> {return swerveDrive.getPose(); };
    public Supplier<Rotation2d> headingSupplier = () -> {return poseSupplier.get().getRotation(); };

    public Supplier<ChassisSpeeds> speedsRobotRelative = () -> {return swerveDrive.getRobotVelocity(); };
    public Supplier<ChassisSpeeds> speedsFieldRelative = () -> {return swerveDrive.getFieldVelocity(); };
    
    public Consumer<Pose2d> poseResetter = (p) -> {swerveDrive.resetOdometry(p); };
    public Consumer<ChassisSpeeds> driveRobotRelative = (speeds) -> { swerveDrive.drive(speeds); };

    public Consumer<Boolean> setHeadingCorrection = (value) -> { swerveDrive.setHeadingCorrection(value); };


    public Pose2d speaker()
    {
        if(m_alliance == DriverStation.Alliance.Red) { return red_speaker; }
        else { return blue_speaker; }
    }

    public void driveAngle(Translation2d translationMetersPerSec, Rotation2d headingDegrees)
    {
        ChassisSpeeds speeds = swerveDrive.swerveController.getTargetSpeeds(
                                translationMetersPerSec.getX(),
                                translationMetersPerSec.getY(),
                                headingDegrees.getRadians(),
                                headingSupplier.get().getRadians(),
                                maximumSpeed);

        swerveDrive.driveFieldOriented(speeds);
    }
    
    public void driveRate(Translation2d translationMetersPerSec, double rotationRadiansPerSec)
    {
        swerveDrive.drive(translationMetersPerSec, rotationRadiansPerSec, true, false);
    }

    /**
     * One stick controls translation, heading is set to a desired angle
     * @param xTrans
     * @param yTrans
     * @param autoHeadingAngle degrees
     * @return
     */
    public Command drive_semiAuto(DoubleSupplier xTrans, DoubleSupplier yTrans, DoubleSupplier autoHeadingAngle)
    {
        return run(() -> {
            swerveDrive.driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    Math.pow(xTrans.getAsDouble(), 3),
                    Math.pow(yTrans.getAsDouble(), 3),
                    Units.degreesToRadians(autoHeadingAngle.getAsDouble()),
                    headingSupplier.get().getRadians(),
                    swerveDrive.getMaximumVelocity()
                )
            );
        });
    }

    

    /**
     * One stick controls translation, heading is set to face a point
     * @param xTrans
     * @param yTrans
     * @param pointTowards {@link Pose2d} to face towards
     * @return
     */
    public Command drive_semiAuto(DoubleSupplier yTrans, DoubleSupplier xTrans, Pose2d pointTowards)
    {
        return run(() -> {
            Translation2d target = pointTowards.getTranslation();
            Translation2d bot = swerveDrive.getPose().getTranslation();
            double angle = 0;
            try {
                angle = bot.minus(target).unaryMinus().getAngle().getRadians();
            } catch (Exception e) {
                
            }
            swerveDrive.driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    -Math.pow(xTrans.getAsDouble(), 3),
                    -Math.pow(yTrans.getAsDouble(), 3),
                    angle,
                    headingSupplier.get().getRadians(),
                    swerveDrive.getMaximumVelocity()
                )
            );
        });
    }

    

    @Override
    public void periodic() 
    {

        //LimelightPose.evaluate(getPoseEstimator.get(), speedsFieldRelative.get(), field);

        field.setRobotPose(poseSupplier.get());
    }

}
