package frc.robot.Subsystems;

import static frc.robot.Utility.Constants.DriveConstants.*;

import java.io.File;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.Vision;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;


public class SwerveSubsystem extends SubsystemBase
{
    private SwerveDrive swerveDrive;
    Vision limelights;

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

        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setCosineCompensator(false);
        swerveDrive.getModules()[0].setAntiJitter(false);
        swerveDrive.getModules()[1].setAntiJitter(false);
        swerveDrive.getModules()[2].setAntiJitter(false);
        swerveDrive.getModules()[3].setAntiJitter(false);
    
        field = new Field2d();
        
        
        limelights = new Vision(() -> swerveDrive.getRobotVelocity().omegaRadiansPerSecond,
                                () -> swerveDrive.swerveDrivePoseEstimator);
    }

    /**
    * Configure Path Planner following config
    * Called by robotcontainer when driver station is connected (so we know our alliance)
    */
    public void configurePathPlanner(Alliance alliance)
    {
        m_alliance = alliance;
        AutoBuilder.configureHolonomic(
            poseSupplier,
            poseResetter,
            speedsRobotRelative,
            driveRobotRelative,
            new HolonomicPathFollowerConfig(
                new PIDConstants(5, 0, 0), //translation
                new PIDConstants(5, 0, 0), //rotation
                maximumSpeed,
                17.68,
                new ReplanningConfig()
            ),
            () -> { return alliance == DriverStation.Alliance.Red; },
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
    Supplier<Pose2d> poseSupplier = () -> {return swerveDrive.getPose(); };
    Consumer<Pose2d> poseResetter = (p) -> {swerveDrive.resetOdometry(p); };
    Supplier<ChassisSpeeds> speedsRobotRelative = () -> {return swerveDrive.getRobotVelocity(); };
    Consumer<ChassisSpeeds> driveRobotRelative = (speeds) -> {swerveDrive.drive(speeds); };
    Supplier<ChassisSpeeds> speedsFieldRelative = () -> {return swerveDrive.getFieldVelocity(); };
    Supplier<Rotation2d> getHeading = () -> {return poseSupplier.get().getRotation(); };
    Supplier<SwerveDrivePoseEstimator> getPoseEstimator = () -> {return swerveDrive.swerveDrivePoseEstimator; };

    public Pose2d speaker()
    {
        if(m_alliance == DriverStation.Alliance.Red)
        {
            return red_speaker;
        }
        else
        {
            return blue_speaker;
        }
    }
    
    
    /**
     * One stick controls translation, the other stick controls heading
     * @param xTrans
     * @param yTrans
     * @param xRot
     * @param yRot
     * @return Command to run
     */
    public Command drive_heading(DoubleSupplier xTrans, DoubleSupplier yTrans, DoubleSupplier xRot, DoubleSupplier yRot)
    {
        return run(() -> {
            swerveDrive.driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    Math.pow(xTrans.getAsDouble(), 3),
                    Math.pow(yTrans.getAsDouble(), 3),
                    xRot.getAsDouble(),
                    yRot.getAsDouble(),
                    getHeading.get().getRadians(),
                    swerveDrive.getMaximumVelocity()
                )
            );
        });
    }

    /**
     * One stick controls translation, one axis of the other controls rotation (or 3d flight stick)
     * @param xTrans
     * @param yTrans
     * @param rotVel
     * @return Command to run
     */
    public Command drive_rotation(DoubleSupplier xTrans, DoubleSupplier yTrans, DoubleSupplier rotVel)
    {
        return run(() -> {
            swerveDrive.drive(
                new Translation2d(
                    Math.pow(xTrans.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                    Math.pow(yTrans.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()
                ),
                Math.pow(rotVel.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                true,
                false
            );

        });
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
                    getHeading.get().getRadians(),
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
    public Command drive_semiAuto(DoubleSupplier xTrans, DoubleSupplier yTrans, Pose2d pointTowards)
    {
        return run(() -> {
            Translation2d target = pointTowards.getTranslation();
            Translation2d bot = swerveDrive.getPose().getTranslation();
            double angle = 0;
            try {
                angle = bot.minus(target).getAngle().getRadians();
            } catch (Exception e) {
                
            }
            swerveDrive.driveFieldOriented(
                swerveDrive.swerveController.getTargetSpeeds(
                    Math.pow(xTrans.getAsDouble(), 3),
                    Math.pow(yTrans.getAsDouble(), 3),
                    angle,
                    getHeading.get().getRadians(),
                    swerveDrive.getMaximumVelocity()
                )
            );
        });
    }

    /**
     * One stick controls translation, one axis of the other controls rotation (or 3d flight stick)
     * Code can help
     * @param xTrans
     * @param yTrans
     * @param rotVel
     * @param codeRotVel
     * @return Command to run
     */
    public Command drive_rotation_with_assist(DoubleSupplier xTrans, DoubleSupplier yTrans, DoubleSupplier rotVel, DoubleSupplier codeRotVel)
    {
        return run(() -> {
            swerveDrive.drive(
                new Translation2d(
                    Math.pow(xTrans.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                    Math.pow(yTrans.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()
                ),
                Math.pow(rotVel.getAsDouble() + codeRotVel.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                true,
                false
            );

        });
    }

    @Override
    public void periodic() 
    {
        field.setRobotPose(poseSupplier.get());
    }

}
