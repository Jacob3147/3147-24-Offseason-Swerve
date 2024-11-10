package frc.robot.Commands;

import static frc.robot.util.Constants.DriveConstants.maxAngular;
import static frc.robot.util.Constants.DriveConstants.maximumSpeed;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.math.SwerveMath;

public class TeleopDriveRate extends Command
{
    boolean init = false;
    DoubleSupplier vX, vY, vR;
    SwerveSubsystem swerve;

    public TeleopDriveRate(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vR)
    {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.vR = vR;

        addRequirements(swerve);
    }

    @Override
    public void initialize() 
    {
        init = true;
    }

    @Override
    public void execute() 
    {
        Translation2d translationInputs = new Translation2d(-vY.getAsDouble(), -vX.getAsDouble());
        Translation2d scaledTranslation = SwerveMath.scaleTranslation(translationInputs, maximumSpeed);
        
        double scaledRotation = -Math.pow(vR.getAsDouble(), 3) * maxAngular;

        swerve.driveRate(scaledTranslation, scaledRotation);
    }

    @Override
    public void end(boolean interrupted) 
    {
        
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
