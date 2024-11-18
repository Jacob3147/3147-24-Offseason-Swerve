package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.math.SwerveMath;

public class TeleopDriveAngle extends Command
{
    SwerveSubsystem m_SwerveSubsystem;
    DoubleSupplier vx, vy, rx, ry;

    public TeleopDriveAngle(SwerveSubsystem m_SwerveSubsystem, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rX, DoubleSupplier rY )
    {
        this.m_SwerveSubsystem = m_SwerveSubsystem;

        addRequirements(m_SwerveSubsystem);

        this.vx = vX;
        this.vy = vY;
        this.rx = rX;
        this.ry = rY;
    }

    @Override
    public void initialize() 
    {
        m_SwerveSubsystem.setHeadingCorrection.accept(true);
    }

    
    public void execute()
    {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(-vy.getAsDouble(),
                                                                                  -vx.getAsDouble()));
                                                                                  
        Rotation2d rotation = new Rotation2d(-ry.getAsDouble(), -rx.getAsDouble());
        m_SwerveSubsystem.driveAngle(scaledInputs, rotation); 
    
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_SwerveSubsystem.setHeadingCorrection.accept(false);
    }
}
