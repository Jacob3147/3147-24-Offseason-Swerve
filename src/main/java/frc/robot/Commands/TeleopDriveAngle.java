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
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(vx.getAsDouble(),
                                                                                  vy.getAsDouble()));
                                                                                  
        double rotationX = -rx.getAsDouble();
        double rotationY = -ry.getAsDouble();
        if(Math.abs(rotationX) > 0.5 && Math.abs(rotationY) > 0.5)
        {
            Rotation2d rotation = new Rotation2d(rx.getAsDouble(), ry.getAsDouble());
            m_SwerveSubsystem.driveAngle(scaledInputs, rotation); 
        }
        else
        {
            m_SwerveSubsystem.driveRate(scaledInputs, 0);
        }
    
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_SwerveSubsystem.setHeadingCorrection.accept(false);
    }
}
