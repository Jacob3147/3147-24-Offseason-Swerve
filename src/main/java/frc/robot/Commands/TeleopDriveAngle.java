package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import swervelib.math.SwerveMath;

public class TeleopDriveAngle extends Command
{
    SwerveSubsystem m_SwerveSubsystem;
    DoubleSupplier vx, vy, rx, ry;
    Translation2d scaledInputs;
    Rotation2d rotation;
    double vxAct, vyAct, rxAct, ryAct;
    SlewRateLimiter xSlew, ySlew;

    public TeleopDriveAngle(SwerveSubsystem m_SwerveSubsystem, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier rX, DoubleSupplier rY )
    {
        this.m_SwerveSubsystem = m_SwerveSubsystem;

        addRequirements(m_SwerveSubsystem);

        this.vx = vX;
        this.vy = vY;
        this.rx = rX;
        this.ry = rY;

        xSlew = new SlewRateLimiter(2);
        ySlew = new SlewRateLimiter(2);
    }

    @Override
    public void initialize() 
    {
        m_SwerveSubsystem.setHeadingCorrection.accept(true);
    }

    
    public void execute()
    {
        vxAct = vx.getAsDouble();
        vyAct = vy.getAsDouble();
        rxAct = rx.getAsDouble();
        ryAct = ry.getAsDouble();

        scaledInputs = SwerveMath.cubeTranslation(new Translation2d(-vyAct, -vxAct));
        
                                                                     
        if(Math.abs(ryAct) < 0.5 && Math.abs(rxAct) < 0.5)
        {
            rotation = new Rotation2d(0);
        }
        else
        {
            rotation = new Rotation2d(-ryAct, -rxAct);
        }
        m_SwerveSubsystem.driveAngle(scaledInputs, rotation); 
    
    }

    @Override
    public void end(boolean interrupted) 
    {
        m_SwerveSubsystem.setHeadingCorrection.accept(false);
    }
}
