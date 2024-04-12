// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

    CommandXboxController m_driverController = new CommandXboxController(0);

    SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(
                                                                Filesystem.getDeployDirectory(),
                                                                   "swerve"
                                                                )
                                                            );

    SendableChooser<Command> autoSelector;

    public RobotContainer() 
    {
        Command driveFieldOrientedDirectAngle = m_swerveSubsystem.drive_heading(
            () -> {return MathUtil.applyDeadband(m_driverController.getLeftY(), 0.08); },
            () -> {return MathUtil.applyDeadband(m_driverController.getLeftX(), 0.08); },
            () -> {return m_driverController.getRightX(); },
            () -> {return m_driverController.getRightY(); }
        );


        Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.drive_rotation(
            () -> { return MathUtil.applyDeadband(m_driverController.getLeftY(), 0.08); },
            () -> { return MathUtil.applyDeadband(m_driverController.getLeftX(), 0.08); },
            () -> { return 0.5*MathUtil.applyDeadband(m_driverController.getRightX(), 0.08); }
        );

        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);


        

        autoSelector = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoSelector);

        configureBindings();
    }

    private void configureBindings()
    {
        m_driverController.b().whileTrue(
            m_swerveSubsystem.driveToPose(
                new Pose2d(
                    new Translation2d(0, 0),
                    Rotation2d.fromDegrees(0)
                )
            )
        );
        
        
        m_driverController.a().whileTrue(
            m_swerveSubsystem.drive_semiAuto(
                () -> {return MathUtil.applyDeadband(m_driverController.getLeftY(), 0.08); },
                () -> {return MathUtil.applyDeadband(m_driverController.getLeftX(), 0.08); },
                m_swerveSubsystem.speaker()
            )
        );


        m_driverController.x().whileTrue(Commands.runOnce(() -> m_swerveSubsystem.lock()));
    }

    public void allianceSet(Alliance m_alliance)
    {
        m_swerveSubsystem.configurePathPlanner(m_alliance);
    }

    public Command getAutonomousCommand() 
    {
        return autoSelector.getSelected();
    }
}
