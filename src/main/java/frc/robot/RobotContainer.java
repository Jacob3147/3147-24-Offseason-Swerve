// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.TeleopDriveAngle;
import frc.robot.Commands.TeleopDriveRate;
import frc.robot.Subsystems.SwerveSubsystem;

public class RobotContainer {

    CommandXboxController m_driverController = new CommandXboxController(0);
    DoubleSupplier left_x = () -> MathUtil.applyDeadband(m_driverController.getLeftX(), 0.05);
    DoubleSupplier left_y = () -> MathUtil.applyDeadband(m_driverController.getLeftY(), 0.05);
    DoubleSupplier right_x = () -> MathUtil.applyDeadband(m_driverController.getRightX(), 0.05);
    DoubleSupplier right_y = () -> MathUtil.applyDeadband(m_driverController.getRightY(), 0.05);

    SwerveSubsystem m_swerveSubsystem = 
        new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve")
        );


    Command TeleopDriveRate = new TeleopDriveRate(m_swerveSubsystem, left_x, left_y, right_x);
    Command TeleopDriveAngle = new TeleopDriveAngle(m_swerveSubsystem, left_x, left_y, right_x, right_y);
    Command faceSpeaker = m_swerveSubsystem.drive_semiAuto(left_x, left_y, m_swerveSubsystem.speaker());

    SendableChooser<Command> autoSelector;

    public RobotContainer() 
    {

        m_swerveSubsystem.setDefaultCommand(TeleopDriveAngle);

        autoSelector = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoSelector);

        configureBindings();
    }

    private void configureBindings()
    {
       // m_driverController.y().toggleOnTrue(TeleopDriveRate);
        m_driverController.b().whileTrue(faceSpeaker);

        m_driverController.a().whileTrue(Commands.runOnce(() -> m_swerveSubsystem.poseResetter.accept(new Pose2d(0, 0, new Rotation2d(0)))));

        m_driverController.x().whileTrue(Commands.runOnce(() -> m_swerveSubsystem.lock(), m_swerveSubsystem).repeatedly());
    }

    public Command getAutonomousCommand() 
    {
        return autoSelector.getSelected();
    }
}
