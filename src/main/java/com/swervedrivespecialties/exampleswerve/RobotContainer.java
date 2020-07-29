package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.util.BeakXboxController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;

public class RobotContainer {

    public DrivetrainSubsystem drive = DrivetrainSubsystem.getInstance(); 

    public RobotContainer(){
        initDefaultCommands();
    }

    public void initDefaultCommands(){
        CommandScheduler.getInstance().setDefaultCommand(drive, new DriveCommand());
    }
}
