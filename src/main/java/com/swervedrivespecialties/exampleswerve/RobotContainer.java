package com.swervedrivespecialties.autonrefresh;

import com.swervedrivespecialties.autonrefresh.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.autonrefresh.util.BeakXboxController;
import com.swervedrivespecialties.autonrefresh.commands.DriveCommand;

public class RobotContainer {

    public DrivetrainSubsystem drive = DrivetrainSubsystem.getinstance(); 

    public RobotContainer(){
        initDefaultCommands();
    }

    public void initDefaultCommands(){
        CommandScheduler.getinstance().setDefaultCommand(drive, DriveSubsystemCommands.getDriveCommand());
    }
}
//don't merge this w/ master...just checking to see if my commits are working right