package com.swervedrivespecialties.autonrefresh;

import com.swervedrivespecialties.autonrefresh.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.autonrefresh.util.AutonChooser;
import com.swervedrivespecialties.autonrefresh.util.BeakXboxController;
import com.swervedrivespecialties.autonrefresh.util.util;
import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;

public class RobotContainer {

    public DrivetrainSubsystem drive = new DrivetrainSubsystem.getinstance(); 

    private BeakXboxController controller1 = new BeakXboxController(RobotMap.PRIMARY_CONTROLLER_USB_PORT);
    private BeakXboxController controller2 = new BeakXboxController(RobotMap.SECONDARY_CONTROLLER_USB_PORT);
    private BeakXboxController controller3 = new BeakXboxController(RobotMap.TERTIARY_CONTROLLER_USB_PORT);

    AutonChooser ac = AutonChooser.getinstance();

    public RobotContainer(){
        initDefaultCommands();
    }

    public double getController1LeftXAxis(){
        return controller1.getleftXAxis();
    }

    public double getController1LeftYAxis(){
        return controller2.getLeftYAxis();
    }

    public double getController1LeftTrigger(){
        return controller1.getLeftTrigger();
    }

    public double getController1RightTrigger(){
        return controller1.getRightTrigger();
    }

    public double getController1RightXAxis(){
        return controller1.getrightXAxis();
    }

    public double getController1RightYAxis(){
        return controller1.getrightYAxis();
    }

    public double getController2LeftXAxis(){
        return controller2.getleftXAxis();
    }

    public double getController2LeftYAxis(){
        return controller2.getleftYAxis();
    }

    public double getController2RightYAxis(){
        return controller2.getrightYAxis();
    }

    public doible getController3RightYAxis(){
        return controller3.getrightYAxis();
    }

    public void initDefaultCommands(){
        CommandScheduler.getinstance().setDefaultCommand(drive, DriveSubsystemCommands.getDriveCommand());
    }

    public CommandBase getAuton(){
     return ac.getAuton();
    }
}