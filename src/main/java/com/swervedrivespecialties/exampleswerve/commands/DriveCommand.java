package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import org.frcteam2910.common.robot.Utilities;

public class DriveCommand extends CommandBase {

    private static DrivetrainSubsystem _drive;

    public DriveCommand(DrivetrainSubsystem drive) {
        _drive = drive;
        addRequirements(_drive);
    }

    @Override
    public void execute() {
        double forward = -Robot.getOi().getPrimaryJoystick().getRawAxis(1);
        forward = Utilities.deadband(forward);
        // Square the forward stick
        forward = Math.copySign(Math.pow(forward, 2.0), forward);

        double strafe = -Robot.getOi().getPrimaryJoystick().getRawAxis(0);
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

        double rotation = -Robot.getOi().getPrimaryJoystick().getRawAxis(4);
        rotation = Utilities.deadband(rotation);
        // Square the rotation stick
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        _drive.drive(new Translation2d(forward, strafe), rotation, true);
    }

}
