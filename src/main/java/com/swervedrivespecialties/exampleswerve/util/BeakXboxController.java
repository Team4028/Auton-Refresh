package com.swervedrivespecialties.autonrefresh.util;

import com.swervedrivespecialties.autonrefresh.RobotMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BeakXboxController{

    private XboxController controller;

    public JoystickButton a;
    public JoystickButton b;
    public JoystickButton x;
    public JoystickButton y;
    public JoystickButton START;
    public JoystickButton BACK;
    public JoystickButton LEFT_BUMPER;
    public JoystickButton RIGHT_BUMPER;
    public TriggerButton LEFT_TRIGGER;
    public TriggerButton RIGHT_TRIGGER;
    public DPADButton DPAD_UP;
    public DPADButton DPAD_DOWN;
    public DPADButton DPAD_LEFT;
    public DPADButton DPAD_RIGHT;

    public BeakXboxController(int port){
        controller = new XboxController(port);
        a = new JoystickButton(controller,RobotMap.BUTTON_A);
        b = new JoystickButton(controller,RobotMap.BUTTON_B);
        x = new JoystickButton(controller,RobotMap.BUTTON_X);
        y = new JoystickButton(controller,RobotMap.BUTTON_Y);
        LEFT_BUMPER = new JoystickButton(controller,RobotMap.BUTTON_LB);
        RIGHT_BUMPER = new JoystickButton(controller,RobotMap.BUTTON_RB);
        BACK = new JoystickButton(controller,RobotMap.BUTTON_BACK);
        START = new JoystickButton(controller,RobotMap.BUTTON_START);
        LEFT_TRIGGER = new JoystickButton(controller,RobotMap.LEFT_TRIGGER); //true
        RIGHT_TRIGGER = new JoystickButton(controller,RobotMap.RIGHT_TRIGGER); //false
    //still need to create dpad directions
    }
    public double getLeftXAxis(){
        return controller.getRawAxis(0);
    }
    public double getLeftYAxis(){
        return controller.getRawAxis(1);
    }
    public double getLeftTrigger(){
        return controller.getRawAxis(2);
    }
    public double getRightTrigger(){
        return controller.getRawAxis(3);
    }
    public double getRightXAxis(){
        return controller.getRawAxis(4);
    }
    public double getRightYAxis(){
        return controller.getRawAxis(5);
    }
}

