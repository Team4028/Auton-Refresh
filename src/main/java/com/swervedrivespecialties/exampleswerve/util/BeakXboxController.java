package com.swervedrivespecialties.autonrefresh.util;

import com.swervedrivespecialties.autonrefresh.RobotMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class BeakXboxController{

    public static final double DEADBAND = .05;

    public XboxController controller;

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
        LEFT_TRIGGER = new TriggerButton(controller,RobotMap.LEFT_TRIGGER); //true = left
        RIGHT_TRIGGER = new TriggerButton(controller,RobotMap.RIGHT_TRIGGER); //false = right
        DPAD_UP = new DpadButton(controller,Directions.UP);
        DPAD_DOWN = new DpadButton(controller,Directions.DOWN);
        DPAD_LEFT = new DpadButton(controller,Directions.LEFT);
        DPAD_RIGHT = new DpadButton(controller,Directions.RIGHT);
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
     public static class TriggerButton extends Button {
       private XboxController TB_Controller;
       private boolean left;
       private boolean right;
       
        public TriggerButton(XboxController controller, boolean left, boolean right){
            TB_Controller = controller;
           
            if (controller.getLeftTrigger() > DEADBAND){
                left = true;
            } else {
                left = false;
            }
            if (controller.getRightTrigger() > DEADBAND){
                right = true;
            } else {
                right = false;
            }
        }
        @Override
        public boolean get(){
            return TB_Controller.getTriggerAxis(left) > DEADBAND; 
            return TB_Controller.getTriggerAxis(right) > DEADBAND;
        }
    }
    public static class DpadButton extends Button{
        public enum Directions {
            UP(0),
            DOWN(180),
            RIGHT(90),
            LEFT(270);

            private final int angle;
        
            Direction(int angleVal){
                angle = angleVal;
            }
            public int getAngle(){
                return angleVal;
            }
        }
        XboxController DPAD_CONTROLLER;
        Directions direction;

        public DpadButton(XboxController controller, Directions dir){
            DPAD_CONTROLLER = controller;
            direction = dir;
        }
        @Override
        public boolean get(){
            return DPAD_CONTROLLER.getPOV() == direction.getAngle();
        }
    }
}

