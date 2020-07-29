package com.swervedrivespecialties.exampleswerve.util;

import com.swervedrivespecialties.exampleswerve.util.BeakXboxController.DpadButton.Direction;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Button;

public class BeakXboxController {

    // these constants all come from the layout of our physical controllers
    // and their IO interfaces with WPILib

    // deadband
    private static final double DEADBAND = .05;

    // button indexing
    private static final int BUTTON_A_INDEX = 1;
    private static final int BUTTON_B_INDEX = 2;
    private static final int BUTTON_X_INDEX = 3;
    private static final int BUTTON_Y_INDEX = 4;
    private static final int BUTTON_LB_INDEX = 5;
    private static final int BUTTON_RB_INDEX = 6;
    private static final int BUTTON_BACK_INDEX = 7;
    private static final int BUTTON_START_INDEX = 8;

    // trigger identification booleans
    private static final boolean IS_LEFT_TRIGGER_DEFAULT = true;
    private static final boolean LEFT_TRIGGER_BOOLEAN = IS_LEFT_TRIGGER_DEFAULT;
    private static final boolean RIGHT_TRIGGER_BOOLEAN = !IS_LEFT_TRIGGER_DEFAULT;

    // axis indexing
    private static final int LEFT_X_AXIS_INDEX = 0;
    private static final int LEFT_Y_AXIS_INDEX = 1;
    private static final int LEFT_TRIGGER_AXIS_INDEX = 2;
    private static final int RIGHT_TRIGGER_AXIS_INDEX = 3;
    private static final int RIGHT_X_AXIS_INDEX = 4;
    private static final int RIGHT_Y_AXIS_INDEX = 5;

    // DPAD degree values; like a clock--0 at top, clockwise is positive
    private static final int DPAD_UP_DEGREES = 0;
    private static final int DPAD_RIGHT_DEGREES = 90;
    private static final int DPAD_DOWN_DEGREES = 180;
    private static final int DPAD_LEFT_DEGREES = 270;

    // end of constants, declarations and class follow

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
    public DpadButton DPAD_UP;
    public DpadButton DPAD_DOWN;
    public DpadButton DPAD_LEFT;
    public DpadButton DPAD_RIGHT;

    public BeakXboxController(int port){
        controller = new XboxController(port);
        a = new JoystickButton(controller, BUTTON_A_INDEX);
        b = new JoystickButton(controller, BUTTON_B_INDEX);
        x = new JoystickButton(controller, BUTTON_X_INDEX);
        y = new JoystickButton(controller, BUTTON_Y_INDEX);
        LEFT_BUMPER = new JoystickButton(controller, BUTTON_LB_INDEX);
        RIGHT_BUMPER = new JoystickButton(controller,BUTTON_RB_INDEX);
        BACK = new JoystickButton(controller, BUTTON_BACK_INDEX);
        START = new JoystickButton(controller, BUTTON_START_INDEX);
        //true = left, false = right (right now)
        LEFT_TRIGGER = new TriggerButton(controller, LEFT_TRIGGER_BOOLEAN); 
        RIGHT_TRIGGER = new TriggerButton(controller, RIGHT_TRIGGER_BOOLEAN); 
        DPAD_UP = new DpadButton(controller, Direction.UP);
        DPAD_DOWN = new DpadButton(controller, Direction.DOWN);
        DPAD_LEFT = new DpadButton(controller, Direction.LEFT);
        DPAD_RIGHT = new DpadButton(controller, Direction.RIGHT);
    }

    public double getLeftXAxis(){
        return controller.getRawAxis(LEFT_X_AXIS_INDEX);
    }

    public double getLeftYAxis(){
        return controller.getRawAxis(LEFT_Y_AXIS_INDEX);
    }

    public double getLeftTrigger(){
        return controller.getRawAxis(LEFT_TRIGGER_AXIS_INDEX);
    }

    public double getRightTrigger(){
        return controller.getRawAxis(RIGHT_TRIGGER_AXIS_INDEX);
    }

    public double getRightXAxis(){
        return controller.getRawAxis(RIGHT_X_AXIS_INDEX);
    }
    public double getRightYAxis(){
        return controller.getRawAxis(RIGHT_Y_AXIS_INDEX);
    }

    public static class TriggerButton extends Button {
        private XboxController tbController;
        private Hand hand;

        //based on the true/false trigger identification convention established by the constants up top
        public TriggerButton(XboxController controller, boolean identificationBoolean){
            tbController = controller;
            hand = (identificationBoolean == IS_LEFT_TRIGGER_DEFAULT) ? Hand.kLeft : Hand.kRight;
        }

        @Override
        public boolean get() {
            return tbController.getTriggerAxis(hand) > DEADBAND;// was kDeadband before
        }
    }


    public static class DpadButton extends Button{
        public enum Direction {
            UP(DPAD_UP_DEGREES),
            DOWN(DPAD_DOWN_DEGREES),
            RIGHT(DPAD_RIGHT_DEGREES),
            LEFT(DPAD_LEFT_DEGREES);

            private final int angle;
        
            Direction(int angleVal){
                angle = angleVal;
            }

            public int getAngle(){
                return angle;
            }
        }
        XboxController DPAD_CONTROLLER;
        Direction direction;

        public DpadButton(XboxController controller, Direction dir){
            DPAD_CONTROLLER = controller;
            direction = dir;
        }

        @Override
        public boolean get(){
            return DPAD_CONTROLLER.getPOV() == direction.getAngle();
        }
    }
}