package frc.lib.Controller;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.Controller.Buttons;

public class MappedController {
    
    private Joystick joy;
    private List<Buttons> buttons = new ArrayList<Buttons>();
    private List<Integer> buttonsPorts = new ArrayList<Integer>();
    private List<Axis> axis = new ArrayList<Axis>();
    private List<Integer> axisPorts = new ArrayList<Integer>();



    public MappedController(int port) {
        joy = new Joystick(port);

    }

    public MappedController mapButton(Buttons Buttons, int port) {
        buttons.add(Buttons);
        buttonsPorts.add(port);
        return this;
    }

    public MappedController mapAxis(Axis Axis, int port) {
        this.axis.add(Axis);
        axisPorts.add(port);
        return this;
    }

    public boolean checkForButton(Buttons button) {
        int count = getButtonCount(button);
        if (count == -1) {
            return false;
        } else {
            return true;
        }
    }

    public boolean checkForAxis(Axis axis) {
        int count = getAxisCount(axis);
        if (count == -1) {
            return false;
        } else {
            return true;
        }
    }

    public JoystickButton getButton(Buttons button) {
        int count = getButtonCount(button);
        int buttonNumber = buttonsPorts.get(count);
        return new JoystickButton(joy, buttonNumber);
    }

    public double getAxis(Axis axis) {
        int count = getAxisCount(axis);
        int axisNumber = axisPorts.get(count);
        return joy.getRawAxis(axisNumber);
    }

    private int getButtonCount(Buttons button) {
        Buttons recallButton = null;
        int count = -1;
        while (recallButton != button) {
            count++;
            try {
                recallButton = buttons.get(count);
            } catch (Exception e) {
                return -1;
            }
        }
        return count;
    }

    private int getAxisCount(Axis axis) {
        Axis recallAxis = null;
        int count = -1;
        while (recallAxis != axis) {
            count++;
            try {
                recallAxis = this.axis.get(count);
            } catch (Exception e) {
                return -1;
            }
        }
        return count;
    }

}
