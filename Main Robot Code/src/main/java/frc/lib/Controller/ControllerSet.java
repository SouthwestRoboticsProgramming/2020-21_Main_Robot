package frc.lib.controller;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.controller.Buttons;

public class ControllerSet {
    
    private List<MappedController> controllerSet = new ArrayList<MappedController>();

    public void addMappedController(MappedController... mappedController) {
        int count = 0;
        while (count <= mappedController.length-1) {
            controllerSet.add(mappedController[count]);
            count++;
        }
    }

    public JoystickButton getButton(Buttons button) {
        int count = -1;
        while (true) {
            count++;
            if (controllerSet.get(count).checkForButton(button)) {
               return controllerSet.get(count).getButton(button);
            } else if (count >= controllerSet.size()-1) {
                return new JoystickButton(new Joystick(5), 20);
            }
            
        }
        
    }

    public double getAxis(Axis axis) {
        int count = -1;
        while (true) {
            count++;
            if (controllerSet.get(count).checkForAxis(axis)) {
               return controllerSet.get(count).getAxis(axis);
            } else if (count >= controllerSet.size()-1) {
                return 0;
            }
            
        }
    }
}
