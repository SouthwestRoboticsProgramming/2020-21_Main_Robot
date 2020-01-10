package frc.lib.Controller;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Controller {

    private ControllerSet controllerSet;

    public void setDefaultControllerSet(ControllerSet controllerSet) {
        this.controllerSet = controllerSet;
    }

    public ControllerSet getDefaultControllerSet() {
        return controllerSet;
    }

    public JoystickButton getButton(Buttons Button) {
        return controllerSet.getButton(Button);
    }

    public double getAxis(Axis Axis) {
        return controllerSet.getAxis(Axis);
    }

}
