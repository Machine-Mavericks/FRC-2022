package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the subsystems.
 */
public class OI {
    /**
     * Inner class containing controller bindings
     */
    private static class Bindings {
        /** Example button */
        static final Button EXAMPLE_BUTTON = XboxController.Button.kA;
        static final Button INTAKE_BUTTON = XboxController.Button.kX;
    }

    /** Port for controller used by driver */
    private static final int DRIVER_CONTROLLER_PORT = 0;
    /** Port for controller used by operator */
    private static final int OPERATOR_CONTROLLER_PORT = 1;

    /** Controller used by driver, mapped to {@link #DRIVER_CONTROLLER_PORT} */
    public static final XboxController driverController = new XboxController(DRIVER_CONTROLLER_PORT);
    /** Controller used by driver, mapped to {@link #OPERATOR_CONTROLLER_PORT} */
    public static final XboxController operatorController = new XboxController(OPERATOR_CONTROLLER_PORT);

    /** Example button. Mapped to {@link Bindings#EXAMPLE_BUTTON} */
    public static final JoystickButton exampleButton = new JoystickButton(driverController, Bindings.EXAMPLE_BUTTON.value);
    /** Button to deploy intake for 5 seconds. Mapped to {@link Bindings#INTAKE_BUTTON} */
    public static final JoystickButton intakeButton = new JoystickButton(operatorController, Bindings.INTAKE_BUTTON.value);
}
