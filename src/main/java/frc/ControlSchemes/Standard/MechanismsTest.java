package frc.ControlSchemes.Standard;

import frc.diagnostics.DoubleEntry;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.subsystems.Intake;

public class MechanismsTest implements ControlScheme{
    public static void configure(Intake intake, int port){

        intake.setDefaultCommand(
                new RunCommand(() -> intake.manageIntake(
                        OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL) * (OI.axis(1, ControlMap.RT) > 0.5 ? 0.8 : 0.7) * (OI.button(1, ControlMap.RB_BUTTON) ? 0.5 : 1),
                        OI.axis(port, ControlMap.R_JOYSTICK_VERTICAL),
                        OI.axis(port, ControlMap.LT) > 0.5), intake));

        configureButtons(intake, port);
    }

    private static void configureButtons(Intake intake, int port){
        // this is how you do teleop stuff
        // when you create a trigger or trigger subclass, it will run whatever you want
        // run when you specify

        DoubleEntry power = new DoubleEntry("Shoot Power", -1);
        DoubleEntry breakoff = new DoubleEntry("Shoot Breakoff", .15);
        DoubleEntry total = new DoubleEntry("Shoot total", .2);
        new JoystickButton(controllers[port], ControlMap.A_BUTTON)
                .onTrue(intake.autoShoot(-0.5, 0.15, 0.2));
    }
}
//22 steel hex shaft
