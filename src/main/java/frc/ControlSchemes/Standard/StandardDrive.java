package frc.ControlSchemes.Standard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.subsystems.DriveTrain;

public class StandardDrive implements ControlScheme{
public static void configure(DriveTrain chassis, int port){

        chassis.setDefaultCommand(new RunCommand(() -> chassis.axisDrive(
                OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL) * (OI.axis(0, ControlMap.LT) > 0.5 ? 1 : 0.7) * (OI.axis(0, ControlMap.RT) > 0.5 ? 0.5 : 1),
                OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL) * (OI.axis(0, ControlMap.LT) > 0.5 ? 1.3 : 1) * (OI.axis(0, ControlMap.RT) > 0.5 ? 0.75 : 1),
                OI.button(port, ControlMap.L_JOYSTICK_BUTTON) ? 0 : chassis.defaultAccelTime), chassis));


        configureButtons(chassis, port);
    }

    private static void configureButtons(DriveTrain chassis, int port){
        // this is how you do teleop stuff
        // when you create a trigger or trigger subclass, it will run whatever you want
        // run when you specify

        new JoystickButton(controllers[port], ControlMap.A_BUTTON)
                .onTrue(new InstantCommand(() -> chassis.gyro.reset(), chassis));

        // new JoystickButton(controllers[0], ControlMap.B_BUTTON)
        // .onTrue(new SequentialCommandGroup(new InstantCommand(() ->
        // chassis.gyro.reset(), chassis), chassis.turnAngle(180)));


    }
}
