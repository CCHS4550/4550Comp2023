package frc.ControlSchemes.Bongos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.ControlScheme;
import frc.helpers.Bongos.Combo;
import frc.helpers.Bongos.Comedy;
import frc.maps.DKMap;
import frc.robot.subsystems.Intake;

public class BongosMechanisms implements ControlScheme{
    private static Comedy control;
    public static void configure(Intake intake, int port){
        control = new Comedy(2);
        control.addCombos( 
            shootHigh(intake),
            shootMid(intake),
            intake(intake),
            outtake(intake),
            toggle(intake)
        );
        configureButtons(intake, port);
    }

    private static void configureButtons(Intake intake, int port){
        new JoystickButton(controllers[port], DKMap.UP_RIGHT)
            .onTrue(control.add("tr"));

        new JoystickButton(controllers[port], DKMap.DOWN_RIGHT)
            .onTrue(control.add("br"));

        new JoystickButton(controllers[port], DKMap.UP_LEFT)
            .onTrue(control.add("tl"));
            
        new JoystickButton(controllers[port], DKMap.DOWN_LEFT)
            .onTrue(control.add("bl"));
    }

    private static Combo shootHigh(Intake intake){
        return new Combo(control, intake.autoShoot("High"), 
            "tr",
            "tl",
            "tr"
        );
    }

    private static Combo shootMid(Intake intake){
        return new Combo(control, intake.autoShoot("Middle"), 
            "br",
            "bl",
            "br"
        );
    }

    private static Combo intake(Intake intake){
        return new Combo(control, new SequentialCommandGroup(
            new InstantCommand(() -> intake.spintake(.5, false)),
            new WaitCommand(3),
            new InstantCommand(() -> intake.spintake(0, false))
            // new InstantCommand(() -> System.out.println("in take"))
        ), 
            "bl",
            "bl"
        );
    }

    private static Combo outtake(Intake intake){
        return new Combo(control, new SequentialCommandGroup(
            new InstantCommand(() -> intake.spintake(-.5, false)),
            new WaitCommand(3),
            new InstantCommand(() -> intake.spintake(0, false))
            // new InstantCommand(() -> System.out.println("out take"))
        ), 
            "tl",
            "tl"
        );
    }

    private static Combo toggle(Intake intake){
        return new Combo(control, intake.toggle(), 
            "tl",
            "br"
        );
    }
}
