package frc.ControlSchemes.Bongos;

import frc.helpers.ControlScheme;
import frc.robot.subsystems.Intake;

public class Bongos implements ControlScheme{
    public static void configure(Intake intake){
        BongosMechanisms.configure(intake, 1);
    }
}
