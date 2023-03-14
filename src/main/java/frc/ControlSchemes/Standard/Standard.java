package frc.ControlSchemes.Standard;

import frc.helpers.ControlScheme;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

public class Standard implements ControlScheme{
    public static void configure(DriveTrain chassis, Intake intake){
        StandardDrive.configure(chassis, 0);
        StandardMechanisms.configure(intake, 1);
    }
}
