package frc.robot.autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;





public class DriveAutonomous extends SequentialCommandGroup{
    public DriveAutonomous(DriveTrain chassis, Arm arm, Claw claw, boolean seven){
        this.setName(seven ? "Straight normal" : "Straight reverse");

        super.addCommands(
            claw.toggleClawCommand(),
            claw.moveClawCommand(-1),
            chassis.moveTo(-3),
            chassis.turnAngle(180),
            chassis.moveTo(12.6),
            chassis.turnAngle(-90),
            claw.moveClawCommand(1),
            chassis.moveTo(3),
            claw.moveClawCommand(-1),
            chassis.turnAngle(-90),
            chassis.moveTo(13.6)

            


        );
        


    }
}