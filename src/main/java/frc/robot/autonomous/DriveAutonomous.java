package frc.robot.autonomous;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;





public class DriveAutonomous extends SequentialCommandGroup{
    public DriveAutonomous(DriveTrain chassis, Arm arm, Claw claw){
        this.setName("Middle");

        super.addCommands(
            claw.toggleArmCommand(),
            claw.moveClawCommand(-.5),
            new WaitCommand(.5),
            claw.moveClawCommand(0),
            new WaitCommand(.1),
            claw.toggleArmCommand(),
            
            chassis.moveTo(-15, true),
            chassis.turnAngle(180),
            chassis.moveToToBalnenceBackwards(-15),


            // chassis.turnAngle(-90),
            // claw.moveClawCommand(1),
            // chassis.moveTo(3, false),
            // claw.moveClawCommand(-1),
            // chassis.turnAngle(-90),
            // chassis.moveTo(13.6, false),
            // new WaitCommand(.5),
            // chassis.moveTo(6, false),
            chassis.balanceCommand()

            


        );
        


    }
}