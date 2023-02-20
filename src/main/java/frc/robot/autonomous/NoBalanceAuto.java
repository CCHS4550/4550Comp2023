package frc.robot.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;


public class NoBalanceAuto extends SequentialCommandGroup{
    //auto is set up such that it will run when you want it to
    
    //shuffleboard
    //inside the constructor you have to put an object of each subsystem you plan to use
    public NoBalanceAuto(DriveTrain chassis, Arm arm, Claw claw){
        this.setName("No Balance");
        //put all commands within this super.addcommands
        //make note that it uses commas instead of semicolons because you're technically adding them in a list
        super.addCommands(
            //whatever you put here must extend from command
            //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/package-summary.html
            //check link for relevant subclasses
            // Score #1
            claw.toggleArmCommand(),
            claw.moveClawCommand(-.5),
            new WaitCommand(0.5),
            claw.moveClawCommand(0),
            new WaitCommand(.1),
            // claw.toggleArmCommand(),
            chassis.moveTo(-12, true),
            // Move to cone
            chassis.turnAngle(180),
            claw.moveClawCommand(.5),
            chassis.moveTo(3, true),
            // Pick up
            new WaitCommand(0.5),
            claw.moveClawCommand(0),
            new WaitCommand(.1),
            // Move back
            chassis.turnAngle(180),
            chassis.moveTo(15, true),
            // Score #2
            claw.moveClawCommand(-.5),
            new WaitCommand(0.5),
            claw.moveClawCommand(0)
        );
    }
}
