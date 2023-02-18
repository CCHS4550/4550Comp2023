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


public class BallinAutonomous extends SequentialCommandGroup{
    //auto is set up such that it will run when you want it to
    
    //shuffleboard
    //inside the constructor you have to put an object of each subsystem you plan to use
    public BallinAutonomous(DriveTrain chassis, Arm arm, Claw claw){
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
            chassis.moveTo(-3),
            // Move to cone
            chassis.turnAngle(180),
            chassis.moveTo(17),
            // Pick up
            claw.moveClawCommand(.5),
            new WaitCommand(0.5),
            claw.moveClawCommand(0),
            // Move back
            chassis.turnAngle(90),
            chassis.moveTo(2),
            chassis.turnAngle(90),
            chassis.moveTo(17),
            // Score #2
            claw.moveClawCommand(-.5),
            new WaitCommand(0.5),
            claw.moveClawCommand(0)
        );
    }
}
