package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.MotorEx;

public class Autonomous extends SequentialCommandGroup{
    //auto is set up such that it will run when you want it to
    
    //shuffleboard
    //inside the constructor you have to put an object of each subsystem you plan to use
    public Autonomous(DriveTrain chassis, Arm arm, Claw claw){
        //put all commands within this super.addcommands
        //make note that it uses commas instead of semicolons because you're technically adding them in a list
        super.addCommands(
            //whatever you put here must extend from command
            //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/package-summary.html
            //check link for relevant subclasses
            claw.toggleClawCommand(),
            claw.moveClawCommand(.5),
            new WaitCommand(0.5),
            claw.moveClawCommand(0),
            chassis.moveTo(-17),
            chassis.turnAngle(90),
            chassis.moveTo(13),
            chassis.turnAngle(-90),
            chassis.moveTo(3),
            chassis.balanceCommand()
        );
    }
}