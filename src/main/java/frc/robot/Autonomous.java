package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MotorEx;

public class Autonomous extends SequentialCommandGroup{
    //auto is set up such that it will run when yuo want it to

    //inside the constructor you have to put an object of each subsystem you plan to use
    public Autonomous(MotorEx example){
        //put all commands within this super.addcommands
        //make note that it uses commas instead of semicolons because you're technically adding them in a list
        super.addCommands(
            //whatever you put here must extend from command
            //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/package-summary.html
            //check link for relevant subclasses
            
            example.runForTime(5, 0.5), //runs the motor at half power for 5 seconds
            example.reachPos(0.7) //runs the motor until it reaches the specified position
        );
    }
}