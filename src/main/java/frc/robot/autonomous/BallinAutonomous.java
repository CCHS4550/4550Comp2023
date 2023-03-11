package frc.robot.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class BallinAutonomous extends SequentialCommandGroup{
    //auto is set up such that it will run when you want it to
    
    //shuffleboard
    //inside the constructor you have to put an object of each subsystem you plan to use
    public BallinAutonomous(DriveTrain chassis, Intake intake){
        //put all commands within this super.addcommands
        //make note that it uses commas instead of semicolons because you're technically adding them in a list
        super.addCommands(
            //whatever you put here must extend from command
            //https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/package-summary.html
            //check link for relevant subclasses
            intake.autoShoot("High"),
            chassis.moveTo(-11.5, false),
            new ParallelCommandGroup(
                chassis.turnAngle(180),
                intake.toggle()
            ),
            new InstantCommand(() -> chassis.gyro.reset()),
            new InstantCommand(() -> intake.setSpin(0.5), intake),
            chassis.moveTo(4.25, false),
            new WaitCommand(0.25),
            new InstantCommand(() -> intake.setSpin(0), intake),
            intake.toggle(),
            chassis.turnAngle(180),
            chassis.moveTo(5, false),
            intake.autoShoot("Horizontal"),
            chassis.moveTo(4, false),
            new ParallelCommandGroup(
                chassis.turnAngle(-135),
                intake.toggle()
            ),
            new InstantCommand(() -> intake.setSpin(0.5), intake),
            chassis.moveTo(3, false),
            new InstantCommand(() -> intake.setSpin(0), intake),
            new ParallelCommandGroup(
                chassis.turnAngle(135),
                intake.toggle()
            ),
            chassis.moveToToBalnenceBackwards(15),


            


            new ParallelCommandGroup(
                chassis.balanceCommand(),
                intake.autoShoot("High")
            )
        );
    }
}
