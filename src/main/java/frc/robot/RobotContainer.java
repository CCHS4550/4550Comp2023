package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.diagnostics.BooleanSwitch;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.ControlSchemes.Bongos.BongosMechanisms;
import frc.ControlSchemes.Standard.MechanismsTest;
import frc.ControlSchemes.Standard.Standard;
import frc.ControlSchemes.Standard.StandardDrive;
import frc.ControlSchemes.Standard.StandardMechanisms;

// import frc.robot.autonomous.Autonomous;
// import frc.robot.autonomous.BallinAutonomous;
// import frc.robot.autonomous.DriveAutonomous;

// import frc.robot.autonomous.*;
import frc.diagnostics.*;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.BallinAutonomous;
import frc.robot.autonomous.Move;
// import frc.robot.subsystems.MotorEx;
import frc.robot.subsystems.*;

public class RobotContainer {
    // must instantiate an object of each subsystem you use
    private DriveTrain chassis = new DriveTrain();
    private Intake intake = new Intake(chassis);

    DoubleEntry pow = new DoubleEntry("power", 0.1);

    public RobotContainer() {
        StandardMechanisms.configure(intake, 1);
        StandardDrive.configure(chassis, 0);
        //BongosMechanisms.configure(intake, 1);
    }

    DoubleEntry turnval = new DoubleEntry("bollocks", 0);
    EncoderTest enc = new EncoderTest(0);
    AHRS gyro = chassis.gyro;
    void test() {
        // double angle = -1;
        // chassis.arcade(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), OI.axis(0, ControlMap.R_JOYSTICK_HORIZONTAL));
        // double ang = gyro.getYaw() * angle < 0 ? gyro.getYaw() + 360 * Math.signum(angle) : gyro.getYaw();
        // System.out.println(ang);
        chassis.test();
        // System.out.println(enc.get());
        // System.out.println()
        // System.out.println(chassis.motorbrr());
    }

    // CommandSelector selector = new CommandSelector(uuuuuuu
    // "Autonomous",
    // new EmptyAuto(chassis);
    // new Autonomous(chassis, arm, intake, false),
    // new Autonomous(chassis, arm, intake, true),
    // new BallinAutonomous(chassis, arm, intake),
    // new DriveAutonomous(chassis, arm, intake),
    // new NoBalanceAuto(chassis, arm, intake));

    public BooleanSwitch enabled = new BooleanSwitch("Enable", false);
    public DoubleEntry angle = new DoubleEntry("Angle", 0);
    public DoubleEntry distance = new DoubleEntry("Distance", 0);

    public Command getAutoCommand() {
        
        // see Autonomous class for more details
        // SequentialCommandGroup s = new SequentialCommandGroup(
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // chassis.moveTo(5),
        // chassis.turnAngle(enabled, angle),
        // // chassis.balanceCommand()
        // );
        // return chassis.moveTo(4, false);

        // SequentialCommandGroup sex = new SequentialCommandGroup(
        //     new InstantCommand(() -> intake.setSpin(.5)),
        //     new WaitCommand(1),
        //     new InstantCommand(() -> intake.accSpin(1, 2)),
        //     new WaitCommand(2), 
        //     new InstantCommand(() -> intake.setSpin(0)),
        //     chassis.moveTo(-16.5, false),
        // return chassis.moveToToBalnenceBackwards(-10);


        ////////////////Balance auto 
        
        // return new SequentialCommandGroup(
        //     new InstantCommand(() -> intake.moveIntake(0.2), intake),
        //     new WaitCommand(0.2),
        //     new InstantCommand(() -> intake.moveIntake(0), intake),
        //     intake.autoShoot("High")
        // );


        // return chassis.turnAngle(90);
        //turning optimization
        // return new Autonomous(chassis, intake, false);






        //this is the good one
        // return new Autonomous(chassis, intake);

        return new Move(chassis);
        
        
        
        
        
        
        
        
        
        
        // return new SequentialCommandGroup(
        //     // intake.autoShoot("High"),
        //     new InstantCommand(() -> intake.spintake(-.3, false)),
        //     new WaitCommand(0.5),
        //     new InstantCommand(() -> intake.spintake(0, true)),
        //     //chassis.moveTo(-14.5, false)
        //     chassis.moveToToBalnenceBackwards(-10),
        //     new WaitCommand(.2),
        //     chassis.balanceCommand()
        // );

        //stress test
        // return new SequentialCommandGroup(
        //     chassis.moveTo(5, false),
        //     chassis.moveTo(-5, false),
        //     chassis.moveTo(5, false),
        //     chassis.moveTo(-5, false)
        // );


        //return new Autonomous(chassis, intake);
        //return new SequentialCommandGroup(chassis.moveToToBalnenceBackwards(-10), chassis.balanceCommand());
        
        

        //Theoretical code for new autonomous. Highlander auto.
        /* 
        return new SequentialCommandGroup(
        new InstantCommand(() -> intake.moveIntake(0.5) , intake),
        //method to set distance??? Encoder????
        new WaitCommand(0.5),
        chassis.moveTo(5, false),
        chassis.moveTo(-18,false),
        chassis.turnAngle(90),
        new InstantCommand(() -> intake.spintake(0.5 , false), intake),
        chassis.moveTo(8, false),
        new WaitCommand(0.5),
        chassis.turnAngle(90),
        chassis.moveTo(18 , false),
        new InstantCommand(() -> intake.spintake(0.5 , false))
        );
        */


        // return chassis.moveToToBalnenceBackwards(18);
        // return sex;
        //return new SequentialCommandGroup(
            // intake.accSpin(-0.3, 0.1),
            // new WaitCommand(2),
            //new InstantCommand(() -> intake.setSpin(0)),
            // //new WaitCommand(1),
            // chassis.turnAngle(180),
            // chassis.turnAngle(180),
            // chassis.turnAngle(180),
            // chassis.turnAngle(180)
            //new InstantCommand(() -> intake.setSpin(1))
        //);

        // return selector.value();

        // return chassis.balanceCommand();

        // return chassis.turnAngle(-90);

        // if(enabled.value()){
        // return chassis.turnAngle(angle.value());
        // }
    }
}
