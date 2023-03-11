package frc.robot;

import java.util.ResourceBundle.Control;

import javax.management.InstanceAlreadyExistsException;
import javax.print.attribute.standard.JobHoldUntil;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.diagnostics.BooleanSwitch;
import frc.diagnostics.CommandSelector;
import frc.helpers.OI;
import frc.maps.ControlMap;

// import frc.robot.autonomous.Autonomous;
// import frc.robot.autonomous.BallinAutonomous;
// import frc.robot.autonomous.DriveAutonomous;

// import frc.robot.autonomous.*;
import frc.diagnostics.*;
import frc.robot.autonomous.Autonomous;
// import frc.robot.subsystems.MotorEx;
import frc.robot.subsystems.*;

public class RobotContainer {
    // must instantiate an object of each subsystem you use
    private DriveTrain chassis = new DriveTrain();
    private Intake intake = new Intake(chassis);
    // private Arm arm = new Arm();

    Joystick[] controllers = OI.joystickArray;

    DoubleEntry pow = new DoubleEntry("power", 0.1);

    public RobotContainer() {
        // chassis.defaultAccelTime
        chassis.setDefaultCommand(new RunCommand(() -> chassis.axisDrive(
                OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL) * (OI.axis(0, ControlMap.LT) > 0.5 ? 1 : 0.7) * (OI.axis(0, ControlMap.RT) > 0.5 ? 0.5 : 1),
                OI.axis(0, ControlMap.R_JOYSTICK_HORIZONTAL) * (OI.axis(0, ControlMap.LT) > 0.5 ? 1.3 : 1) * (OI.axis(0, ControlMap.RT) > 0.5 ? 0.75 : 1),
                chassis.defaultAccelTime), chassis));
        // arm.setDefaultCommand(new RunCommand(() -> arm.move(OI.axis(1,
        // ControlMap.L_JOYSTICK_VERTICAL) * 0.75), arm));
        // intake.setDefaultCommand(
        // new RunCommand(() -> intake.spintake(OI.axis(1,
        // ControlMap.R_JOYSTICK_VERTICAL) * pow.value()), intake));
        // Math.cos(OI.dPadAng(1) >= 0 ? Math.toRadians(OI.dPadAng(1)) : Math.PI/2)

        intake.setDefaultCommand(
                new RunCommand(() -> intake.manageIntake(
                        OI.axis(1, ControlMap.L_JOYSTICK_VERTICAL) * (OI.axis(1, ControlMap.RT) > 0.5 ? 0.7 : 0.5) * (OI.button(1, ControlMap.RB_BUTTON) ? 0.5 : 1),
                        OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL),
                        OI.axis(1, ControlMap.LT) > 0.5), intake));


        // intake.setDefaultCommand(new RunCommand(() -> intake.printEncoder(),
        // intake));
        configureButtons();
        // Humza wrote the line above
        // arms.setDefualtCommand(new RunCommand(() -> arms.setSpeed(OI.dPad(1, )),
        // arms);
    }

    // This is Tyler's contribution to this code, you're welcome!!!
    private void configureButtons() {
        // this is how you do teleop stuff
        // when you create a trigger or trigger subclass, it will run whatever you want
        // run when you specify

        // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Button.html
        // list of modifiers to control what happens for trigger objects
        // (joystickbuttons extend trigger)

        // https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html
        // triggers are like buttons, but you can control when they go off
        // any logic amongst triggers must be done with .and, .negate, and others
        // see link for full list of logic operators

        new JoystickButton(controllers[1], ControlMap.A_BUTTON)
                .onTrue(new InstantCommand(() -> chassis.gyro.reset(), chassis));

        new JoystickButton(controllers[1], ControlMap.Y_BUTTON)
                .onTrue(intake.toggle());
        // new JoystickButton(controllers[0], ControlMap.B_BUTTON)
        // .onTrue(new SequentialCommandGroup(new InstantCommand(() ->
        // chassis.gyro.reset(), chassis), chassis.turnAngle(180)));

        new JoystickButton(controllers[0], ControlMap.A_BUTTON)
                .onTrue(new InstantCommand(() -> chassis.toggleSlowMode(), chassis));

        // new JoystickButton(controllers[1], ControlMap.B_BUTTON)
                // .onTrue(new InstantCommand(() -> intake.toggle()));
        new JoystickButton(controllers[1], ControlMap.A_BUTTON)
                .onTrue(intake.autoShoot("High"));
        new JoystickButton(controllers[1], ControlMap.B_BUTTON)
                .onTrue(intake.autoShoot("Middle"));
        new JoystickButton(controllers[1], ControlMap.X_BUTTON)
                .onTrue(new InstantCommand(() -> intake.resetEncoders()));

    }

    DoubleEntry turnval = new DoubleEntry("bollocks", 0);
    EncoderTest enc = new EncoderTest(0);
    AHRS gyro = chassis.gyro;
    void test() {
        double angle = -1;
        chassis.arcade(OI.axis(0, ControlMap.L_JOYSTICK_VERTICAL), OI.axis(0, ControlMap.R_JOYSTICK_HORIZONTAL));
        double ang = gyro.getYaw() * angle < 0 ? gyro.getYaw() + 360 * Math.signum(angle) : gyro.getYaw();
        System.out.println(ang);
        // System.out.println(enc.get());
        // System.out.println()
        // System.out.println(chassis.motorbrr());
    }

    // CommandSelector selector = new CommandSelector(
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
        return new Autonomous(chassis, intake, false);
        
        

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
