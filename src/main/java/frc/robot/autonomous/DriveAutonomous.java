// package frc.robot.autonomous;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.DriveTrain;





// public class DriveAutonomous extends SequentialCommandGroup{
//     public DriveAutonomous(DriveTrain chassis, Arm arm, Intake intake){
//         this.setName("Middle");

//         super.addCommands(
//             intake.toggleArmCommand(),
//             intake.spintake(-.5),
//             new WaitCommand(.5),
//             intake.spintake(0),
//             new WaitCommand(.1),
//             intake.toggleArmCommand(),
            
//             chassis.moveTo(-15, true),
//             chassis.turnAngle(180),
//             chassis.moveToToBalnenceBackwards(-15),


//             // chassis.turnAngle(-90),
//             // intake.spintake(1),
//             // chassis.moveTo(3, false),
//             // intake.spintake(-1),
//             // chassis.turnAngle(-90),
//             // chassis.moveTo(13.6, false),
//             // new WaitCommand(.5),
//             // chassis.moveTo(6, false),
//             chassis.balanceCommand()

            


//         );
        


//     }
// }