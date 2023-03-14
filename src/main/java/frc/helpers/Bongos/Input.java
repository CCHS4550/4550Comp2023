package frc.robot.subsystems.Bongos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Input extends SubsystemBase{

    private String code;

    public Input(String code, double lifespan, ArrayList<Input> container){
        this.code = code;

        this.setDefaultCommand(new SequentialCommandGroup(
            new WaitCommand(lifespan),
            new InstantCommand(() -> container.remove(0))
        ));
    }

    public Input(String code){
        this.code = code;
    }

    public String code(){
        return code;
    }

}
