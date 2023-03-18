package frc.helpers.Bongos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Input extends SubsystemBase{

    private String code;

    public Input(String code, double lifespan, ArrayList<Input> container){
        this.code = code;
        start(lifespan, container).schedule();
    }

    public Command start(double lifespan, ArrayList<Input> container){
        return new SequentialCommandGroup(
            new WaitCommand(lifespan),
            new InstantCommand(() -> {
                if(container.size() > 0) container.remove(this);
            })
        );
    }

    public Input(String code){
        this.code = code;
    }

    public String code(){
        return code;
    }

}
