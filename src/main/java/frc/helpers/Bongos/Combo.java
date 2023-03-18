package frc.helpers.Bongos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Combo extends SubsystemBase{

    private ArrayList<Input> sequence = new ArrayList<Input>();

    private int breakoff;
    public Combo(Comedy controller, Command activation, Input... inputs){
        ArrayList<Input> concurrent = controller.inputs();
        for(Input i : inputs) sequence.add(i);
        this.setDefaultCommand(new SequentialCommandGroup(
            new RunCommand(() -> {}, this){
                @Override
                public boolean isFinished(){
                    for(int i = 0; i < concurrent.size(); i++){
                        if(i + sequence.size() > concurrent.size()) return false;
                        for(int j = 0; j < sequence.size(); j++){
                            if(!concurrent.get(i + j).code().equals(sequence.get(j).code())) break;
                            if(j == sequence.size() - 1){
                                breakoff = i + j;
                                return true;
                            }
                        }
                    }
                    return false;
                }
            },
            new InstantCommand(() -> {
                for(int i = 0; i < breakoff; i++){
                    concurrent.remove(0);
                }
            }),
            activation
        ));
    }

    public Combo(Comedy controller, Command activation, String... inputs){
        ArrayList<Input> concurrent = controller.inputs();
        for(String s : inputs) sequence.add(new Input(s));
        this.setDefaultCommand(new SequentialCommandGroup(
            new RunCommand(() -> {}, this){
                @Override
                public boolean isFinished(){
                    for(int i = 0; i < concurrent.size(); i++){
                        if(i + sequence.size() > concurrent.size()) return false;
                        for(int j = 0; j < sequence.size(); j++){
                            if(!concurrent.get(i + j).code().equals(sequence.get(j).code())) break;
                            if(j == sequence.size() - 1){
                                breakoff = i + j;
                                return true;
                            } 
                        }
                    }
                    return false;
                }
            },
            new InstantCommand(() -> {
                for(int i = 0; i < breakoff + 1; i++){
                    if(concurrent.size() == 0) break;
                    concurrent.remove(0);
                }
            }),
            new InstantCommand(() -> activation.schedule())
        ));
    }
    
}
