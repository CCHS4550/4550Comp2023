package frc.helpers.Bongos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Comedy extends SubsystemBase{
    private ArrayList<Input> inputs = new ArrayList<Input>();
    private ArrayList<Combo> combos = new ArrayList<Combo>();

    private double lifespan;

    public Comedy(double lifespan) {
        this.lifespan = lifespan;
        this.setDefaultCommand(new RunCommand(() -> {
            for(Input i : inputs) System.out.print(i.code());
        }, this));
    }

    public ArrayList<Input> inputs() {return inputs;}

    public void addCombos(Combo... combos){
        for(Combo c : combos) this.combos.add(c);
    }

    public Command add(String s){
        return new InstantCommand(() -> {
            Input i = new Input(s, lifespan, inputs);
            inputs.add(i);
        });
    }
}
