package frc.diagnostics;


import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.SendableCameraWrapper;


public class Camera extends ShuffleManager{


    private ComplexWidget cam;
    private SendableCameraWrapper camera;
    private GenericEntry entry;


    /**
     *
     * @param title
     * @param defaultValue
     * @param min
     * @param max
     */
    public Camera(String title, Boolean defaultValue){
        System.out.print(pos);
        camera = SendableCameraWrapper.wrap(VideoSource.enumerateSources()[0]);
        cam =
        Shuffleboard.getTab("Camera_Live")
        .add(camera)
        .withPosition(pos.x, pos.y)
        .withSize(1, 1);


        //entry = widget.getEntry();
        pos.translate(1, 0);
        if(pos.x >= 7) pos.setLocation(1, pos.y + 2);
    }


}


