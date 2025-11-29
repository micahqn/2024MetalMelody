package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
    private Pivot pivot;
    
    public enum Goals {
        DEFAULT,
        L3_CORAL,
        L2_CORAL,
        L1_CORAL,
        L2_ALGAE,
        PROCESSOR,

        CLIMBING,
        FLOOR,
    }

    public Superstructure() {
        this.pivot = new Pivot();
    }
}
