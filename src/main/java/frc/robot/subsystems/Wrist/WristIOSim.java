package frc.robot.subsystems.Wrist;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class WristIOSim implements WristIO {
        public DCMotorSim wristMotor = new DCMotorSim(null,
         DCMotor.getNeoVortex(1),
          null);

}
