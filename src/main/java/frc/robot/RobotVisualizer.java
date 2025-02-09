package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class RobotVisualizer extends SubsystemBase {
    private LoggedMechanism2d robotVisualizer = new LoggedMechanism2d(2, 2.5);    
    private LoggedMechanismRoot2d robotRoot = robotVisualizer.getRoot("Robot Visualizer", 1, 0.1);
    
    private LoggedMechanismLigament2d elevatorstage1Ligament = robotRoot.append(new LoggedMechanismLigament2d("Elevator Stage 1 Ligament", ElevatorConstants.maxDistance/.8, 90, 10, new Color8Bit(Color.kRed)));
    private LoggedMechanismLigament2d elevatorstage2Ligament = robotRoot.append(new LoggedMechanismLigament2d("Elevator Stage 2 Ligament", ElevatorConstants.maxDistance, 90, 10, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d armLigament = elevatorstage2Ligament.append(new LoggedMechanismLigament2d("Arm Ligament", ArmConstants.Sim.LENGTH, -90, 10, new Color8Bit(Color.kGreen)));

    public RobotVisualizer() {

    }

    @Override
    public void periodic() {
        armLigament.setAngle(Units.radiansToDegrees(Arm.getInstance().getAngle()) - 90);
        elevatorstage2Ligament.setLength(ElevatorConstants.maxDistance/.8 + Elevator.getInstance().getPosition());
        Logger.recordOutput("RobotVisualizer", robotVisualizer);
    }
}
