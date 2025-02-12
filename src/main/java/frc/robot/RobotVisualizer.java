package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;

public class RobotVisualizer extends SubsystemBase {
    private LoggedMechanism2d robotVisualizer = new LoggedMechanism2d(2, 2.5);    
    private LoggedMechanismRoot2d robotRoot = robotVisualizer.getRoot("Robot Visualizer", 1, 0.1);
    
    private LoggedMechanismLigament2d elevatorstage1Ligament = robotRoot.append(new LoggedMechanismLigament2d("Elevator Stage 1 Ligament", ElevatorConstants.maxDistance/.9, 90, 10, new Color8Bit(Color.kRed)));
    private LoggedMechanismLigament2d elevatorstage2Ligament = robotRoot.append(new LoggedMechanismLigament2d("Elevator Stage 2 Ligament", ElevatorConstants.maxDistance, 90, 9, new Color8Bit(Color.kBlue)));
    private LoggedMechanismLigament2d armLigament = elevatorstage2Ligament.append(new LoggedMechanismLigament2d("Arm Ligament", ArmConstants.Sim.LENGTH, -90, 8, new Color8Bit(Color.kGreen)));
    private LoggedMechanismLigament2d intakeLigament = armLigament.append(new LoggedMechanismLigament2d("Intake Ligament", 0.1, -90, 7, new Color8Bit(Color.kYellow)));

    public RobotVisualizer() {

    }

    @Override
    public void periodic() {
        intakeLigament.setAngle(Units.rotationsToDegrees(Intake.getInstance().getPosition()));
        armLigament.setAngle(Units.radiansToDegrees(Arm.getInstance().getAngle()) - 90);
        elevatorstage2Ligament.setLength(ElevatorConstants.maxDistance/.9 + Elevator.getInstance().getPosition());
        Logger.recordOutput("RobotVisualizer", robotVisualizer);
        Pose2d fakePose = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape).getTagPose(7).get().toPose2d().plus(new Transform2d(1, 0, new Rotation2d(Math.PI)));
        Logger.recordOutput("Fake robot pose", fakePose);
    }
}
