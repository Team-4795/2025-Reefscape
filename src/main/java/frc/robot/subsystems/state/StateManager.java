package frc.robot.subsystems.state;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.state.State.Setpoint;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.Util;

public class StateManager extends SubsystemBase {
    private State state;
    private Setpoint setpoint = StateConstants.DEFAULT;
    private Intake intake = Intake.getInstance();
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Swerve swerve = Swerve.getInstance();

    // non-setpoint robot stuff
    public static class OperationStates {
        public static boolean isScoringLeft = true;
        public static boolean aligned = false; 
        public static boolean inScoringDistance = false;
        public static boolean isReefTagOnly = true;
        public static State autoScoreMode = State.L4;
    }

    private static StateManager instance;
    
    public static StateManager initalize() {
        instance = new StateManager();
        return instance;
    }

    public static StateManager getInstance() {
        if(instance != null) {
            return instance;
        } else {
            return initalize();
        }
    }

    private StateManager() {
        setState(State.STOW);
    }

    private Setpoint getDynamicSetpoint(State state) {
        Setpoint setpoint = new Setpoint(null, null, 0.0);
        Translation3d target = switch (state) {
            case L4_DYNAMIC -> StateConstants.VFB_TARGETS.get(state);
            default -> new Translation3d();
        };

        double distance = new Translation2d(target.getX(), target.getY()).getDistance(swerve.getState().Pose.getTranslation());
        double angle = Math.acos(distance / ArmConstants.Sim.LENGTH);
        double heightWithAngle = StateConstants.MIN_AXIS_HEIGHT + Math.sin(angle) * ArmConstants.Sim.LENGTH;

        if(distance > ArmConstants.Sim.LENGTH) {
            return StateConstants.DEFAULT;
        } else if(heightWithAngle < target.getZ()) {
            setpoint.elevatorHeight = target.getZ() - heightWithAngle;
            setpoint.armAngle = angle;
        } else {
            angle = -angle;
            heightWithAngle = StateConstants.MIN_AXIS_HEIGHT + Math.sin(heightWithAngle) * ArmConstants.Sim.LENGTH;
            setpoint.elevatorHeight = target.getZ() - heightWithAngle;
            setpoint.armAngle = angle;
        }

        return setpoint;
    }

    private void setState(State state) {
        this.state = state;
        Setpoint targetSetpoint;

        if(state.setpoint.equals(StateConstants.DYNAMIC)) {
            targetSetpoint = getDynamicSetpoint(state);
        } else {
            targetSetpoint = state.setpoint;
        }

        Util.nullOrDo(targetSetpoint.armAngle, (value) -> setpoint.armAngle = value);
        Util.nullOrDo(targetSetpoint.elevatorHeight, (value) -> setpoint.elevatorHeight = value);
        Util.nullOrDo(targetSetpoint.intakeSpeed, (value) -> setpoint.intakeSpeed = value);
    }

    public Command stateCommand(State state) {
        Command command = 
        Commands.runOnce(() -> setState(state)).andThen(
        Commands.parallel(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> arm.setGoal(setpoint.armAngle)),
                    Commands.waitUntil(() -> elevatorCanMove())
                        .andThen(() -> elevator.setGoalHeight(setpoint.elevatorHeight))
                ),
                Commands.sequence(
                    Commands.runOnce(() -> elevator.setGoalHeight(setpoint.elevatorHeight)),
                    Commands.waitUntil(() -> armCanMove())
                        .andThen(() -> arm.setGoal(setpoint.armAngle))
                ),
                () -> elevator.getPosition() < setpoint.elevatorHeight
            ),
            Commands.runOnce(() -> intake.setIntakeSpeed(setpoint.intakeSpeed))
        ));

        command.addRequirements(this);

        return command;
    }

    public boolean elevatorCanMove() {
        return arm.getAngle() > -1.5;
    }

    public boolean armCanMove() {
        return elevator.getPosition() < .2;
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("StateManager/OperationStates/autoScoreMode", OperationStates.autoScoreMode);
        Logger.recordOutput("StateManager/OperationStates/aligned", OperationStates.aligned);
        Logger.recordOutput("StateManager/OperationStates/inScoringDistance", OperationStates.inScoringDistance);
        Logger.recordOutput("StateManager/OperationStates/isReefTagOnly", OperationStates.isReefTagOnly);

        SmartDashboard.putBoolean("Score/isLeftL4", OperationStates.autoScoreMode == State.L4 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isLeftL3", OperationStates.autoScoreMode == State.L3 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isLeftL2", OperationStates.autoScoreMode == State.L2 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL4", OperationStates.autoScoreMode == State.L4 && !OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL3", OperationStates.autoScoreMode == State.L3 && !OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL2", OperationStates.autoScoreMode == State.L2 && !OperationStates.isScoringLeft);

        Logger.recordOutput("Score/Dynamic/TargetPose", StateConstants.VFB_TARGETS.get(State.L4_DYNAMIC));

        SmartDashboard.updateValues();
    }
}
