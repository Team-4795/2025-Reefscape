package frc.robot.subsystems.state;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.state.State.Setpoint;
import frc.robot.util.Util;
import frc.robot.subsystems.Wrist.Wrist;

public class StateManager extends SubsystemBase {
    private State state;
    private State lastState = State.DYNAMIC;
    private Setpoint setpoint = StateConstants.STOW;
    private Intake intake = Intake.getInstance();
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Wrist wrist = Wrist.getInstance();

    // non-setpoint robot stuff
    public static class OperationStates {
        public static boolean canAlign = true;
        public static boolean isScoringLeft = true;
        public static boolean aligned = false; 
        public static boolean inScoringDistance = false;
        public static boolean isBargeFowards = true;
        public static boolean isReefTagOnly = true;
        public static State autoScoreMode = State.L4;
        public static State autoAlgaeMode = State.HIGH_ALGAE;
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
        this.state = State.DYNAMIC;
    }

    private void setState(State state) {
        this.lastState = this.state;
        this.state = state;
        Logger.recordOutput("state being set", state);
        Logger.recordOutput("state arm angle is", state.setpoint.equals(StateConstants.STOW));
        Util.nullOrDo(state.setpoint.armAngle, (value) -> {setpoint.armAngle = value;});
        Util.nullOrDo(state.setpoint.elevatorHeight, (value) -> setpoint.elevatorHeight = value);
        Util.nullOrDo(state.setpoint.intakeSpeed, (value) -> setpoint.intakeSpeed = value);
        Util.nullOrDo(state.setpoint.wristAngle, (value) -> setpoint.wristAngle = value);
    }

    public Command stateCommand(State state) {
        Command command = 
        Commands.runOnce(() -> setState(state)).andThen(
        Commands.parallel(
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> Logger.recordOutput("armFirst?", true)),
                    Commands.runOnce(() -> arm.setGoal(setpoint.armAngle)),
                    Commands.waitUntil(() -> elevatorCanMove())
                        .andThen(() -> elevator.setGoalHeight(setpoint.elevatorHeight))
                ),
                Commands.sequence(
                    Commands.runOnce(() -> Logger.recordOutput("elevatorFirst?", true)),
                    Commands.runOnce(() -> elevator.setGoalHeight(setpoint.elevatorHeight)),
                    Commands.waitUntil(() -> armCanMove())
                        .andThen(() -> arm.setGoal(setpoint.armAngle))
                ),
                () -> (elevator.getPosition() < setpoint.elevatorHeight  || OperationStates.aligned) && (arm.getAngle() < setpoint.armAngle)
            ),
            Commands.runOnce(() -> intake.setIntakeSpeed(setpoint.intakeSpeed))
        )).andThen(() -> wrist.setGoal(setpoint.wristAngle));

        command.addRequirements(this);

        return command;
    }

    public boolean elevatorCanMove() {
        if(state == State.BACKWARD_NET) {
            return true;
        } 
        // else if(state = State.L2) {
        //     return arm.getAngle() > -(Math.PI / 2.0);
        // }
        else if(state == State.L4) {
            return arm.getAngle() > -Math.PI/2;
        }
        else {
            return MathUtil.isNear(setpoint.armAngle, arm.getAngle(), 0.05);
        }
    }

    public boolean armCanMove() {
        if(state == State.VSTOW && lastState == State.L4) {
            Logger.recordOutput("L4 to Vstow", true);
            return true;
        } 
        else if(state == State.L4 && lastState == State.VSTOW) {
            Logger.recordOutput("L4 to Vstow", false);
            return MathUtil.isNear(setpoint.elevatorHeight, elevator.getPosition(), 0.03);
        } 
        else {
            return elevator.getPosition() < .4;
        }
    }

    public State getState() {
        return state;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("StateManager/OperationStates/canAlign", OperationStates.canAlign);
        Logger.recordOutput("StateManager/OperationStates/autoScoreMode", OperationStates.autoScoreMode);
        Logger.recordOutput("StateManager/OperationStates/aligned", OperationStates.aligned);
        Logger.recordOutput("StateManager/OperationStates/inScoringDistance", OperationStates.inScoringDistance);
        Logger.recordOutput("StateManager/OperationStates/isReefTagOnly", OperationStates.isReefTagOnly);
        Logger.recordOutput("StateManager/OperationStates/autoAlgaeMode", OperationStates.autoAlgaeMode);
        Logger.recordOutput("StateManager/OperationStates/isBargeFowards", OperationStates.isBargeFowards);

        Logger.recordOutput("StateManager/Setpoint/Arm Angle", setpoint.armAngle);
        Logger.recordOutput("StateManager/Setpoint/Elevator Height", setpoint.elevatorHeight);
        Logger.recordOutput("StateManager/Setpoint/Intake speed", setpoint.intakeSpeed);
        Logger.recordOutput("StateManager/Setpoint/Wrist Angle", setpoint.wristAngle);

        Logger.recordOutput("StateManager/State", state);
        Logger.recordOutput("StateManager/Last State", lastState);

        SmartDashboard.putBoolean("Score/isLeftL4", OperationStates.autoScoreMode == State.L4 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isLeftL3", OperationStates.autoScoreMode == State.L3 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isLeftL2", OperationStates.autoScoreMode == State.L2 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL4", OperationStates.autoScoreMode == State.L4 && !OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL3", OperationStates.autoScoreMode == State.L3 && !OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL2", OperationStates.autoScoreMode == State.L2 && !OperationStates.isScoringLeft);

        SmartDashboard.updateValues();
    }
}
