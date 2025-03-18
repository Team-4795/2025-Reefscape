package frc.robot.subsystems.state;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.state.State.Setpoint;
import frc.robot.util.Util;

public class StateManager extends SubsystemBase {
    private State state;
    private Setpoint setpoint = StateConstants.DEFAULT;
    private Intake intake = Intake.getInstance();
    private Arm arm = Arm.getInstance();
    private Elevator elevator = Elevator.getInstance();

    // non-setpoint robot stuff
    public static class OperationStates {
        public static boolean isScoringLeft = true;
        public static boolean aligned = false; 
        public static boolean inScoringDistance = false;
        public static boolean isReefTagOnly = true;
        public static int autoScoreMode = 4;
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
        this.state = state;
        Util.nullOrDo(state.setpoint.armAngle, (value) -> setpoint.armAngle = value);
        Util.nullOrDo(state.setpoint.elevatorHeight, (value) -> setpoint.elevatorHeight = value);
        Util.nullOrDo(state.setpoint.intakeSpeed, (value) -> setpoint.intakeSpeed = value);
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

        SmartDashboard.putBoolean("Score/isLeftL4", OperationStates.autoScoreMode == 4 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isLeftL3", OperationStates.autoScoreMode == 3 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isLeftL2", OperationStates.autoScoreMode == 2 && OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL4", OperationStates.autoScoreMode == 4 && !OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL3", OperationStates.autoScoreMode == 3 && !OperationStates.isScoringLeft);
        SmartDashboard.putBoolean("Score/isRightL2", OperationStates.autoScoreMode == 2 && !OperationStates.isScoringLeft);

        SmartDashboard.updateValues();
    }
}
