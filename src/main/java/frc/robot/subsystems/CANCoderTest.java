package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANCoderTest extends SubsystemBase {
    private CANcoder encoder = new CANcoder(19);
    @Override
    public void periodic() {
        Logger.recordOutput("CANCoder Angle", encoder.getAbsolutePosition().getValueAsDouble());
    }
}
