package frc.robot.subsystems.Wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;

public class WristIOReal implements WristIO{
    public SparkFlex wristMotor = new SparkFlex(WristConstants.id, null);

    public WristIOReal() {
    }

    public void setVoltage(double voltage) {
        wristMotor.setVoltage(voltage);
    }
}
