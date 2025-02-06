// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ExternalEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.DriveCommands;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn
 * motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward
 * motion on the drive motor will propel the robot forward) and copy the
 * reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  private final TalonFX driveTalonFX;
  private final SparkFlex turnSparkFlex;

  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted = true;
  public static final SparkFlexConfig turningConfig = new SparkFlexConfig();
  
  public ModuleIOTalonFX(int index) {

    switch (index) {
      case 0:
        // Front left
        driveTalonFX = new TalonFX(1);
        turnSparkFlex = new SparkFlex(2, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        driveTalonFX.setInverted(false);
        break;
      case 1:
        // Front right
        driveTalonFX = new TalonFX(3);
        turnSparkFlex = new SparkFlex(4, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        break;
      case 2:
        // Back left
        driveTalonFX = new TalonFX(8);
        turnSparkFlex = new SparkFlex(7, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        driveTalonFX.setInverted(false);
        break;
      case 3:
        // Back right
        driveTalonFX = new TalonFX(6);
        turnSparkFlex = new SparkFlex(5, MotorType.kBrushless);
        turnAbsoluteEncoder = turnSparkFlex.getAbsoluteEncoder();
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    
      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.turnCurrentLimit);
      turningConfig.absoluteEncoder
        .inverted(true)
        .positionConversionFactor(DriveConstants.turningFactor)
        .velocityConversionFactor(DriveConstants.turningFactor/60);
      turningConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(DriveConstants.RotationKP, DriveConstants.RotationKI, DriveConstants.RotationKD)
        .outputRange(-1, 1)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(0, DriveConstants.turningFactor); 
    

    turnSparkFlex.setCANTimeout(250);

    turnRelativeEncoder = turnSparkFlex.getEncoder(); // ???
    final ExternalEncoderConfig encoderconfig = new ExternalEncoderConfig();

    encoderconfig.measurementPeriod(10);
    turnRelativeEncoder.setPosition(0.0);
    encoderconfig.measurementPeriod(10);
    encoderconfig.averageDepth(2);

    turnSparkFlex.setInverted(isTurnMotorInverted);

    driveTalonFX.setPosition(0);
    driveTalonFX.setInverted(true);

    turnSparkFlex.setCANTimeout(0);

   
    var config = config();
    var configSpark = configSpark(IdleMode.kBrake);
    configSpark.apply(encoderconfig);

    driveTalonFX.optimizeBusUtilization();

    driveTalonFX.clearStickyFaults();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    REVLibError status2 = REVLibError.kError; // not work maybe
    for (int i = 0; i < 5; i++) {
      status = driveTalonFX.getConfigurator().apply(config);
      turnSparkFlex.configure(configSpark, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      if (status.isOK())
        break;
    }

    if (!status.isOK()) {
      System.out.println(
          "Talon ID "
              + driveTalonFX.getDeviceID()
              + " failed config with error "
              + status.toString());
    }
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = Units.rotationsToRadians(driveTalonFX.getPosition().getValueAsDouble())
        / DriveConstants.DriveGearing;
    inputs.driveVelocityRadPerSec = Units
        .rotationsPerMinuteToRadiansPerSecond(driveTalonFX.getVelocity().getValueAsDouble())
        / DriveConstants.DriveGearing;
    inputs.driveAppliedVolts = driveTalonFX.getMotorVoltage().getValueAsDouble();
    inputs.driveCurrentAmps = new double[] { driveTalonFX.getSupplyCurrent().getValueAsDouble() };

    inputs.turnAbsolutePosition = Rotation2d.fromRadians(turnAbsoluteEncoder.getPosition());
    inputs.turnPosition = Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / DriveConstants.TurnGearing);
    inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
        / DriveConstants.TurnGearing;
    inputs.turnAppliedVolts = turnSparkFlex.getAppliedOutput() * turnSparkFlex.getBusVoltage();
    inputs.turnCurrentAmps = new double[] { turnSparkFlex.getOutputCurrent() };
  }

  private SparkFlexConfig configSpark(IdleMode idle) {
    var turningConfig = new SparkFlexConfig();

    turningConfig.smartCurrentLimit(DriveConstants.turnCurrentLimit);
    turningConfig.voltageCompensation(12.0);

    if (idle != null) {
      turningConfig.idleMode(idle);
    }

    return turningConfig;
  }

  private TalonFXConfiguration config() {
    var talonFXConfig = new TalonFXConfiguration();

    talonFXConfig.Slot0.kP = 1;
    talonFXConfig.Slot0.kI = 0;
    talonFXConfig.Slot0.kD = 0;
    talonFXConfig.Slot0.kS = 0;
    talonFXConfig.Slot0.kV = 0;

    talonFXConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveCurrentLimit; // CurrentLimits.drive;

    talonFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfig.Audio.BeepOnBoot = true;

    return talonFXConfig;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalonFX.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkFlex.setVoltage(volts);
  }

  // @Override
  // public void setDriveBrakeMode(boolean enable) {
  // driveTalonFX.setBrakeMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  // }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    // turnSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    configSpark(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

 
}