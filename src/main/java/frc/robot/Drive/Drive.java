package frc.robot.Drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.mutable.MutableMeasureBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Drive.DriveConstants.Mode;
import frc.robot.Drive.GyroIO.GyroIOInputs;
import frc.robot.Drive.ModuleIO.ModuleIOInputs;


public class Drive extends SubsystemBase {
    private static Drive instance;
    private static final double MAX_LINEAR_SPEED = Units.feetToMeters(5);
    private static final double TRACK_WIDTH_X = Units.inchesToMeters(21.25);
    private static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.25);
    private static final double DRIVE_BASE_RADIUS = 
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    private static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysId;
    private final MutableMeasure<VoltageUnit,Voltage,MutVoltage> m_appliedVoltage = new MutVoltage(0, 0, Volt);
    private final MutableMeasure<AngleUnit,Angle,MutAngle> m_position = new MutAngle(0,0, Radian);
    private final MutableMeasure<AngularVelocityUnit, AngularVelocity,MutAngularVelocity> m_velocity = new MutAngularVelocity(0, 0, RadiansPerSecond);
    private static SwerveDriveKinematics Modulekinematics = new SwerveDriveKinematics(getModuleTranslation2d());

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslation2d());
    private Rotation2d rawGyroRotation2d = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = 
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

    private SwerveDrivePoseEstimator poseEstimator =
        new SwerveDrivePoseEstimator(kinematics, rawGyroRotation2d, lastModulePositions, new Pose2d());
    
    public static Drive getInstance(){
        return instance;
    }

    public static Drive initialize (GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
        if (instance == null) {
            instance = new Drive(gyro, fl, fr, bl, br);
        }
        return instance;
    }

    public Drive(
        GyroIO gyroIO,
        ModuleIO flModuleIO,
        ModuleIO frModuleIO,
        ModuleIO blModuleIO,
        ModuleIO brModuleIO) {
            this.gyroIO = gyroIO;
            modules[0] = new Module(flModuleIO, 0);
            modules[1] = new Module(frModuleIO, 1);
            modules[2] = new Module(blModuleIO, 2);
            modules[3] = new Module(brModuleIO, 3);


            AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                () -> Kinematics.toChassisSpeeds(getModuleStates()),
                this:: runVelocity,
                new HolonomicPathFollowerConfig(
                    MAX_LINEAR_SPEED, DRIVE_BASE_RADIUS, new ReplanningConfig()),
                () ->
                    DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == Alliance.Red,
                this);
            Pathfinding.setPathfinder(new LocalADStarAK());
            PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]))
                });
            PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
            sysId =
                new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            for (int i = 0; i < 4; i++){
                                modules[i].runCharacterization(voltage.in(Volts));
                            }
                        },
                        log -> {
                            log.motor("driveSparkMax")
                                .voltage(m_appliedVoltage.mustreplace(inputs.driveAppliedVolts, Volts))
                                .angularPosition(m_position.must_replace(inputs.drivePositionRad, Radians))
                                .AngularVelocity(
                                    m_velocity.must_replace(inputs.driveVelocityRadPerSec, RadiansPerSecond));
       
                        },
                    this));
        }
        
        public void periodic() {
            gyroIO.updateInputs(inputs);
            Logger.processInputs("Drive/Gyro", gyroIOInputs);
            for (var module : modules) {
                module.periodic();
            }

        
        if (DriverStation.isDisabled()){
            for (var module : modules){
                module.stop();
            }
        }
    
    if (DriverStation.isDisabled()){
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    SwerveModulePosition[] modulePositions = SwerveModulePosition();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex ++){
        moduleDeltas [moduleIndex] = 
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    if (gyro.connected) {

        rawGyroRotation2d = gyroInputs.yawPosition;
    } else {

        Twist2d twist = Kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation2d = rawGyroRotation2d.plus(new Rotation2d(twist.dtheta));
    }

poseEstimator.update(rawGyroRotation2d, modulePositions);

        }


public void runVelocity(ChassisSpeeds speeds){

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = Kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,MAX_LINEAR_SPEED);

    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {

        optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
}

public void stop(){
    runVelocity(new ChassisSpeeds());
}




public void stopWithx() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
        headings[i] = getModuleTranslations()[i].getAngle();
    }
    Modulekinematics.resetHeadings();
    stop();
}

    public void zeroHeading() {
        if (DriveConstants.currentMode == Mode.SIM){
            poseEstimator.resetPosition(
                new Rotation2d(),
                new SwerveModulePosition[]{
                    modules[0].getPosition(),
                    modules[1].getPosition(),
                    modules[2].getPosition(), 
                    modules[3].getPosition(),
                },
                getPose());
        }

        gyroIO.reset();
    }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return sysId.dynamic(direction);
        }     

        @AutoLogOutput (key = "SwerveStates/Measured")
        private SwerveModuleState[] getModuleStates(){
            SwerveModuleState[] states = new SwerveModuleState[4];
            for (int i = 0; i < 4; i++){
                states[i] = modules[i].getState();
            }
            return states;
        }
    
        public Rotation2d getRotation() {
            return getPose().getRotation();
        }
    

        public void setPose(Pose2d pose) {
            poseEstimator.resetPosition(rawGyroRotation2d, getModulePositions(),pose);
        }
    







        public void addVisionMeasurement(Pose2d visionPose, double timestamp){
            poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }

     
        
        public double getMaxLinearSpeedMeterPerSec(){
            return MAX_LINEAR_SPEED;
        }


        public double getMaxAngluarSpeeddRadPerSec(){
            return MAX_ANGULAR_SPEED;
        }


        public static Translation2d[] getModuleTranslation2d(){
            return new Translation2d[]{
                new Translation2d(TRACK_WIDTH_X/2.0, TRACK_WIDTH_Y/2.0),
                new Translation2d(TRACK_WIDTH_X/2.0, TRACK_WIDTH_Y/2.0),
                new Translation2d(TRACK_WIDTH_X/2.0, TRACK_WIDTH_Y/2.0),
                new Translation2d(TRACK_WIDTH_X/2.0, TRACK_WIDTH_Y/2.0),  
            };
        }
    }
    