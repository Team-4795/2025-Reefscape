package frc.robot.subsystems.Drive.Drive;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Drive.Drive.DriveConstants.Mode;


public class Drive extends SubsystemBase {
    private static Drive instance;
    private static final SwerveDriveKinematics K_DRIVE_KINEMATICS = new SwerveDriveKinematics(getModuleTranslation2d());
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final Module[] modules = new Module[4];
    private static SysIdRoutine sysId;
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

            RobotConfig config;
            try{
              config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
              // Handle exception as needed
              e.printStackTrace();
            } finally {
                config = null; // if something went horribly wrong
            }
        
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(DriveConstants.TranslationKP, DriveConstants.TranslationKI, DriveConstants.TranslationKD), // Translation PID constants
                            new PIDConstants(DriveConstants.RotationKP, DriveConstants.RotationKI, DriveConstants.RotationKD) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                      // Boolean supplier that controls when the path will be mirrored for the red alliance
                      // This will flip the path being followed to the red side of the field.
                      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        
                      var alliance = DriverStation.getAlliance();
                      if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                      }
                      return false;
                    },
                    this // Reference to this subsystem to set requirements
            );
          }
         
        
           
           public void robotInit(){

           
           
           
           
            //Pathfinding.setPathfinder(new LocalADStarAK()); // AK code needs to be added
            PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
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
                                .voltage(m_appliedVoltage.mut_replace(inputs.driveAppliedVolts, Volts))
                                .angularPosition(m_position.mut_replace(inputs.drivePositionRad, Radians))
                                .angularVelocity(
                                    m_velocity.mut_replace(inputs.driveVelocityRadPerSec, RadiansPerSecond));
       
                        },
                    this));
                }
        
        public void periodic() {
            gyroIO.updateInputs(gyroInputs);;
            Logger.processInputs("Drive/Gyro", gyroInputs);
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

    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex ++){
        moduleDeltas [moduleIndex] = 
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    if (gyroInputs.connected) {

        rawGyroRotation2d = gyroInputs.yawPosition;
    } else {

        Twist2d twist = K_DRIVE_KINEMATICS.toTwist2d(moduleDeltas, moduleDeltas);
        rawGyroRotation2d = rawGyroRotation2d.plus(new Rotation2d(twist.dtheta));
    }

poseEstimator.update(rawGyroRotation2d, modulePositions);

        }


public void runVelocity(ChassisSpeeds speeds){

    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = K_DRIVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,DriveConstants.MAX_LINEAR_SPEED);

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
        headings[i] = getModuleTranslation2d()[i].getAngle();
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


        private SwerveModulePosition[] getModulePositions(){
            SwerveModulePosition[] states = new SwerveModulePosition[4];
            for (int i = 0; i < 4; i++){
                states[i] = modules[i].getPosition();
            }
            return states;
        }
    
        public void setModuleStates(SwerveModuleState[] desiredStates) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, null); // DriveConstants.kMaxSpeedMetersPerSecond
            for (int i = 0; i < 4; i++) {
                modules[i].setDesiredState(desiredStates[i]);
            }
        }

        @AutoLogOutput(key = "odometry/Robot")
        public Pose2d getPose(){
            return poseEstimator.getEstimatedPosition();
        }

        public Rotation2d getRotation() {
            return getPose().getRotation();
        }
    

        public void setPose(Pose2d pose) {
            poseEstimator.resetPosition(rawGyroRotation2d, getModulePositions(),pose);
        }
    
        private ChassisSpeeds getRobotRelativeSpeeds(){
            return K_DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
            
        }

        public void driveRobotRelative(ChassisSpeeds speeds){
            SwerveModuleState[] moduleStates = K_DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
            setModuleStates(moduleStates);
        }


        public void addVisionMeasurement(Pose2d visionPose, double timestamp){
            poseEstimator.addVisionMeasurement(visionPose, timestamp);
        }

     
        
        public double getMaxLinearSpeedMetersPerSec(){
            return DriveConstants.MAX_LINEAR_SPEED;
        }


        public double getMaxAngularSpeedRadPerSec(){
            return DriveConstants.MAX_ANGULAR_SPEED;
        }


        public static Translation2d[] getModuleTranslation2d(){
            return new Translation2d[]{
                new Translation2d(DriveConstants.TRACK_LENGTH/2.0, DriveConstants.TRACK_WIDTH/2.0),
                new Translation2d(DriveConstants.TRACK_LENGTH/2.0, DriveConstants.TRACK_WIDTH/2.0),
                new Translation2d(DriveConstants.TRACK_LENGTH/2.0, DriveConstants.TRACK_WIDTH/2.0),
                new Translation2d(DriveConstants.TRACK_LENGTH/2.0, DriveConstants.TRACK_WIDTH/2.0),  
            };
        }

        public static void setInstance(Drive instance) {
            Drive.instance = instance;
        }

        public static double getMaxLinearSpeed() {
            return DriveConstants.MAX_LINEAR_SPEED;
        }

        public static double getTrackWidthX() {
            return DriveConstants.TRACK_LENGTH;
        }

        public static double getTrackWidthY() {
            return DriveConstants.TRACK_WIDTH;
        }

        public static double getDriveBaseRadius() {
            return DriveConstants.DRIVE_BASE_RADIUS;
        }

        public static double getMaxAngularSpeed() {
            return DriveConstants.MAX_ANGULAR_SPEED;
        }

        public static SwerveDriveKinematics getkDriveKinematics() {
            return K_DRIVE_KINEMATICS;
        }

        public GyroIO getGyroIO() {
            return gyroIO;
        }

        public GyroIOInputsAutoLogged getGyroIOInputs() {
            return gyroInputs;
        }

        public ModuleIOInputsAutoLogged getInputs() {
            return inputs;
        }

        public Module[] getModules() {
            return modules;
        }

        public SysIdRoutine getSysId() {
            return sysId;
        }

        public MutableMeasure<VoltageUnit, Voltage, MutVoltage> getM_appliedVoltage() {
            return m_appliedVoltage;
        }

        public MutableMeasure<AngleUnit, Angle, MutAngle> getM_position() {
            return m_position;
        }

        public MutableMeasure<AngularVelocityUnit, AngularVelocity, MutAngularVelocity> getM_velocity() {
            return m_velocity;
        }

        public static SwerveDriveKinematics getModulekinematics() {
            return Modulekinematics;
        }

        public static void setModulekinematics(SwerveDriveKinematics modulekinematics) {
            Modulekinematics = modulekinematics;
        }

        public SwerveDriveKinematics getKinematics() {
            return kinematics;
        }

        public void setKinematics(SwerveDriveKinematics kinematics) {
            this.kinematics = kinematics;
        }

        public Rotation2d getRawGyroRotation2d() {
            return rawGyroRotation2d;
        }

        public void setRawGyroRotation2d(Rotation2d rawGyroRotation2d) {
            this.rawGyroRotation2d = rawGyroRotation2d;
        }

        public SwerveModulePosition[] getLastModulePositions() {
            return lastModulePositions;
        }

        public void setLastModulePositions(SwerveModulePosition[] lastModulePositions) {
            this.lastModulePositions = lastModulePositions;
        }

        public SwerveDrivePoseEstimator getPoseEstimator() {
            return poseEstimator;
        }

        public void setPoseEstimator(SwerveDrivePoseEstimator poseEstimator) {
            this.poseEstimator = poseEstimator;
        }
    }
    
    
    
    