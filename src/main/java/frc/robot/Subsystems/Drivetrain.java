package frc.robot.Subsystems;

import frc.robot.Util.*;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;



public class Drivetrain {
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    swerveModule moduleFR = new swerveModule(Settings.DRIVE_FR_ID, Settings.STEER_FR_ID, Settings.ENCODER_FR_ID);
    swerveModule moduleRR = new swerveModule(Settings.DRIVE_RR_ID, Settings.STEER_RR_ID, Settings.ENCODER_RR_ID);
    swerveModule moduleRL = new swerveModule(Settings.DRIVE_RL_ID, Settings.STEER_RL_ID, Settings.ENCODER_RL_ID);
    swerveModule moduleFL = new swerveModule(Settings.DRIVE_FL_ID, Settings.STEER_FL_ID, Settings.ENCODER_FL_ID);

    public double L = 0.5334; // front to back measurement in meters\\
    public double W = 0.5334; // left to right measurement in meters\\
    public double R = Math.sqrt(L * L + W * W);

    boolean gyroToggle = true;



    ///Gyro Shit\\\

    public double getGyro(){
        return gyro.getAngle();
    }
    public void resetGyro(){
        gyro.reset();
    }
    public void setGyro(double offset){
        gyro.setAngleAdjustment(offset - getGyro() + gyro.getAngleAdjustment());
    }

    public double getGyroSwerve(){
        if(gyroToggle) return -gyro.getAngle();
        else return 0;
    }

    public void toggleGyro(){
        gyroToggle = !gyroToggle;
    }

    public void zeroHeading(){
        gyro.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }



    /// Auto Shit \\\

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(Settings.kTrackwidthMeters  /2, -Settings.kTrackwidthMeters /2),
        new Translation2d(Settings.kTrackwidthMeters /2, Settings.kTrackwidthMeters /2),        
        new Translation2d(-Settings.kTrackwidthMeters /2, -Settings.kTrackwidthMeters /2),
        new Translation2d(-Settings.kTrackwidthMeters /2, Settings.kTrackwidthMeters /2));

    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(swerveDriveKinematics, new Rotation2d());

    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        odometer.resetPosition(pose, getRotation2d());
    }

    //won't let me override??
    public void periodic(){
        odometer.update(getRotation2d(), moduleFL.getState(), moduleFR.getState(), moduleRL.getState(), moduleRR.getState());
    }
    
    public void stopModules(){
        moduleFL.stop();
        moduleFR.stop();
        moduleRL.stop();
        moduleRR.stop();
    }

    public Command getAutonomousCommand(){
        // 1. create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Settings.kMaxSpeedMetersPerSecond, 
            Settings.kMaxAccelerationMetersPerSecondSquared).setKinematics(swerveDriveKinematics);

        // 2. generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(1, 0), 
                new Translation2d(1, -1)), 
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(Settings.kPXController, 0, 0);
        PIDController yController = new PIDController(Settings.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Settings.kPThetaController, 0, 0, Settings.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory            
        SwerveControllerCommand command = new SwerveControllerCommand(
            trajectory, 
            this::getPose,
            swerveDriveKinematics, 
            xController, 
            yController, 
            thetaController, 
            this::setModuleStates, 
            this);

        // 5. add new init and wrap-up, and return everything
        return new SequentialCommandGroup(
            new InstantCommand(() ->  resetOdometry(trajectory.getInitialPose()),
            swerveControllerCommand,
            new InstantCommand(() -> stopModules()));
            //idk why there is an error here?
    }
    /// Swerve Drive \\\

    public void swerve(double forward, double strafe, double rotation) {
        if (forward != 0 || strafe != 0 || rotation != 0) {
            // Calculates position on field\\

            double newFWD = forward * Math.cos(getGyroSwerve()* Math.PI /180) + strafe * Math.sin(getGyroSwerve()* Math.PI /180);
            double newSTR = strafe * Math.cos(getGyroSwerve()* Math.PI /180) - forward * Math.sin(getGyroSwerve()* Math.PI /180);
            // double newFWD = forward * Math.cos(0* Math.PI /180) + strafe * Math.sin(0* Math.PI /180);
            // double newSTR = strafe * Math.cos(0* Math.PI /180) - forward * Math.sin(0* Math.PI /180);
           
            double A = newSTR - rotation * (L / R);
            double B = newSTR + rotation * (L / R);
            double C = newFWD - rotation * (W / R);
            double D = newFWD + rotation * (W / R);

            // Calculate wheel speeds\\
            double wheelSpeedFR = Math.sqrt(Math.pow(B, 2) + Math.pow(C, 2));
            double wheelSpeedFL = Math.sqrt(Math.pow(B, 2) + Math.pow(D, 2));
            double wheelSpeedRR = Math.sqrt(Math.pow(A, 2) + Math.pow(C, 2));
            double wheelSpeedRL = Math.sqrt(Math.pow(A, 2) + Math.pow(D, 2));

            double wheelSpeedMax1 = Math.max(wheelSpeedFR, wheelSpeedFL);
            double wheelSpeedMax2 = Math.max(wheelSpeedRR, wheelSpeedRL);
            double wheelSpeedMax = Math.max(wheelSpeedMax1, wheelSpeedMax2);

            // Calculate wheel angles\\
            double wheelAngleFR = Math.atan2(B, C) * 180/ Math.PI;
            double wheelAngleFL = Math.atan2(B, D) * 180/ Math.PI;
            double wheelAngleRR = Math.atan2(A, C) * 180/ Math.PI;
            double wheelAngleRL = Math.atan2(A, D) * 180/ Math.PI;

            if (wheelSpeedMax > 1) {
                wheelSpeedFR = wheelSpeedFR / wheelSpeedMax;
                wheelSpeedFL = wheelSpeedFL / wheelSpeedMax;
                wheelSpeedRR = wheelSpeedRR / wheelSpeedMax;
                wheelSpeedRL = wheelSpeedRL / wheelSpeedMax;
            }

            moduleFR.setSwervePod(wheelSpeedFR, wheelAngleFR);
            moduleRR.setSwervePod(wheelSpeedRR, wheelAngleRR);
            moduleRL.setSwervePod(wheelSpeedRL, wheelAngleRL);
            moduleFL.setSwervePod(wheelSpeedFL, wheelAngleFL);
        }
        else{
            moduleFR.setSwervePod(0, 0);
            moduleRR.setSwervePod(0, 0);
            moduleRL.setSwervePod(0, 0);
            moduleFL.setSwervePod(0, 0);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 5);
        moduleFL.setDesiredState(desiredStates[0]);
        moduleFR.setDesiredState(desiredStates[1]);
        moduleRL.setDesiredState(desiredStates[2]);
        moduleRR.setDesiredState(desiredStates[3]);
    }


    
    public class swerveModule{
        TalonFX driveMotor;
        TalonFX steerMotor;
        CANCoder moduleEncoder;

        public swerveModule(int driveID, int steerID, int encoderID){
            driveMotor = new TalonFX(driveID, "Vroom");
            steerMotor = new TalonFX(steerID, "Vroom");
            moduleEncoder = new CANCoder(encoderID, "Vroom");

            driveMotor.config_kP(0, .75);
            driveMotor.config_kI(0, 0);
            driveMotor.config_kD(0, .2);
            driveMotor.configPeakOutputForward(1);
            driveMotor.configPeakOutputReverse(-1);
            driveMotor.setInverted(false);
    
            steerMotor.config_kP(0, .75);
            steerMotor.config_kI(0, 0);
            steerMotor.config_kD(0, .2);
            steerMotor.configPeakOutputForward(1);
            steerMotor.configPeakOutputReverse(-1);
            steerMotor.setSensorPhase(true);
            steerMotor.configRemoteFeedbackFilter(moduleEncoder.getDeviceID(), RemoteSensorSource.CANCoder, 0);
            steerMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
        }
        
        public void setSwervePod(double speed, double angle) {
            double currentPos = -steerMotor.getSelectedSensorPosition();
            currentPos /= wrap(4096/360);
            double absolutePos = moduleEncoder.getAbsolutePosition();
            double errorPosition = wrap(absolutePos - angle);
    
            if (Math.abs(errorPosition) > 90) {
                errorPosition = errorPosition - 180 * MathUtil.sign(errorPosition);
                speed *= -1;
            }
 
            double rotationVal = currentPos - errorPosition;
            rotationVal *= wrap(4096/360);


            driveMotor.set(ControlMode.PercentOutput, speed);
            steerMotor.set(ControlMode.Position, -rotationVal);
        }

        public double getDrivePosition() {
            return driveMotor.getSelectedSensorPosition();
        }
    
        public double getTurningPosition() {
            return moduleEncoder.getAbsolutePosition();
        }

        public double getDriveVelocity() {
            return driveMotor.getSelectedSensorVelocity() / 5000;
        }
    
        public double getTurningVelocity() {
            return steerMotor.getSelectedSensorVelocity();
        }

        public double wrap(double angle){
            double newAngle = angle % 360;
            if(newAngle > 180) newAngle -=360;
            else if(newAngle < -180) newAngle += 360;

            return newAngle;
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(getDriveVelocity(), new Rotation2d((getTurningPosition() * Math.PI)/180));
        }
    
        public void setDesiredState(SwerveModuleState state) {
            // if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            //     stop();
            //     return;
            // }

            setSwervePod(state.speedMetersPerSecond / 5, state.angle.getDegrees());
            
            // System.out.println("speedMeters: " + state.speedMetersPerSecond);
            // System.out.println("get Radians: " + state.angle.getDegrees());
        }

    
        public void stop() {
            driveMotor.set(ControlMode.PercentOutput, 0);
            steerMotor.set(ControlMode.PercentOutput, 0);
        }

        
    }
}
