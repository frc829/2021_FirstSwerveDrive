package frc.robot;


import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.kauailabs.navx.frc.AHRS;

import frc.util.LogitechF310;


public class SwerveDrive{
    
    LogitechF310 driver;    
    AHRS gyro;
    Translation2d m_frontLeftLocation;
    Translation2d m_frontRightLocation;
    Translation2d m_backLeftLocation;
    Translation2d m_backRightLocation;
    SwerveDriveKinematics kinematics;
    SwerveModule[] swerveModules;
    double vx;
    double vy;
    double rotationSpeed;
    ChassisSpeeds chassisSpeeds;
    double JoyStickAngle;
    double JoyStickSpeed;
    SwerveModuleState[] moduleStates;
    SwerveModuleState[] moduleStatesFixedSpeed;
    SwerveModuleState[] optimizedModuleStates;
    double[] currentModuleAngles;

    NetworkTableInstance instance;
    NetworkTable stickSpeed;
    NetworkTable stickAngle;
    NetworkTable optimizedFrontLeftAngle;
    NetworkTable optimizedFrontLeftSpeed;
    NetworkTable frontLeftAngle;
    NetworkTable frontLeftSpeed;
    NetworkTable frontLeftTarget;
    
    
    boolean reseting = false;
    
    public SwerveDrive(LogitechF310 driver, AHRS gyro){
        
        // Locations for the swerve drive modules relative to the robot center.
        this.driver = driver;
        this.currentModuleAngles = new double[4];
        this.m_frontLeftLocation = new Translation2d(0.5, 0.5);
        this.m_frontRightLocation = new Translation2d(0.5, -0.5);
        this.m_backLeftLocation = new Translation2d(-0.5, 0.5);
        this.m_backRightLocation = new Translation2d(-0.5, -0.5);
        this.kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );
        this.swerveModules = new SwerveModule[4]; 
        for(int i = 0; i < 4; i++){
            this.swerveModules[i] = new SwerveModule(i);
        } 

        this.instance = NetworkTableInstance.getDefault();
        this.stickSpeed = instance.getTable("JoyStickSpeed");
        this.stickAngle = instance.getTable("JoyStickAngle");
        this.optimizedFrontLeftAngle = instance.getTable("FrontLeftAngleState");
        this.optimizedFrontLeftSpeed = instance.getTable("FrontLeftSpeedState");
        this.frontLeftAngle = instance.getTable("FrontLeftAngle");
        this.frontLeftSpeed = instance.getTable("FrontLeftSpeed");
        this.frontLeftTarget = instance.getTable("frontLeftTarget");

        this.moduleStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            this.moduleStates[i] = new SwerveModuleState();
        }
        
   
    }

    
    public void teleopUpdate(){

        currentModuleAngles[0] = this.swerveModules[0].getCurrentAngle();
        currentModuleAngles[1] = this.swerveModules[1].getCurrentAngle();
        currentModuleAngles[2] = this.swerveModules[2].getCurrentAngle();
        currentModuleAngles[3] = this.swerveModules[3].getCurrentAngle();

        GetChassisSpeedFromJoyStick();
        GetSwerveModuleStates();
        FixModuleStateSpeeds();
        OptimizeModuleStates();
        SetDrive();
        Report();
        

    }  



    public void GetChassisSpeedFromJoyStick() {        
         
        this.rotationSpeed = -this.driver.getRotation();
        
        if(this.driver.getMagnitude() > 1){
            this.JoyStickSpeed = 1;
            this.JoyStickAngle = -this.driver.getDirectionDegrees();
        }
        else if (this.driver.getMagnitude() >= 0.2)
        {
            this.JoyStickSpeed = this.driver.getMagnitude();
            this.JoyStickAngle = -this.driver.getDirectionDegrees();
        }
        else{
            this.JoyStickSpeed = 0; 
            this.JoyStickAngle = 0;     
        }

        //System.out.println("GetDegrees" + this.driver.getDirectionDegrees());
        //System.out.println("JoystickAngle" + this.JoyStickAngle);
        if(Math.abs(rotationSpeed) < 0.2){rotationSpeed = 0;}
    
        this.vx = this.JoyStickSpeed * Math.sin(Math.toRadians(this.JoyStickAngle));
        this.vy = this.JoyStickSpeed * Math.cos(Math.toRadians(this.JoyStickAngle));        


        this.chassisSpeeds = new ChassisSpeeds(this.vx,this.vy,this.rotationSpeed);

    
       
    }


    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    // ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);  
    public void GetSwerveModuleStates() {      
        
        
        if(this.vx == 0 && this.vy == 0 && this.rotationSpeed == 0){
            for(int i = 0; i < 4; i++){
                moduleStates[i].speedMetersPerSecond = 0;
            }
        }
        else{
            moduleStates =  this.kinematics.toSwerveModuleStates(this.chassisSpeeds);
        }
    }  

    private void FixModuleStateSpeeds() {
        this.moduleStatesFixedSpeed = new SwerveModuleState[4];

        //Set our max speed to a specific value;
        double ourMaxSpeed = 0.2;
        
        for(int i = 0; i < 4; i++){
            this.moduleStatesFixedSpeed[i] = new SwerveModuleState();
            this.moduleStatesFixedSpeed[i].angle = this.moduleStates[i].angle;
            this.moduleStatesFixedSpeed[i].speedMetersPerSecond = this.moduleStates[i].speedMetersPerSecond * ourMaxSpeed;
        }
        
    }

    private void OptimizeModuleStates() {
        optimizedModuleStates = new SwerveModuleState[4];

        for(int i = 0; i < 4; i++){
            this.optimizedModuleStates[i] = new SwerveModuleState();
            this.optimizedModuleStates[i].speedMetersPerSecond = this.moduleStatesFixedSpeed[i].speedMetersPerSecond;
            this.optimizedModuleStates[i].angle = this.moduleStatesFixedSpeed[i].angle;
            int currentAngleQuad = FindQuadrant(this.currentModuleAngles[i]);
            int targetAngleQuad = FindQuadrant(this.moduleStatesFixedSpeed[i].angle.getDegrees());

            AdjustModuleAngles(currentAngleQuad, targetAngleQuad, i);
        }
    }      



    private int FindQuadrant(double d) {
        if(d >= -90 && d < 0){
            return 1;
        }
        else if(d >= 0 && d < 90){
            return 2;
        }
        else if(d >= 90 && d < 180){
            return 3;
        }
        else{
            return 4;
        }
    }

    private void AdjustModuleAngles(int currentAngleQuad, int targetAngleQuad, int i) {
        double targetAngle = this.moduleStatesFixedSpeed[i].angle.getDegrees();
        double currentAngle = this.currentModuleAngles[i];

        switch(currentAngleQuad){
            case 1:
                switch (targetAngleQuad){
                    case 2:Check90(targetAngle, currentAngle, i);break;
                    case 3:SuppRev(targetAngle, i);break;
                    case 4:Check90(targetAngle, currentAngle, i);break;
                    default:break;
                };break;
            
            case 2:
            switch (targetAngleQuad){
                case 1:Check90(targetAngle, currentAngle, i);break;
                case 3:Check90(targetAngle, currentAngle, i);break;
                case 4:SuppRev(targetAngle, i);break;
                default:break;
            };break;
            
            case 3:
            switch (targetAngleQuad){
                case 1:SuppRev(targetAngle, i);break;
                case 2:Check90(targetAngle, currentAngle, i);break;
                case 4:Check90(targetAngle, currentAngle, i);break;
                default:break;
            };break;
            
            case 4:
            switch (targetAngleQuad){
                case 1:Check90(targetAngle, currentAngle, i);break;
                case 2:SuppRev(targetAngle, i);break;
                case 3:Check90(targetAngle, currentAngle, i);break;
                default:break;
            };break;
        }
    }



    private void Check90(double targetAngle, double currentAngle, int i) {

        if(currentAngle < -90 && targetAngle >= 90){
            currentAngle += 360;
        }
        else if(currentAngle >= 90 && targetAngle < -90){
            targetAngle += 360;
        }

        if(Math.abs(targetAngle - currentAngle) > 90){
            SuppRev(targetAngle, i);
        }       
    }

    private void SuppRev(double targetAngle, int i) {
        if(targetAngle > 0){
            targetAngle -= 180;
        }
        else if(targetAngle < 0){
            targetAngle += 180;
        }
        else{
            targetAngle = 180;
        }
        optimizedModuleStates[i].speedMetersPerSecond = -optimizedModuleStates[i].speedMetersPerSecond;
        optimizedModuleStates[i].angle = new Rotation2d(Math.toRadians(targetAngle));
    }

    private void SetDrive() {

        for(int i = 0; i < 4; i++){
            swerveModules[i].SetModule(this.optimizedModuleStates[i]);
        }
        
    }  

    private void Report() {
        this.stickAngle.getEntry("Value:").setDouble(this.JoyStickAngle);
        this.stickSpeed.getEntry("Value:").setDouble(this.JoyStickSpeed);

        this.optimizedFrontLeftAngle.getEntry("Value:").setDouble(this.moduleStatesFixedSpeed[0].angle.getDegrees());
        this.optimizedFrontLeftSpeed.getEntry("Value:").setDouble(this.moduleStatesFixedSpeed[0].speedMetersPerSecond);

        this.frontLeftAngle.getEntry("Value:").setDouble(this.swerveModules[0].swerveAzimuth.getCurrentAngle());
        this.frontLeftSpeed.getEntry("Value:").setDouble(this.swerveModules[0].swerveDrive.getSpeed());
        this.frontLeftTarget.getEntry("Value:").setDouble(this.swerveModules[0].swerveAzimuth.getTargetAngle());

    }
}
