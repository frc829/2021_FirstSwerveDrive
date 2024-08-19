package frc.robot;

import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;


public class SwerveModule {
    
    SwerveModuleAzimuth swerveAzimuth;
    SwerveModuleDrive swerveDrive;
    private double speedToSet;
    private double angleToSet;
    private double currentAngle;
    private double currentSpeed;
    
    public SwerveModule(int i){
        this.swerveAzimuth = new SwerveModuleAzimuth(i);
        this.swerveDrive = new SwerveModuleDrive(i);

        
    }

    public void SetModule(SwerveModuleState moduleState){
        this.speedToSet = moduleState.speedMetersPerSecond;
        this.angleToSet = moduleState.angle.getDegrees();
        this.currentAngle = swerveAzimuth.getCurrentAngle();
        this.currentSpeed = swerveDrive.getSpeed();

        swerveAzimuth.SetAzimuth(this.angleToSet);
        swerveDrive.SetDrive(this.speedToSet);


             
    
    }


    public double getCurrentSpeed()
    {
        return this.currentSpeed;
    }

    public double getCurrentAngle()
    {
        return this.currentAngle;

    }

    public double getSpeedToSet()
    {
        return this.speedToSet;
    }

    public double getAngleToSet()
    {
        return this.angleToSet;

    }
}