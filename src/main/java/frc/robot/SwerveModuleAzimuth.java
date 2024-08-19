package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class SwerveModuleAzimuth {
	private CANSparkMax sparkMax;
	private CANCoder encoder;
	int slot = 0;
	private double azimuthSpeed;
	private double angleTolerance;
	private double targetAngle;
	private double angleWhenZeroed;
		
	

	public SwerveModuleAzimuth(int i) {
		this.sparkMax = new CANSparkMax(i + 14, MotorType.kBrushless);
		this.encoder = new CANCoder(i + 30);
		sparkMax.restoreFactoryDefaults();


		sparkMax.setSmartCurrentLimit(15);
    	sparkMax.setSecondaryCurrentLimit(30.0);
    	sparkMax.enableVoltageCompensation(12.0);
		sparkMax.setIdleMode(IdleMode.kCoast);

		this.angleTolerance = 0.1;
		this.azimuthSpeed = 0.2;

		System.out.println(i + " wheel set to " + encoder.getAbsolutePosition());		
		


		switch(i){
			case 0: this.angleWhenZeroed = 125.4; break;
			case 1: this.angleWhenZeroed = 325.4; break;
			case 2: this.angleWhenZeroed = 271.1; break;
			default: this.angleWhenZeroed = 27.0; break;
		}	
		
		this.encoder.setPosition(0);
		
	}


	public void SetAzimuth(double angle) {
		this.targetAngle = angle;
		
		if(encoder.getPosition() > 180){
			encoder.setPosition(encoder.getPosition() - 360);
			
		}
		else if(encoder.getPosition() < -180){
			encoder.setPosition(encoder.getPosition() + 360);
			
		}
		
		if(encoder.getPosition() >= 90 && encoder.getPosition() < 180){
			if(angle >= -180 && angle < -90){
				angle += 360;
			}
		}
		else if(encoder.getPosition() < -90 && encoder.getPosition() >= -180){
			if(angle >= 90 && angle < 180){
				angle -= 360;
			}
		}


		angleTolerance = .1;
		azimuthSpeed = 0.2 * Math.abs(angle - encoder.getPosition())/90;


		// if(encoder.getPosition() < angle - angleTolerance){
		// 	sparkMax.set(azimuthSpeed);
		// }
		// else if(encoder.getPosition() > angle + angleTolerance){
		// 	sparkMax.set(-azimuthSpeed);
		// }
		// else{
		// 	sparkMax.set(0);
		// }

		if(encoder.getPosition() > angle + angleTolerance ){
			sparkMax.set(azimuthSpeed);
		}
		else if(encoder.getPosition() < angle - angleTolerance ){
			sparkMax.set(-azimuthSpeed);
		}
		else {
			sparkMax.set(0);
		}
		
	}

	public double getCurrentAngle(){
		return this.encoder.getPosition();
	}

	public double getTargetAngle(){
		return this.targetAngle;
	}

}
