package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class SwerveModuleDrive {
	private TalonFX talonFX;
	private double currentSpeed;


	public SwerveModuleDrive(int i) {
		this.talonFX = new TalonFX(i + 10);
		TalonFXConfiguration talonFXConfig = new TalonFXConfiguration();
		talonFXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		talonFXConfig.supplyCurrLimit.enable = true;
		talonFXConfig.supplyCurrLimit.triggerThresholdCurrent = 60;
		talonFXConfig.supplyCurrLimit.triggerThresholdTime = 1.0;
		talonFXConfig.supplyCurrLimit.currentLimit = 80.0;
	
		talonFXConfig.statorCurrLimit.enable = true;
		talonFXConfig.statorCurrLimit.triggerThresholdCurrent = 60;
		talonFXConfig.statorCurrLimit.triggerThresholdTime = 1.0;
		talonFXConfig.statorCurrLimit.currentLimit = 80.0;
	
		talonFXConfig.slot0.kP = 2.0;
		talonFXConfig.slot0.kI = 0.0;
		talonFXConfig.slot0.kD = 30.0;
		talonFXConfig.slot0.kF = 0.0;
		talonFXConfig.slot0.integralZone = (int) 0.0;
		talonFXConfig.slot0.allowableClosedloopError = (int) 0.0;
	
		talonFXConfig.slot1.kP = 2.0;
		talonFXConfig.slot1.kI = 0.0;
		talonFXConfig.slot1.kD = 30.0;
		talonFXConfig.slot1.kF = 0.0;
		talonFXConfig.slot1.integralZone = (int) 0.0;
		talonFXConfig.slot1.allowableClosedloopError = (int) 0.0;
	
		talonFXConfig.slot2.kP = 2.0;
		talonFXConfig.slot2.kI = 0.0;
		talonFXConfig.slot2.kD = 30.0;
		talonFXConfig.slot2.kF = 0.0;
		talonFXConfig.slot2.integralZone = (int) 0.0;
		talonFXConfig.slot2.allowableClosedloopError = (int) 0.0;
	
		talonFXConfig.slot3.kP = 2.0;
		talonFXConfig.slot3.kI = 0.0;
		talonFXConfig.slot3.kD = 30.0;
		talonFXConfig.slot3.kF = 0.0;
		talonFXConfig.slot3.integralZone = (int) 0.0;
		talonFXConfig.slot3.allowableClosedloopError = (int) 0.0;
	
		talonFXConfig.motionAcceleration = (int) 20000.0;
		talonFXConfig.motionCruiseVelocity = (int) 5500.0;
		talonFXConfig.velocityMeasurementWindow = 64;
		talonFXConfig.voltageCompSaturation = 12.0;
	
		talonFX.configAllSettings(talonFXConfig);
		talonFX.enableVoltageCompensation(true);
		talonFX.setNeutralMode(NeutralMode.Brake);
	}

	public void SetDrive(double speed) {
		this.currentSpeed = speed;
		this.talonFX.set(ControlMode.PercentOutput, speed);		
	}    

	public double getSpeed()
	{
		return this.currentSpeed;
	}
}
