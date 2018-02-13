/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4454.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends IterativeRobot {
	
    int controllerMode = 1; // 1 is normal, 2 is experimental Attack 3 mode, and 3 is Joystick normal

	AHRS ahrs;
	
	TalonSRX frontLeft = new TalonSRX(1);
	TalonSRX middleLeft = new TalonSRX(2);
    TalonSRX rearLeft = new TalonSRX(3);

    TalonSRX frontRight = new TalonSRX(4);
    TalonSRX middleRight = new TalonSRX(5);
    TalonSRX rearRight = new TalonSRX(6);

    DoubleSolenoid driveTrainShift = new DoubleSolenoid(0, 0, 1);
    Compressor compressor = new Compressor(0);
    
	private Joystick leftStick;
	private Joystick rightStick;
	private Joystick operatorStick;

	@Override
	public void robotInit() {
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		operatorStick = new Joystick(2);
		
		// Top motors go through an idler
		// Right side is additionally inverted
		rearLeft.setInverted(true);
		middleRight.setInverted(true);
		rearRight.setInverted(true);
		
		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
			ahrs = new AHRS(SPI.Port.kMXP);
			ahrs.zeroYaw();
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

	}

	@Override
	public void teleopPeriodic() {
		double scale = getDrivePowerScale();
		
		if (leftStick.getTrigger()) {
			driveTrainShift.set(DoubleSolenoid.Value.kForward);
		}
		else if (rightStick.getTrigger()) {
			driveTrainShift.set(DoubleSolenoid.Value.kReverse);
		}
		else {
			driveTrainShift.set(DoubleSolenoid.Value.kOff);
		}
		
		if (controllerMode == 1) {
			adaptiveDrive(scale * (-1 * leftStick.getY()), scale * (-1 * rightStick.getY())); //  Negated
		} else if (controllerMode == 2) {
			
			if (!nearZero(leftStick.getY()) && nearZero(rightStick.getX())) {
				adaptiveDrive(scale * leftStick.getY(), scale * leftStick.getY());
			} else if (nearZero(leftStick.getY()) && !nearZero(rightStick.getX())) {
				if (rightStick.getX() < 0) {
					adaptiveDrive((scale * rightStick.getX()) * -1, 0.0);
				} else {
					adaptiveDrive(0.0, scale * rightStick.getX());
				}
			} else if (nearZero(leftStick.getY()) && !nearZero(rightStick.getX())) {
				if (rightStick.getX() < 0) {
					adaptiveDrive((scale * rightStick.getX()) * -1, rightStick.getX());
				} else {
					adaptiveDrive((scale * rightStick.getX()) * -1, scale * rightStick.getX());
				}
			} else {
				adaptiveDrive(0.0, 0.0);
			}
			
		} else if (controllerMode == 3) {
			adaptiveDrive(scale * (-1 * operatorStick.getRawAxis(1)), scale * (-1 * operatorStick.getRawAxis(5)));
		}
	}
	
	public boolean nearZero(double in) {
		if (in >= -0.01 && in <= 0.01) {
			return true;
		} else {
			return false;
		}
	}

	public void setDriveMotors(double l, double r) {
		frontRight.set(ControlMode.PercentOutput, r);
		middleRight.set(ControlMode.PercentOutput, r);
		rearRight.set(ControlMode.PercentOutput, r);

		frontLeft.set(ControlMode.PercentOutput, l);
		middleLeft.set(ControlMode.PercentOutput, l);
		rearLeft.set(ControlMode.PercentOutput, l);
	}
	
	public void adaptiveDrive(double l, double r) {

		// alpha is a parameter between 0 and 1
		final double alpha = 0.5;
		double c = 0.5 * (l+r);
		double d = 0.5 * (l-r);
		double scale = (1 - (alpha * c * c));
		d *= scale;

		// GYRO CORRECTION -- high if d is close to zero, low otherwise
		double gRate = ahrs.getRate();
		final double CORR_COEFF = 0.5;
		double corr = 0.0;
		if (Math.abs(d) < 0.05)
			corr = (gRate * Math.abs(c) * CORR_COEFF * (1-Math.abs(d)));
		d -= corr;

		double l_out = c + d;
		double r_out = c - d;

		setDriveMotors(l_out, r_out);
	}

	
	public double getDrivePowerScale() {
		double scale = 0.65;

		if (controllerMode == 1 || controllerMode == 2) {
			if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
				scale = 0.85;
			}
			
			if (leftStick.getTrigger() && rightStick.getTrigger()) {
				scale = 1;
			}
		} else {
			if (operatorStick.getRawButton(5) || operatorStick.getRawButton(6)) {
				scale = 0.85;
			}
			
			if (operatorStick.getRawButton(5) && operatorStick.getRawButton(6)) {
				scale = 1;
			}
		}
		
		return scale;
	}

}
