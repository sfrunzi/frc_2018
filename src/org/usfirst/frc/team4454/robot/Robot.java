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

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

@SuppressWarnings("deprecation")
public class Robot extends IterativeRobot {
	UsbCamera intakeCamera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");
	CvSource intakeOutputStream;
	VisionThread intakeVisionThread;
	
	int contourNumber = 0;
	
	int pipelineRunning = 0;
	
	int exposureValue = 10;
	boolean exposureChanged = false;
	
	boolean foundTarget = false;
	
	// Vision Parameters
	double hueMin = 50;
	double hueMax = 122;
	double satMin = 105;
	double satMax = 255;
	double valMin = 50;
	double valMax = 255;
	
	double F = 0.027;
	double P = 0.05;
	double I = 0.0;
	double D = 1.0;
	
	//Test Variables
	String switchSide = "left";
	int robotPosition = 1;
	
	String autonMode;
	String autonSwitchPlace = "forward";
	double autonCrossDistance;
	
	double autonDistance;
	
	double inchPerSecond = 3.0;
	
	double targetDistance;
	
	double aspectRatio;
	
	boolean firstTimeAuton = true;
	boolean autonTurnDone = false;
	
	double autonCrossTime;
	double autonCrossWait = 3000;
	
	double autonPlaceTime;
	double autonPlaceWait = 3000;
	
	double autonSpeed;
	
	AHRS ahrs;
	
	TalonSRX frontLeft = new TalonSRX(1);
	TalonSRX middleLeft = new TalonSRX(2);
    TalonSRX rearLeft = new TalonSRX(3);

    TalonSRX frontRight = new TalonSRX(4);
    TalonSRX middleRight = new TalonSRX(5);
    TalonSRX rearRight = new TalonSRX(6);
    
    TalonSRX intakeLeft;
    TalonSRX intakeRight;
    TalonSRX rampLeft;
    TalonSRX rampRight;
    TalonSRX climberTop;
    TalonSRX climberBottom;
    
    Ultrasonic leftUltrasonic = new Ultrasonic(1,1);
    Ultrasonic rightUltrasonic = new Ultrasonic(2,2);
    
    Encoder encLeft;
	Encoder encRight;
	
	NetworkTable table;
	
	int countR;
	double rawDistanceR;
	double distanceR;
	double rateR;
	boolean directionR;
	boolean stoppedR;

	int countL;
	double rawDistanceL;
	double distanceL;
	double rateL;
	boolean directionL;
	boolean stoppedL;
    
    String gameData;
    
    int controllerMode = 1; // 1 is normal, 2 is experimental Attack 3 mode, and 3 is Joystick normal
    
	private Joystick leftStick;
	private Joystick rightStick;
	Joystick operatorStick;
	
	public boolean nearZero(double in) {
		if (in >= -0.01 && in <= 0.01) {
			return true;
		} else {
			return false;
		}
	}
	
	public void updateEncoders() {
		countR = encRight.get();
		rawDistanceR = encRight.getRaw();
		distanceR = encRight.getDistance();
		rateR = encRight.getRate();
		directionR = encRight.getDirection();
		stoppedR = encRight.getStopped();

		countL = encLeft.get();
		rawDistanceL = encLeft.getRaw();
		distanceL = encLeft.getDistance();
		rateL = encLeft.getRate();
		directionL = encLeft.getDirection();
		stoppedL = encLeft.getStopped();
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
	
	public double getLeftUltrasonic() {
		return leftUltrasonic.getRangeInches();
	}
	
	public double getRightUltrasonic() {
		return rightUltrasonic.getRangeInches();
	}
	
	public void setDriveMotors(double l, double r) {
		frontRight.set(ControlMode.PercentOutput, r);
		middleRight.set(ControlMode.PercentOutput, r);
		rearRight.set(ControlMode.PercentOutput, r);

		frontLeft.set(ControlMode.PercentOutput, l);
		middleRight.set(ControlMode.PercentOutput, l);
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
	
	void resetDistanceAndYaw () {
		ahrs.zeroYaw();
		//encLeft.reset();
		//encRight.reset();
	}
	

	@Override
	public void robotInit() {
		table = NetworkTable.getTable("SmartDashboard");
		
		frontLeft.setInverted(true);
		middleLeft.setInverted(true);
	    rearLeft.setInverted(true);
	    
		frontRight.setInverted(true);
		middleRight.setInverted(true);
	    rearRight.setInverted(true);
		
		leftUltrasonic.setAutomaticMode(true);
		rightUltrasonic.setAutomaticMode(true);
		
		SmartDashboard.putNumber("filterContoursMinArea", 40.0);
		SmartDashboard.putNumber("filterContoursMinPerimeter", 0);
		SmartDashboard.putNumber("filterContoursMaxWidth", 1000);
		SmartDashboard.putNumber("filterContoursMinWidth", 10);
		SmartDashboard.putNumber("filterContoursMaxHeight", 1000);
		SmartDashboard.putNumber("filterContoursMinHeight", 10);
		SmartDashboard.putNumber("filterContoursMaxVertices", 1000000);
		SmartDashboard.putNumber("filterContoursMinVertices", 0);
		SmartDashboard.putNumber("filterContoursMinRatio", 0.3);
		SmartDashboard.putNumber("filterContoursMaxRatio", 0.5);
		SmartDashboard.putNumber("Auton Cross Distance", 119.0);
		SmartDashboard.putNumber("Auton Speed", 0.65);
		SmartDashboard.putNumber("Inch Per Second", 38.0);
		
		SmartDashboard.putString("Auton Mode","cross");
		SmartDashboard.putString("hsvThresholdHue","[0.0, 255.0]");
		SmartDashboard.putString("hsvThresholdSaturation", "[15.0, 255.0]");
		SmartDashboard.putString("hsvThresholdValue", "[0.0, 255.0]");
		SmartDashboard.putString("filterContoursSolidity", "[0, 100]");
		
		intakeLeft = new TalonSRX(4);
	    intakeRight  = new TalonSRX(5);
	    rampLeft = new TalonSRX(6);
	    rampRight = new TalonSRX(7);
	    climberTop = new TalonSRX(8);
	    climberBottom = new TalonSRX(9);
		
		intakeCamera.setResolution(320, 240);
		//intakeCamera.setFPS(5);
		
		intakeOutputStream = CameraServer.getInstance().putVideo("intakeOverlay", 320, 240);
		//intakeOutputStream.setFPS(5);
		
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		operatorStick = new Joystick(2);
		
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
		 
		intakeVisionThread = new VisionThread(intakeCamera, new OurVisionPipeline(),
				pipeline->{
					double filterContoursMinArea = SmartDashboard.getNumber("filterContoursMinArea", 40.0);
					double filterContoursMinPerimeter = SmartDashboard.getNumber("filterContoursMinPerimeter", 0);
					double filterContoursMaxWidth = SmartDashboard.getNumber("filterContoursMaxWidth", 1000);
					double filterContoursMinWidth = SmartDashboard.getNumber("filterContoursMinWidth", 10);
					double filterContoursMaxHeight = SmartDashboard.getNumber("filterContoursMaxHeight", 1000);
					double filterContoursMinHeight = SmartDashboard.getNumber("filterContoursMinHeight", 10);
					double filterContoursMaxVertices = SmartDashboard.getNumber("filterContoursMaxVertices", 1000000);
					double filterContoursMinVertices = SmartDashboard.getNumber("filterContoursMinVertices", 0);
					double filterContoursMinRatio = SmartDashboard.getNumber("filterContoursMinRatio", 0.3);
					double filterContoursMaxRatio = SmartDashboard.getNumber("filterContoursMaxRatio", 0.5);
					
					String hsvThresholdHue = SmartDashboard.getString("hsvThresholdHue","[0.0, 255.0]");
					String hsvThresholdSaturation = SmartDashboard.getString("hsvThresholdSaturation", "[15.0, 255.0]");
					String hsvThresholdValue = SmartDashboard.getString("hsvThresholdValue", "[0.0, 255.0]");
					String filterContoursSolidity =SmartDashboard.getString("filterContoursSolidity", "[0, 100]");
					
					pipeline.setVariable("filterContoursMinArea", filterContoursMinArea);
					pipeline.setVariable("filterContoursMinPerimeter", filterContoursMinPerimeter);
					pipeline.setVariable("filterContoursMinWidth", filterContoursMinWidth);
					pipeline.setVariable("filterContoursMaxWidth", filterContoursMaxWidth);
					pipeline.setVariable("filterContoursMinHeight", filterContoursMinHeight);
					pipeline.setVariable("filterContoursMaxHeight", filterContoursMaxHeight);
					pipeline.setVariable("filterContoursMaxVertices", filterContoursMaxVertices);
					pipeline.setVariable("filterContoursMinVertices", filterContoursMinVertices);
					pipeline.setVariable("filterContoursMinRatio", filterContoursMinRatio);
					pipeline.setVariable("filterContoursMaxRatio", filterContoursMaxRatio);
					
					pipeline.setArray("hsvThresholdHue", hsvThresholdHue);
					pipeline.setArray("hsvThresholdSaturation", hsvThresholdSaturation);
					pipeline.setArray("hsvThresholdValue", hsvThresholdValue);
					pipeline.setArray("filterContoursSolidity", filterContoursSolidity);
					
					SmartDashboard.putNumber("Target Angel", pipeline.targetAngle);
				
					intakeOutputStream.putFrame(pipeline.overlayOutput());

					if (pipeline.foundTarget) {
						targetDistance = pipeline.targetDistance;
					} else {
						targetDistance = -1.0;
					}
					
					aspectRatio = pipeline.aspectRatioOut;
					
					foundTarget = pipeline.foundTarget;
					contourNumber = pipeline.contourNumber;
					pipelineRunning = pipeline.pipelineRunning;
					
					if (exposureChanged) {
						intakeCamera.setExposureManual(exposureValue);
						exposureChanged = false;						
					}

					/*pipeline.hsvThresholdHue[0] = hueMin;
					pipeline.hsvThresholdHue[1] = hueMax;

					pipeline.hsvThresholdSaturation[0] = satMin;
					pipeline.hsvThresholdSaturation[1] = satMax;

					pipeline.hsvThresholdValue[0] = valMin;
					pipeline.hsvThresholdValue[1] = valMax;*/
				});
		
		intakeVisionThread.start();
		
		encLeft = new Encoder(8, 9, false, CounterBase.EncodingType.k4X);

		encLeft.setMaxPeriod(1);
		encLeft.setMinRate(0.1);
		encLeft.setDistancePerPulse(1.0/((4.0 * 25.4 / .001) * Math.PI / 360.0));
		encLeft.setReverseDirection(false);
		encLeft.setSamplesToAverage(7);

		encRight = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);
		
		encRight.setMaxPeriod(1);
		encRight.setMinRate(0.1);
		encRight.setDistancePerPulse(1.0/((4.0 * 25.4 / .001) * Math.PI / 360.0));
		encRight.setReverseDirection(false);
		encRight.setSamplesToAverage(7);
	}
	
	public void autonCross() {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= ((autonCrossDistance / inchPerSecond) * 1000) + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			adaptiveDrive(1.0 * autonSpeed, 1.0 * autonSpeed);
		} else {
			adaptiveDrive(0, 0);
		}
	}
	
	public void autonTurn(double turnAngle, double turnPower) {
		double temp = Math.signum(turnAngle) * turnPower;
		setDriveMotors(temp, -temp);
		if ((turnAngle == 0.0) || (Math.abs(ahrs.getAngle()) > Math.abs(turnAngle))) {
			autonTurnDone = true;
			setDriveMotors(0.0, 0.0);
			resetDistanceAndYaw();
		}
	}
	
	public void autonPlace(double turnAngle, double turnPower) {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
		}
				
		if ((System.currentTimeMillis() - autonCrossTime) <= ((autonCrossDistance / inchPerSecond) * 1000) + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			autonTurnDone = false;
			adaptiveDrive(1.0 * autonSpeed, 1.0 * autonSpeed);
		} else {
			if (autonTurnDone == false) {
				autonTurn(turnAngle, turnPower);
			} else {
				
			}
		}
	}
	
	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L') {
				switchSide = "left";
			} else {
				switchSide = "right";
			}
		} else {
			switchSide = null;
		}
		
		autonCrossDistance = SmartDashboard.getNumber("Auton Cross Distance", 119.0);
		autonSpeed = SmartDashboard.getNumber("Auton Speed", 0.65);
		inchPerSecond = SmartDashboard.getNumber("Inch Per Second",38.0);
		autonMode = SmartDashboard.getString("Auton Mode","cross");
		System.out.println(autonMode.equals("cross"));
		
		firstTimeAuton = true;
		ahrs.zeroYaw();
		encLeft.reset();
		encRight.reset();
	}
	
	@Override
	public void autonomousPeriodic() {
		updateEncoders();
		
		if (autonMode.equals("cross")) {
			autonCross();
		} else if (autonMode.equals("place")) {
			autonPlace(40.0, 0.35); // We need to find the correct turning angle and power. Also change based on the options given.
		} else { // Invalid Mode
			System.out.println("Invalid Mode");
			adaptiveDrive(0, 0);
		}
	}
	
	@Override
	public void teleopInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gameData.length() > 0) {
			if (gameData.charAt(0) == 'L') {
				switchSide = "left";
			} else {
				switchSide = "right";
			}
		} else {
			switchSide = null;
		}
		
		SmartDashboard.putNumber("filterContoursMinArea", 200.0);
		ahrs.zeroYaw();
	}

	@Override
	public void teleopPeriodic() {
		updateEncoders();
		
		double scale = getDrivePowerScale();
		
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
		
		SmartDashboard.putNumber("Target Distance:", targetDistance);
		SmartDashboard.putNumber("Aspect Ratio:", aspectRatio);
		SmartDashboard.putNumber("Countour Number:", contourNumber);
		SmartDashboard.putNumber("Running:", pipelineRunning);
		SmartDashboard.putBoolean("Target Found:", foundTarget);
	
	}
}
