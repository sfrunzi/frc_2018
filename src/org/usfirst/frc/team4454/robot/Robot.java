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
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

// PWM ID 0 -- two front motors with compliant wheels
// PWM ID 1 -- two rear motors with four compliant wheel shafts
// PWM ID 2 -- two intake motors
// CAN ID 9 -- front intake bar
public class Robot extends IterativeRobot {
	UsbCamera intakeCamera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");
	CvSource intakeOutputStream;
	VisionThread intakeVisionThread;
	
	UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0");
	
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

	String autonSwitchPlace = "forward";
	double autonCrossDistance;
	
	double autonDistance;
	
	double inchPerSecond = 3.0;
	
	double targetDistance;
	
	double aspectRatio;
	
	boolean firstTimeAuton = true;
	boolean autonCrossDone = false;
	boolean autonTurnDone = false;
	boolean autonReverseDone = false;
	boolean autonForwardDone = false;
	
	double autonCrossTime;
	double autonCrossWait = 3000;
	
	double autonPlaceTime;
	double autonPlaceWait = 3000;
	
	double autonSpeed;
	
	String autonStep;
	
	AHRS ahrs;
	
	TalonSRX frontLeft = new TalonSRX(1);
	TalonSRX middleLeft = new TalonSRX(2);
    TalonSRX rearLeft = new TalonSRX(3);

    TalonSRX frontRight = new TalonSRX(4);
    TalonSRX middleRight = new TalonSRX(5);
    TalonSRX rearRight = new TalonSRX(6);
    
    Talon intake;
    Talon upperRamp;
    Talon lowerRamp;
    
    TalonSRX beatingStick;
    
    //TalonSRX climberTop;
    //TalonSRX climberBottom;
    
    //Ultrasonic leftUltrasonic = new Ultrasonic(1,1);
    //Ultrasonic rightUltrasonic = new Ultrasonic(2,2);
    
    DoubleSolenoid driveTrainShift = new DoubleSolenoid(0, 0, 1);
    DoubleSolenoid ptoShift = new DoubleSolenoid(0, 4, 5);
    Compressor compressor = new Compressor(0);
    
    DoubleSolenoid intakePiston = new DoubleSolenoid(0, 2, 3);
    
    boolean autonTurn1Done = false;
    
    Encoder encLeft;
	Encoder encRight;
	
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
    
    double[] lowerRampSpeeds = {0.65, 0.75};
    double[] upperRampSpeeds = {0.65};
    
    int lowerRampSpeed = 0;
    int upperRampSpeed = 0;
    
	private Joystick leftStick;
	private Joystick rightStick;
	Joystick operatorStick;
	
	SendableChooser<Integer> autonChoose;
	int autonMode;
	
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
		
		System.out.println("Left Encoder Count " + countL + " Encoder distance " + distanceL);
		System.out.println("Right Encoder Count " + countR + " Encoder distance " + distanceR);
	}
	
	public double getDrivePowerScale() {
		double scale = 1;
		
		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 0.85;
		}
		
		if (leftStick.getTrigger() && rightStick.getTrigger()) {
			scale = 1;
		}
		
		return scale;
	}
	
	/*public double getLeftUltrasonic() {
		return leftUltrasonic.getRangeInches();
	}
	
	public double getRightUltrasonic() {
		return rightUltrasonic.getRangeInches();
	}*/
	
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
	
	void resetDistanceAndYaw () {
		ahrs.zeroYaw();
		encLeft.reset();
		//encRight.reset();
	}
	

	@Override
	public void robotInit() {
	    rearLeft.setInverted(true);

		middleRight.setInverted(true);
	    rearRight.setInverted(true);
		
		//leftUltrasonic.setAutomaticMode(true);
		//rightUltrasonic.setAutomaticMode(true);
		
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

		autonChoose = new SendableChooser<Integer>();
		
		autonChoose.addDefault("cross", 1);
		autonChoose.addObject("place", 2);
		autonChoose.addObject("exchange", 3);
		
		SmartDashboard.putData("Auton Mode", autonChoose);
		
		intake = new Talon(0);
	    lowerRamp = new Talon(1);
	    upperRamp = new Talon(2);
	    beatingStick = new TalonSRX(9);
	    beatingStick.setInverted(true);
	    
	    //climberTop = new TalonSRX(8);
	    //climberBottom = new TalonSRX(9);
		
		//intakeCamera.setResolution(320, 240);
		//backCamera.setResolution(320, 240);
		
		//intakeOutputStream = CameraServer.getInstance().putVideo("intakeOverlay", 320, 240);
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
		 
		/*intakeVisionThread = new VisionThread(intakeCamera, new OurVisionPipeline(),
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
				});
		*/
		
		//intakeVisionThread.start();
		
		encLeft = new Encoder(2, 3, false, CounterBase.EncodingType.k4X);

		encLeft.setMaxPeriod(1);
		encLeft.setMinRate(0.1);
		encLeft.setDistancePerPulse(1.0/((4.0 * 25.4 / .001) * Math.PI / 360.0));
		encLeft.setReverseDirection(false);
		encLeft.setSamplesToAverage(7);

		encRight = new Encoder(0, 1, false, CounterBase.EncodingType.k4X);
		
		encRight.setMaxPeriod(1);
		encRight.setMinRate(0.1);
		encRight.setDistancePerPulse(1.0/((4.0 * 25.4 / .001) * Math.PI / 360.0));
		encRight.setReverseDirection(false);
		encRight.setSamplesToAverage(7);
	}
	
	// Auton helpers
	
	public void autonPlaceCube() {
		
	}
	
	public void autonPickupCube() {
		
	}
	
	public void autonForward(double distance) {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
			autonForwardDone = false;
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= ((distance / inchPerSecond) * 1000) + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			adaptiveDrive(1.0 * autonSpeed, 1.0 * autonSpeed);
		} else {
			adaptiveDrive(0, 0);
			autonForwardDone = true;
		}
	}
	
	public void autonReverse(double distance) {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
			autonReverseDone = false;
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= ((distance / inchPerSecond) * 1000) + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			adaptiveDrive(-1.0 * autonSpeed, -1.0 * autonSpeed);
		} else {
			adaptiveDrive(0, 0);
			autonReverseDone = true;
		}
	}
	
	public void autonForward_Time(double time) {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
			autonForwardDone = false;
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= time + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			adaptiveDrive(1.0 * autonSpeed, 1.0 * autonSpeed);
		} else {
			adaptiveDrive(0, 0);
			autonForwardDone = true;
		}
	}
	
	public void autonReverse_Time(double time) {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
			autonReverseDone = false;
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= time + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			adaptiveDrive(-1.0 * autonSpeed, -1.0 * autonSpeed);
		} else {
			adaptiveDrive(0, 0);
			autonReverseDone = true;
		}
	}
	
	public void autonTurn(double turnAngle) {
		double temp = Math.signum(turnAngle) * autonSpeed;
		setDriveMotors(temp, -temp);
		if ((turnAngle == 0.0) || (Math.abs(ahrs.getAngle()) > Math.abs(turnAngle))) {
			autonTurnDone = true;
			setDriveMotors(0.0, 0.0);
			resetDistanceAndYaw();
		}
	}
	
	// Auton Modes
	
	public void autonExchange() {
		if (!autonCrossDone) {
			autonCross();
		} else {
			if (!autonReverseDone) {
				firstTimeAuton = true;
				autonReverse(86.0);
			} else {
				
			}
		}
	}
	
	public void autonPlace(double turnAngle) {
		if (!autonCrossDone) {
			autonCross();
		} else {
			if (!autonTurn1Done) {
				firstTimeAuton = true;
				autonTurn(turnAngle);
				
				if (autonTurnDone) {
					autonTurn1Done = true;
				}
			} else {
				setDriveMotors(0.0, 0.0);
			}
		}
	}
	
	public void autonCross() {
		if (firstTimeAuton == true) {
			autonCrossDone = false;
		}
		
		if (!autonForwardDone) {
			autonForward(autonCrossDistance);
		} else {
			autonCrossDone = true;
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
		inchPerSecond = SmartDashboard.getNumber("Inch Per Second", 53.0);
		
		autonTurnDone = false;
		autonTurn1Done = false;
		firstTimeAuton = true;

		resetDistanceAndYaw();
		
		autonMode = autonChoose.getSelected();
		
		autonStep = "firstStraight";
	}
	
	@Override
	public void autonomousPeriodic() {
		driveTrainShift.set(DoubleSolenoid.Value.kOff);
		updateEncoders();
		
		switch (autonMode) {
			case 1: // Cross
				autonCross();
				break;
			case 2: // Side
				switch (switchSide) {
					case "front":
						break;
					case "side":
						break;
					case "back":
						switch (autonStep) {
							case "firstStraight":
								if (robotPosition == 1 || robotPosition == 3) {
									autonForward(12); // What distance?
									autonStep = "firstAutonTurn90";
								} else if (robotPosition == 2 || robotPosition == 4) {
									autonForward(12); // What distance?
									autonStep = "firstAutonTurnNegative90";
								}
								break;
							case "firstAutonTurn90":
								autonTurn(90);
								break;
							case "firstAutonTurnNegative90":
								autonTurn(-90);
								break;
							
						}
						break;
				}
				break;
			case 3: // Switch
				break;
			default:
				System.out.println("Invalid Mode");
				adaptiveDrive(0, 0);
				break;
		}
	}
	
	public void teleopUpdateDashboard() {
		SmartDashboard.putNumber("Target Distance:", targetDistance);
		SmartDashboard.putNumber("Aspect Ratio:", aspectRatio);
		SmartDashboard.putNumber("Countour Number:", contourNumber);
		SmartDashboard.putNumber("Running:", pipelineRunning);
		SmartDashboard.putBoolean("Target Found:", foundTarget);
	}
	
	public void driverController() {
		double scale = getDrivePowerScale();
		
		if (leftStick.getRawButton(1)) { //fixZ
			driveTrainShift.set(DoubleSolenoid.Value.kForward);
			ptoShift.set(DoubleSolenoid.Value.kForward);
		}
		else if (rightStick.getTrigger()) {
			driveTrainShift.set(DoubleSolenoid.Value.kReverse);
			ptoShift.set(DoubleSolenoid.Value.kReverse);
		}
		else {
			driveTrainShift.set(DoubleSolenoid.Value.kOff);
			ptoShift.set(DoubleSolenoid.Value.kOff);
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
	
	public void intakeRoller(boolean in, boolean out) {
		if (in) {
			intake.set(0.65);
			//lowerIntake.set(0.65);
			beatingStick.set(ControlMode.PercentOutput, 0.65);
		} else if (out) {
			intake.set(-0.65);
			//lowerIntake.set(-0.65);
			beatingStick.set(ControlMode.PercentOutput, -0.65);
		} else {
			intake.set(0);
			//lowerIntake.set(0);
			beatingStick.set(ControlMode.PercentOutput, 0);
		}
	}
	
	public void intakePiston(boolean toogle) {
		/*if (in == true) {
			intakePiston.set(DoubleSolenoid.Value.kForward);
		} else if (out == true) {
			intakePiston.set(DoubleSolenoid.Value.kReverse);
		} else {
			intakePiston.set(DoubleSolenoid.Value.kOff);
		}*/
	}
	
	public void outakeLowerRamp(boolean run) {
/*
		if (run >= 0.5) {
			lowerRamp.set(lowerRampSpeeds[lowerRampSpeed]);
		} else if (run <= -0.5) {
			lowerRamp.set(-lowerRampSpeeds[lowerRampSpeed]);
		} else {
			lowerRamp.set(0);
		}
*/
		if (run) {
			lowerRamp.set(-1.0);
		}
		else {
			lowerRamp.set(0.0);
		}
	}
	
	public void outakeUpperRamp(boolean run) {
		if (run) {
//			upperRamp.set(upperRampSpeeds[upperRampSpeed]);
			upperRamp.set(1.0);
		} else {
			upperRamp.set(0.0);
		}
	}
	 
	public void operaterController() {
		intakeRoller(operatorStick.getRawButton(5), operatorStick.getRawButton(6));
		intakePiston(operatorStick.getRawButtonReleased(3)); // a open/close toggle (3)
		outakeLowerRamp(operatorStick.getRawButton(4)); // left up and down w/ speed
		outakeUpperRamp(operatorStick.getRawButton(2)); // right up and down w/ speed
		
		/*
			if (operatorStick.getRawButtonPressed(3)) { // left toggle speed options
				lowerRampSpeed += 1;
				if (lowerRampSpeed >= lowerRampSpeeds.length) {
					lowerRampSpeed = 0;
				}
			}
			
			if (operatorStick.getRawButtonPressed(2)) { // right toggle speed options
				upperRampSpeed += 1;
				if (upperRampSpeed >= upperRampSpeeds.length) {
					upperRampSpeed = 0;
				}
			}
		*/
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
		
		resetDistanceAndYaw();
	}
	
	@Override
	public void teleopPeriodic() {
		updateEncoders();
		driverController();
		operaterController();
		teleopUpdateDashboard();
	}
}
