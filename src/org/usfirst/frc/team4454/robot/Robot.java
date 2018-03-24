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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

// PWM ID 0 -- two front motors with compliant wheels
// PWM ID 1 -- two rear motors with four compliant wheel shafts
// PWM ID 2 -- two intake motors
// CAN ID 9 -- front intake bar 
public class Robot extends IterativeRobot implements RobotInterface {
	UsbCamera intakeCamera = CameraServer.getInstance().startAutomaticCapture("intake", 0);
	//UsbCamera intakeCamera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0");
	CvSource intakeOutputStream;
	VisionThread intakeVisionThread;
	DigitalInput beamBreak;
	
	//UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture("intake", "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0");
	
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
	
	boolean intakePistonToggle = true;
	
	double autonCrossTime;
	double autonCrossWait = 3000;
	
	double autonPlaceTime;
	double autonPlaceWait = 3000;
	
	double autonSpeed;
	
	boolean autonPlaceCubeStart;
	boolean autonPlaceCubeDone;
	double autonPlaceCubeTime;
	
	boolean startStep = true;
	
	boolean highGear = false;
	
		
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
	
	SendableChooser<Double> autonWaitChoose;
	double autonWait;
	
	SendableChooser<String> autonChooseLeft;
	String autonModeLeft;
	
	SendableChooser<String> autonChooseRight;
	String autonModeRight;
	
	SendableChooser<Integer> robotPositionChoose;
	int robotPosition;
	
	SendableChooser<String> switchPositionChoose;
	String switchPosition;
	
	SendableChooser<String> autonAllianceChoose;
	String autonAlliance;
	
	private Auton rr;
	
	public void runAuton() {
		//run the loop which has the logic to tell us what to do
		//e.g. we are the controlled and robotrunnable is the controller
		rr.Run();
	}
	
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
		
		SmartDashboard.putNumber("Left Encoder Count", countL);
		SmartDashboard.putNumber("Right Encoder Count", countR);
		
		//System.out.println("Left Encoder Count " + countL + " Encoder distance " + distanceL);
		//System.out.println("Right Encoder Count " + countR + " Encoder distance " + distanceR);
	}
	
	public double getDrivePowerScale() {
		double scale = 0.75;
		
		if ( leftStick.getTrigger() || rightStick.getTrigger() ) {
			scale = 0.85;
		}
		
		if (leftStick.getTrigger() && rightStick.getTrigger()) {
			scale = 1;
		}
		
		if (!highGear) {
			return 1;
		} else {
			return scale;
		}
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
		SmartDashboard.putNumber("Auton Wait Choose", 0.0);
		
		//SmartDashboard.putString("Auton Mode","cross");
		SmartDashboard.putString("hsvThresholdHue","[0.0, 255.0]");
		SmartDashboard.putString("hsvThresholdSaturation", "[15.0, 255.0]");
		SmartDashboard.putString("hsvThresholdValue", "[0.0, 255.0]");
		SmartDashboard.putString("filterContoursSolidity", "[0, 100]");

		autonChooseLeft = new SendableChooser<>();
		
		autonChooseLeft.addDefault("cross", "cross");
		autonChooseLeft.addObject("place", "place");
		autonChooseLeft.addObject("exchange", "exchange");
		
		SmartDashboard.putData("Auton Mode if left", autonChooseLeft);
		
		autonChooseRight = new SendableChooser<>();
		
		autonChooseRight.addDefault("cross", "cross");
		autonChooseRight.addObject("place", "place");
		autonChooseRight.addObject("exchange", "exchange");
		
		SmartDashboard.putData("Auton Mode if Right", autonChooseRight);
		
		robotPositionChoose = new SendableChooser<>();
		
		robotPositionChoose.addDefault("1", 1);
		robotPositionChoose.addObject("2", 2);
		robotPositionChoose.addObject("3", 3);
		
		SmartDashboard.putData("Robot Position", robotPositionChoose);
		
		switchPositionChoose = new SendableChooser<>();
		
		switchPositionChoose.addDefault("front", "front");
		switchPositionChoose.addObject("side", "side");
		
		SmartDashboard.putData("Switch Position", switchPositionChoose);
		
		autonAllianceChoose = new SendableChooser<>();
		
		autonAllianceChoose.addDefault("red", "red");
		autonAllianceChoose.addObject("blue", "blue");
		
		SmartDashboard.putData("Alliance", autonAllianceChoose);
		
		intake = new Talon(2);
	    lowerRamp = new Talon(0);
	    upperRamp = new Talon(1);
	    beatingStick = new TalonSRX(9);
	    beatingStick.setInverted(true);
	    
	    //climberTop = new TalonSRX(8);
	    //climberBottom = new TalonSRX(9);
		
		intakeCamera.setResolution(320, 240);
		//backCamera.setResolution(320, 240);
		
		intakeOutputStream = CameraServer.getInstance().putVideo("intakeOverlay", 320, 240);
		intakeOutputStream.setFPS(5);
		
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		operatorStick = new Joystick(2);
		
		beamBreak = new DigitalInput(4); // todo: Real id
		
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
	@Override
	public double getAutonWait() {
		return autonWait;
	}
	
	@Override
	public String autonSwitchPosition() {
		return switchPosition;
	}
	
	@Override
	public String autonSwitchSide() {
		return switchSide;
	}
	
	@Override
	public String autonGetMode() {
		if (switchSide == "left") {
			return autonModeLeft;
		} else if (switchSide == "right") {
			return autonModeRight;
		} else {
			return "cross";
		}
	}
	
	@Override
	public int autonRobotPosition() {
		return robotPosition;
	}
	
	@Override
	public boolean autonPlaceCube_Done() {
		return autonPlaceCubeDone;
	}
	
	@Override
	public boolean autonForward_Done() {
		return autonForwardDone;
	}
	
	@Override
	public boolean autonReverse_Done() {
		return autonReverseDone;
	}
	
	@Override
	public boolean autonTurn_Done() {
		return autonTurnDone;
	}
	
	@Override
	public void firstTimeAuton(boolean set) {
		firstTimeAuton = set;
	}
	
	@Override
	public void autonPlaceCube(double time) {
		if (firstTimeAuton) {
			autonPlaceCubeDone = false;
			autonPlaceCubeTime = System.currentTimeMillis();
			firstTimeAuton = false;
		}
		
		if ((System.currentTimeMillis() - autonPlaceCubeTime) <= time) {
			outakeUpperRamp(1);
		} else {
			outakeUpperRamp(0);
			autonPlaceCubeDone = true;
		}
	}
	
	@Override
	public void autonForward(double distance, boolean flip) {
		if (firstTimeAuton) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
			autonForwardDone = false;
			System.out.println("first run of forward");
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= ((distance / inchPerSecond) * 1000)) {
			System.out.println("moving forward");
			if (!flip) {
				adaptiveDrive(1.0 * autonSpeed, 1.0 * autonSpeed);
			} else {
				adaptiveDrive(-1.0 * autonSpeed, -1.0 * autonSpeed);
			}
		} else {
			adaptiveDrive(0, 0);
			autonForwardDone = true;
		}
	}
	
	@Override
	public void autonReverse(double distance, boolean flip) {
		if (firstTimeAuton == true) {
			firstTimeAuton = false;
			autonCrossTime = System.currentTimeMillis();
			autonReverseDone = false;
		}
		
		if ((System.currentTimeMillis() - autonCrossTime) <= ((distance / inchPerSecond) * 1000) + autonCrossWait && (System.currentTimeMillis() - autonCrossTime) >= autonCrossWait) {
			if (!flip) {
				adaptiveDrive(-1.0 * autonSpeed, -1.0 * autonSpeed);
			} else {
				adaptiveDrive(1.0 * autonSpeed, 1.0 * autonSpeed);
			}
		} else {
			adaptiveDrive(0, 0);
			autonReverseDone = true;
		}
	}
	
	@Override
	public void autonTurn(double turnAngle, boolean flip) {
		double temp = Math.signum(turnAngle) * autonSpeed;
		if (!flip) {
			setDriveMotors(temp, -temp);
		} else {
			setDriveMotors(-temp, temp);
		}
		if ((turnAngle == 0.0) || (Math.abs(ahrs.getAngle()) > Math.abs(turnAngle))) {
			autonTurnDone = true;
			setDriveMotors(0.0, 0.0);
			resetDistanceAndYaw();
		}
	}
	
	@Override
	public void autonomousInit() {
		rr = new Auton(this);
		
		intakePiston.set(DoubleSolenoid.Value.kForward);
		ptoShift.set(DoubleSolenoid.Value.kForward);
		highGear = false;
		driveTrainShift.set(DoubleSolenoid.Value.kForward);
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (gameData.length() > 0) {
			if (autonAlliance == "red" ) {
				if (gameData.charAt(0) == 'L') {
					switchSide = "left";
				} else {
					switchSide = "right";
				}
			} else if (autonAlliance == "blue") {
				if (gameData.charAt(2) == 'L') {
					switchSide = "left";
				} else {
					switchSide = "right";
				}
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
		
		autonModeLeft = autonChooseLeft.getSelected();
		autonModeRight = autonChooseRight.getSelected();
		robotPosition = robotPositionChoose.getSelected();
		switchPosition = switchPositionChoose.getSelected();
		autonAlliance = autonAllianceChoose.getSelected();
	}
	
	@Override
	public void autonomousPeriodic() {
		updateEncoders();
		autonWait = SmartDashboard.getNumber("Auton Wait Choose", 0.0);
		runAuton();
	}
	
	public void teleopUpdateDashboard() {
		SmartDashboard.putNumber("Target Distance:", targetDistance);
		SmartDashboard.putNumber("Aspect Ratio:", aspectRatio);
		SmartDashboard.putNumber("Countour Number:", contourNumber);
		SmartDashboard.putNumber("Running:", pipelineRunning);
		SmartDashboard.putBoolean("Target Found:", foundTarget);
		SmartDashboard.putBoolean("Beam Break", getBeamBreak());
	}
	
	public boolean getBeamBreak(){
		return beamBreak.get();
	}
	
	public void driverController() {
		double scale = getDrivePowerScale();
		
		if (rightStick.getRawButton(2)) {
			//high gear
			highGear = true;
			driveTrainShift.set(DoubleSolenoid.Value.kReverse);
			//ptoShift.set(DoubleSolenoid.Value.kReverse);
		} else if (leftStick.getRawButton(2)) {
			//low gear
			highGear = false;
			driveTrainShift.set(DoubleSolenoid.Value.kForward);
			//ptoShift.set(DoubleSolenoid.Value.kForward);
		}
		
		if (!leftStick.getRawButton(4) && !leftStick.getRawButton(4)) {
			adaptiveDrive(scale * (-1 * leftStick.getY()), scale * (-1 * rightStick.getY()));
		} else {
			adaptiveDrive(scale * (leftStick.getY()), scale * (rightStick.getY()));
		}
	}
	
	public void intakeRoller(double run) {
		//if (!getBeamBreak()) {
			if (operatorStick.getRawAxis(2) >= 0.1) {
				intake.set(1);
			} else if (operatorStick.getRawAxis(3) >= 0.1) {
				intake.set(-1);
			} else if (run >= 0.1 || run <= -0.1) {
				intake.set(run);
			} else {
				intake.set(0);
			}
			
			lowerRamp.set(run * 0.5);
			
			if (run > 0.1) {
				beatingStick.set(ControlMode.PercentOutput, -1);
			} else if (run < -0.1) {
				beatingStick.set(ControlMode.PercentOutput, 1);
			} else {
				beatingStick.set(ControlMode.PercentOutput, 0);
			}
		/*} else {
			lowerRamp.set(0);
			intake.set(0);
			beatingStick.set(ControlMode.PercentOutput, 0);
		}*/
	}
	
	public void intakePiston(boolean toggle) {
		if (toggle == true) {
			if (intakePistonToggle) {
				intakePistonToggle = false;
			} else {
				intakePistonToggle = true;
			}
		}
		
		SmartDashboard.putBoolean("Intake Piston", !intakePistonToggle);
		if (intakePistonToggle == true) {
			intakePiston.set(DoubleSolenoid.Value.kForward);
		} else if (intakePistonToggle == false) {
			intakePiston.set(DoubleSolenoid.Value.kReverse);
		}/* else {
			intakePiston.set(DoubleSolenoid.Value.kOff);
		}*/
	}
	
	public void outakeUpperRamp(double run) {
		upperRamp.set(run * -0.40);
	}
	
	public void reverseOutake(boolean run) {
		if (run) {
			upperRamp.set(0.25);
		}
	}
	 
	public void operaterController() {
		intakePiston(operatorStick.getRawButtonReleased(1));
		reverseOutake(operatorStick.getRawButton(2));
		
		//if (!operatorStick.getRawButton(2)) {} // After rewire
		intakeRoller(operatorStick.getRawAxis(1));
		outakeUpperRamp(operatorStick.getRawAxis(5));
	}
	
	@Override
	public void teleopInit() {
		System.out.println("Beam Break" + getBeamBreak());
		
		intakePiston.set(DoubleSolenoid.Value.kForward);
		ptoShift.set(DoubleSolenoid.Value.kForward);
		
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
