package org.usfirst.frc.team4454.robot;

public class Auton {
	private RobotInterface robot;
	
	public Auton(Robot robot2) {
		startStep = true;
		firstRun = true;
		autonHasWait = 0;
		autonStep = "wait";
		robot = robot2;
	}
	
	boolean startStep;
	boolean firstRun;
	double autonHasWait;
	String autonStep;
	
	/**
	 * This is where the if-statements would go to decide when and what to do.
	 */
	public void Run() {
		if (firstRun) {
			firstRun = false;
			System.out.println("first run");
		}
		
		double pivotPoint = 27; // 27
		
		String switchSide = robot.autonSwitchPosition();
		String cubePlacement = robot.autonSwitchSide();
		int robotPosition = robot.autonRobotPosition();
		double autonWait = robot.getAutonWait();
		String autonMode = robot.autonGetMode();
		System.out.println(switchSide);
		System.out.println(autonMode);
		System.out.println(autonStep);
		System.out.println(robotPosition);
		switch (autonMode) {
			case "cross": // Cross
				switch (autonStep) {
					case "wait":
						if (startStep) {
							autonHasWait = System.currentTimeMillis();
							startStep = false;
						}
						
						if ((autonWait * 1000) <= (System.currentTimeMillis() - autonHasWait)) {
							startStep = true;
							autonStep = "firstForward";
						}
						break;
					case "firstForward":
						if (startStep) {
							robot.firstTimeAuton(true);
							startStep = false;
						}
						
						if (!robot.autonForward_Done()) {
							robot.autonForward(151.5, true);
						} else {
							robot.firstTimeAuton(true);
							autonStep = "end";
						}
						break;
					case "end":
						break;
				}
				break;
			case "crossPlace": // Cross and Place
				switch (autonStep) {
					case "wait":
						if (startStep) {
							autonHasWait = System.currentTimeMillis();
							startStep = false;
						}

						if ((autonWait * 1000) <= (System.currentTimeMillis() - autonHasWait)) {
							startStep = true;
							autonStep = "firstForward";
						}
						break;
					case "firstForward":
						if (startStep) {
							robot.firstTimeAuton(true);
							startStep = false;
						}
						
						if (!robot.autonForward_Done()) {
							robot.autonForward(145.5, true);
						} else {
							robot.firstTimeAuton(true);
							autonStep = "place";
						}
						break;
					case "place":
						if (!robot.autonPlaceCube_Done()) {
							if ((robotPosition == 2 && cubePlacement == "left") || (robotPosition == 3 && cubePlacement == "right")) {
								robot.autonPlaceCube(3000);
								System.out.println("placing");
							} else {
								autonStep = "end";
								System.out.println("ended");
							}
						} else {
							autonStep = "end";
							System.out.println("ended");
						}
					case "end":
						break;
				}
				break;
				
			case "place": // Place
				switch (switchSide) {
						
					case "front":
						if (startStep) {
							robot.firstTimeAuton(true);
							startStep = false;
						}
						
						switch(autonStep) {
							case "wait": // should be first
								if (startStep) {
									autonHasWait = System.currentTimeMillis();
									startStep = false;
								}
								
								if ((autonWait * 1000) <= (System.currentTimeMillis() - autonHasWait)) {
									autonStep = "firstForward";
									startStep = true;
								}
								break;
							case "firstForward":
								if (((robotPosition == 1 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
									if (!robot.autonForward_Done()) {
										robot.autonForward(81 - pivotPoint, true);
									} else {
										robot.autonForward_Reset();
										autonStep = "firstTurn";
										startStep = true;
									}
								} else if ((robotPosition == 2 && cubePlacement == "left") || ((robotPosition == 1) && cubePlacement == "right")) {
									if (!robot.autonForward_Done()) {
										robot.autonForward(55 - pivotPoint, true);
									} else {
										robot.autonForward_Reset();
										autonStep = "firstTurn";
										startStep = true;
									}
								} else if (robotPosition == 3 && cubePlacement == "right") {
									if (!robot.autonForward_Done()) {
										robot.autonForward(98, true);
									} else {
										robot.autonForward_Reset();
										autonStep = "place";
										startStep = true;
									}
								} else if (robotPosition == 3 && cubePlacement == "right") {
									if (!robot.autonForward_Done()) {
										robot.autonForward(48 - pivotPoint, true);
									} else {
										robot.autonForward_Reset();
										autonStep = "firstTurn";
										startStep = true;
									}
								}
								break;
							case "firstTurn":
								if (!robot.autonTurn_Done()) {
									if (((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
										robot.autonTurn(-90, true);
									} else if ((robotPosition == 1  && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2) && cubePlacement == "right")) {
										robot.autonTurn(90, true);
									}
								} else {
									robot.autonTurn_Reset();
									autonStep = "secondForward";
									startStep = true;
								}
								break;
							case "secondForward":
								if (!robot.autonForward_Done()) {
									if ((robotPosition == 1 && cubePlacement == "left")) {
										robot.autonForward(82 - pivotPoint, true);
									} else if ((robotPosition == 2 && cubePlacement == "left")) {
										robot.autonForward(51 - pivotPoint, true);
									} else if ((robotPosition == 3 && cubePlacement == "left")) {
										robot.autonForward(106 - pivotPoint, true);
									} else if ((robotPosition == 4 && cubePlacement == "left")) {
										robot.autonForward(167 - pivotPoint, true);
									} else if ((robotPosition == 1 && cubePlacement == "right")) {
										robot.autonForward(201 - pivotPoint, true);
									} else if ((robotPosition == 2 && cubePlacement == "right")) {
										robot.autonForward(64 - pivotPoint, true);
									} else if ((robotPosition == 4 && cubePlacement == "right")) {
										robot.autonForward(80 - pivotPoint, true);
									}
								} else {
									robot.autonForward_Reset();
									autonStep = "secondTurn";
									startStep = true;
								}
								break;
							case "secondTurn":
								if (!robot.autonTurn_Done()) {
									if (((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 4) && cubePlacement == "right")) {
										robot.autonTurn(90, true);
									} else if ((robotPosition == 1 && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2) && cubePlacement == "right")) {
										robot.autonTurn(-90, true);
									}
								} else {
									robot.autonTurn_Reset();
									autonStep = "thirdForward";
									startStep = true;
								}
								break;
							case "thirdForward":
								if (!robot.autonForward_Done()) {
									if (((robotPosition == 1 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
										robot.autonForward(17, true);
									} else if ((robotPosition == 2 && cubePlacement == "left") || (robotPosition == 1 && cubePlacement == "right")) {
										robot.autonForward(43, true);
									} else if ((robotPosition == 2 && cubePlacement == "right")) {
										robot.autonForward(50, true);
									}
								} else {
									robot.autonForward_Reset();
									autonStep = "place";
									startStep = true;
								}
								
								break;
							case "place":
								if (!robot.autonPlaceCube_Done()) {
									robot.autonPlaceCube(2000); // todo: correct time
								} else {
									robot.autonPlaceCube_Reset();
									autonStep = "end";
									startStep = true;
								}
								break;
							case "end":
								break;
						}
						break;
					case "side":
						if (startStep) {
							robot.firstTimeAuton(true);
							startStep = false;
						}
						
						switch (autonStep) {
							case "wait":
								if (startStep) {
									autonHasWait = System.currentTimeMillis();
									startStep = false;
								}
								
								if ((autonWait * 1000) <= (System.currentTimeMillis() - autonHasWait)) {
									autonStep = "firstForward";
									startStep = true;
								}
								break; // Stopped here did not set to smartdashboard... I also need the if this then that part done
						
							case "firstForward":
								if (!robot.autonForward_Done()) {
									if (((robotPosition == 1) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
										robot.autonForward(138 - pivotPoint, true);
									} else if ((robotPosition == 2 || robotPosition == 3 || robotPosition == 4 && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3) && cubePlacement == "right")) {
										robot.autonForward(81 - pivotPoint, true);
									}
								} else {
									robot.autonForward_Reset();
									autonStep = "firstTurn";
									startStep = true;
								}
								break;
							case "firstTurn":
								if (!robot.autonTurn_Done()) {
									if (((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
										robot.autonTurn(-90, true);
									} else if (((robotPosition == 1) && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3 ) && cubePlacement == "right")) {
										robot.autonTurn(90, true);
									}
								} else {
									robot.autonTurn_Reset();
									autonStep = "secondForward";
									startStep = true;
								}
								break;
							case "secondForward":
								if (!robot.autonForward_Done()) {
									if ((robotPosition == 1 && cubePlacement == "right") || (robotPosition == 4 && cubePlacement == "left")) {
										robot.autonForward(173 - pivotPoint, true);
									} else if ((robotPosition == 2 && cubePlacement == "right") || (robotPosition == 3 && cubePlacement == "left")) {
										robot.autonForward(120 - pivotPoint, true);
									} else if ((robotPosition == 2 && cubePlacement == "left") || (robotPosition == 3 && cubePlacement == "right")) {
										robot.autonForward(83 - pivotPoint, true);
									} else if (robotPosition == 1 && cubePlacement == "left") {
										robot.autonForward(16 - pivotPoint, true);
									} else if (robotPosition == 4 && cubePlacement == "right")  {
										robot.autonForward(12 - pivotPoint, true);
									}
								} else {
									robot.autonForward_Reset();
									autonStep = "secondTurn";
									startStep = true;
								}
								break;
							case "secondTurn":
								if (!robot.autonTurn_Done()) {
									if ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3) && cubePlacement == "right") {
										robot.autonTurn(-90, true);
									} else if ((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left")  {
										robot.autonTurn(90, true);
									}
								} else {
									robot.autonTurn_Reset();
									autonStep = "thirdForward";
									startStep = true;
								}
								break;
							case "thirdForward":
								if (!robot.autonForward_Done()) {
									if ((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") {
										robot.autonForward(57, true);
									} else if ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3) && cubePlacement == "right") {
										robot.autonForward(57, true);
									}
								} else {
									robot.autonForward_Reset();
									autonStep = "thirdTurn";
									startStep = true;
								}
								break;
							case "thirdTurn":
								if (!robot.autonTurn_Done()) {
									if ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3) && cubePlacement == "right") {
										robot.autonTurn(-90, true);
									} else if ((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") {
										robot.autonTurn(90, true);
									}
								} else {
									robot.autonTurn_Reset();
									autonStep = "fourthForward";
									startStep = true;
								}
								break;
								
							case "fourthForward":
								if (!robot.autonForward_Done()) {
									if ((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") {
										robot.autonForward(16, true);
									} else if ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3) && cubePlacement == "right") {
										robot.autonForward(12, true);
									}
								} else {
									robot.autonForward_Reset();
									autonStep = "place";
									startStep = true;
								}
								break;
							case "place":
								if (!robot.autonPlaceCube_Done()) {
									robot.autonPlaceCube(3); // todo: correct time
								} else {
									robot.autonPlaceCube_Reset();
									autonStep = "end";
									startStep = true;
								}
								break;
							case "end":
								break;
						}
						break;
				}
				break;
			case "switch": // Switch
				break;
			default:
				System.out.println("Invalid Mode");
				break;
		}
	}	
}
