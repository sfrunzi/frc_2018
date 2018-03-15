package org.usfirst.frc.team4454.robot;

public class auton {
	//a reference to the parent class
	private RobotInterface robot;
	
	public auton(Robot robot2) {
		robot = robot2;
	}
	
	/**
	 * This is where the if-statements would go to decide when and what to do.
	 */
	public void Run()
	{
		String autonStep = "firstForward";
		String switchSide = "front";
		String cubePlacement = "right";
		boolean startStep = true;
		int robotPosition = 2;
		int autonMode = 2;
		//robot.autonPlaceCubeStart = true; is needed
		switch (autonMode) {
				case 1: // Cross
					// Find correct cross length and end after done. Also flip it because it is going backwards
					if (!robot.autonForward_Done()) {
						robot.autonForward(121.56, true);
					} else {
						autonStep = "end";
					}
					break;
				case 2: // Place
					switch (switchSide) {
						case "front":
							if (startStep) {
								robot.firstTimeAuton(true);
								startStep = false;
							}
							
							switch(autonStep) {
							case "firstForward":
								if (((robotPosition == 1 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
									if (!robot.autonForward_Done()) {
										robot.autonForward(81, true);
									} else {
										autonStep = "firstTurn";
										startStep = true;
									}
								} else if ((robotPosition == 2 && cubePlacement == "left") || ((robotPosition == 1) && cubePlacement == "right")) {
									if (!robot.autonForward_Done()) {
										robot.autonForward(55, true);
									} else {
										autonStep = "firstTurn";
										startStep = true;
									}
								} else if (robotPosition == 3 && cubePlacement == "right") {
									if (!robot.autonForward_Done()) {
										robot.autonForward(98, true);
									} else {
										autonStep = "place";
										startStep = true;
									}
								} else if (robotPosition == 3 && cubePlacement == "right") {
									if (!robot.autonForward_Done()) {
										robot.autonForward(48, true);
									} else {
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
									autonStep = "secondForward";
									startStep = true;
								}
								break;
							case "secondForward":
								if (!robot.autonForward_Done()) {
									if (((robotPosition == 1) && cubePlacement == "left")) {
										robot.autonForward(82, true);
									} else if ((robotPosition == 2 && cubePlacement == "left")) {
										robot.autonForward(51, true);
									} else if ((robotPosition == 3 && cubePlacement == "left")) {
										robot.autonForward(106, true);
									} else if ((robotPosition == 4 && cubePlacement == "left")) {
										robot.autonForward(167, true);
									} else if ((robotPosition == 1 && cubePlacement == "right")) {
										robot.autonForward(201, true);
									} else if ((robotPosition == 2 && cubePlacement == "right")) {
										robot.autonForward(64, true);
									} else if ((robotPosition == 4 && cubePlacement == "right")) {
										robot.autonForward(80, true);
									}
								} else {
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
									autonStep = "place";
									startStep = true;
								}
								
								break;
							case "place":
								if (!robot.autonPlaceCube_Done()) {
									robot.autonPlaceCube(5); // todo: correct time
								} else {
									autonStep = "end";
									startStep = true;
								}
								break;
							case "end":
								break;
							}
							break;
						case "side": // WIP
							switch (autonStep) {
								case "firstForward":
									if (((robotPosition == 1) && cubePlacement == "left") || ((robotPosition == 4)  && cubePlacement == "right")) {
										robot.autonForward(138, true);
										autonStep = "firstTurn";
									} else if ((robotPosition == 2 || robotPosition == 3 || robotPosition == 4 && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2 || robotPosition == 3) && cubePlacement == "right")) {
										robot.autonForward(81, true);
										autonStep = "firstTurn";
									}
									break;
								case "firstTurn":
									if (((robotPosition == 1 || robotPosition == 2) && cubePlacement == "left") || ((robotPosition == 4 || robotPosition == 1)  && cubePlacement == "right")) {
										System.out.println("turn -90 degrees");
									} else if ((robotPosition == 4&& cubePlacement == "left") || (robotPosition == 1 && cubePlacement == "right")) {
										System.out.println("turn 90 degrees");
									}
									
									autonStep = "secondForward";
									break;
								case "secondForward":
									System.out.println("forward");
									
									if ((robotPosition == 1 && cubePlacement == "left") || ((robotPosition == 4 || robotPosition == 1) && cubePlacement == "right")) {
										autonStep = "place";
									} else if ((robotPosition == 4 && cubePlacement == "left") || (robotPosition == 1 && cubePlacement == "right")) {
										autonStep = "secondTurn";
									}
									break;
								case "secondTurn":
									if ((robotPosition == 4 && cubePlacement == "left") || (robotPosition == 1 && cubePlacement == "right")) {
										System.out.println("turn -90 degrees");
									}
									autonStep = "thirdForward";
									break;
								case "thirdForward":
									System.out.println("forward");
									autonStep = "thirdTurn";
									break;
								case "thirdTurn":
									if ((robotPosition == 4 && cubePlacement == "left") || (robotPosition == 1 && cubePlacement == "right")) {
										System.out.println("turn -90 degrees");
									}
									autonStep = "forthForward";
									break;
								case "forthForward":
									System.out.println("forward");
									autonStep = "place";
									break;
								case "place":
									autonStep = "end";
									break;
								case "end":
									break;
							}
							break;
					}
					break;
				case 3: // Switch
					break;
				default:
					System.out.println("Invalid Mode");
					//adaptiveDrive(0, 0);
					break;
		}
	}	
}
