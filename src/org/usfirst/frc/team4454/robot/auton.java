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
		int robotPosition = 2;
		int autonMode = 2;
		switch (autonMode) {
				case 1: // Cross
					// Find correct cross length and end after done.
					robot.autonForward(30);
					break;
				case 2: // Place
					switch (switchSide) {
						case "front":
							switch(autonStep) {
							case "firstForward":
								robot.autonForward(81.5);
								autonStep = "firstTurn";
								break;
							case "firstTurn":
								if (((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 3 || robotPosition == 4)  && cubePlacement == "right")) {
									robot.autonTurn(-90);
								} else if ((robotPosition == 1 && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2) && cubePlacement == "right")) {
									robot.autonTurn(90);
								}
								
								autonStep = "secondForward";
								break;
							case "secondForward":
								System.out.println("forward");
								
								autonStep = "secondTurn";
								break;
							case "secondTurn":
								if (((robotPosition == 2 || robotPosition == 3 || robotPosition == 4) && cubePlacement == "left") || ((robotPosition == 3 || robotPosition == 4) && cubePlacement == "right")) {
									robot.autonTurn(-90);
								} else if ((robotPosition == 1 && cubePlacement == "left") || ((robotPosition == 1 || robotPosition == 2) && cubePlacement == "right")) {
									robot.autonTurn(90);
								}
								
								autonStep = "thirdForward";
								break;
							case "thirdForward":
								System.out.println("forward");
								autonStep = "place";
								break;
							case "place":
								System.out.println("place");
								autonStep = "end";
								break;
							case "end":
								break;
							}
							break;
						case "side": // left
							switch (autonStep) {
								case "firstForward":
									System.out.println("forward");
									autonStep = "firstTurn";
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
									System.out.println("place");
									autonStep = "end";
									break;
								case "end":
									break;
							}
							break;
						case "back":
							switch (autonStep) {
								case "firstForward":
									System.out.println("forward");
									if (((robotPosition == 1 || robotPosition == 3 || robotPosition == 4) && (cubePlacement == "left")) || ((robotPosition == 3 || robotPosition == 4) && (cubePlacement == "right"))) {
										autonStep = "secondForward";
									} else if (((robotPosition == 2) && (cubePlacement == "left")) || ((robotPosition == 1 || robotPosition == 2) && (cubePlacement == "right"))) {
										autonStep = "firstTurn";
									}
									break;
								case "firstTurn":
									if (((robotPosition == 1) && (cubePlacement == "left")) || ((robotPosition == 1 || robotPosition == 2) && (cubePlacement == "right"))) {
										System.out.println("turn 90 degrees");
									} else if (((robotPosition == 3 || robotPosition == 4) && (cubePlacement =="left")) || ((robotPosition == 3 || robotPosition == 4) && (cubePlacement == "right"))) {
										System.out.println("turn -90 degrees");
									}
									autonStep = "secondForward";
									break;
								case "secondForward":
									System.out.println("forward");
									autonStep = "secondTurn";
									break;
								case "secondTurn":
									if ((robotPosition == 3 || robotPosition == 4) && (cubePlacement == "left")) {
										System.out.println("turn -90 degrees");
									} else if ((robotPosition == 1) && (cubePlacement == "left")) {
										System.out.println("turn 90 degrees");
									}
									autonStep = "place";
									break;
								case "place":
									System.out.println("place");
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
