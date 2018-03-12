package org.usfirst.frc.team4454.robot;

public interface RobotInterface {
	public void autonPickupCube();
	public void autonPlaceCube();
	void autonTurn(double turnAngle);
	void autonReverse_Time(double time);
	void autonForward_Time(double time);
	void autonReverse(double distance);
	void autonForward(double distance);
}
