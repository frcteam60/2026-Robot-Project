
package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.HOOD_ID;

import edu.wpi.first.hal.simulation.AnalogInDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *ShooterHood subsystem
 *@author Harris Jilani
 */
public class ShooterHood extends SubsystemBase {
	private Servo hoodServo;
	private double lastReportedAngle;
	private double min = 54.5;
    private double max = 80;

	/**
	 * Default constructor for ShooterHood subsystem
	 */
	public ShooterHood() {
		hoodServo = new Servo(HOOD_ID);
		hoodServo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
		hoodServo.enableDeadbandElimination(true);
		lastReportedAngle = 0;
	}
   
	/**
	 * sets how far the servo is out with a soft stop
	 * 
	 * @param percent
	 */
	public void setPosition(double percent) { 
		if(percent > 0.7) {
			percent = 0.7;
		} else if(percent < 0.0){
			percent = 0.0;
		}
		hoodServo.setPosition(percent);
	}


	/**
	 * sets hood angle and prevents it from hiting the hard stop 
	 * 
	 * 
	 * @param angle
	 */
	public void setAngle(double angle) {
		if(angle > max) {
			angle = max;
		} else if(angle < min){
			angle = min;
		}
		//double percent = (-0.0275*angle)+2.2;
		double percent = (0.7-((angle-min)/(max-min))*0.7);
		lastReportedAngle = angle;
		hoodServo.setPosition(percent);
		
	}

	/**
	 * Get the servo angle.
	 * This returns the commanded angle, not the angle that the servo is actually at, as the servo does not report its own angle.
	 * @return 
	 */
	public double getLastAngle(){
		return lastReportedAngle;
	}
	
	// /**
	//  * Finds the motor's current angle. Note this doesn't reflect the true
	//  *   position of the hood, but the last position the hood was commanded to.
	//  *   It will not account for the time it takes for the mechanism to get to
	//  *   its destination position. Therefore this method SHOULD NOT be used to
	//  *   qualify a command has completed moving the hood.
	//  * @return the angle (degrees) the motor was last commanded to.
	//  */
	// public double getAngle() {
	// 	return hoodServo.getAngle();
	// }
}
