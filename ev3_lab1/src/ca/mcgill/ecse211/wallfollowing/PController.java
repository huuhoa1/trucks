package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

	/* Constants */
	private static final int MOTOR_SPEED = 180;
	private static final int FILTER_OUT = 30;
	private static final int MAXCORRECTION = 40;
	private static final int PROCONST = 6;

	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;
	private int difference;

	public PController(int bandCenter, int bandwidth) {
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);	 // Initialize motor rolling forward
		WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);	//Initialize motor rolling forward
		WallFollowingLab.leftMotor.forward();
		WallFollowingLab.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {

		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
		//

		if (distance >= 40 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the
			// filter value
			filterControl++;
		} 
		else if (distance >= 40) {
			// We have repeated large values, so there must actually be nothing
			// there: leave the distance alone
			this.distance = distance;
		}
		else {
			// distance went below 40: reset filter and leave
			// distance alone.
			filterControl = 0;
			this.distance = distance;
		}
		// TODO: process a movement based on the us distance passed in (P style)

		//ERROR = CONTROL DISTANCE- MEAURED DISTANCE
		int ERROR=this.bandCenter-this.distance;

		//if right distance, go forward and set speed to initial speed
		if(Math.abs(ERROR)<=bandWidth) {
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		}

		/* too close to the wall, increase inner wheel, and decrease outer wheel using the proportions calculated
		 * from calcProp method */
		else if(ERROR>bandWidth) {
			difference=calcProp(ERROR);
			/* condition if the robot is too close to the wall, it will move backward to allow some space
			 * will back up away from the wall as the left wheel is set to a higher speed than right wheel */
			if(distance<14) {
				WallFollowingLab.rightMotor.backward();
				WallFollowingLab.leftMotor.backward();
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-difference);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+difference);
			}
			else {
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
				WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+difference);
				WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-difference);
			}
		}

		/* if too far from the wall, speed up the outer wheel, and decrease the inner wheel using the proportions
		 * calculated from calcProp method */
		else if(ERROR<-bandWidth) {
			difference=calcProp(ERROR);
			/* condition if the robot is too far from the wall
			usually would enter this condition when the robot is going backward to allow space, but is backing up too much
			prevents from going in reverse sense */
			if(distance>40) {
				WallFollowingLab.rightMotor.forward();
				WallFollowingLab.leftMotor.forward();
			}
			WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+difference);
			WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-difference);
		}
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}

	/* method that calculates the proportion of a relation between the error and the speed of the robot
	 * this method is taken directly from Frank Ferry's Lecture Notes
	 * Lecture 3-4 Wall following, Simple Feedback Control slide 22 */
	int calcProp(int diff) {
		int correction;
		if(diff<0) 
			diff=-diff;
		correction=(int)(PROCONST*(double)diff);
		if(correction>=MOTOR_SPEED)correction=MAXCORRECTION;
		return correction;
	}
}
