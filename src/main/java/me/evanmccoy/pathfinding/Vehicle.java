package me.evanmccoy.pathfinding;

import java.util.Calendar;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import me.evanmccoy.pathfinding.Pathfinding.Path;

/**
 * A class which represents a mobile vehicle.  The class can be controlled by setting left/right velocities and automatically updates its velocities
 * according to its acceleration and position according to its velocity.
 * Created - February 2018
 * 
 * @author Evan McCoy
 *
 */
public class Vehicle {
	public static double ACCELERATION = 24, WHEEL_ACCELERATION = 96;
	private Path path;
	private Thread t;
	//The (x, y) position on the cartesian plane.
	private Vector2D position = new Vector2D(-4, -4);
	//The total distance traveled
	private double distance;
	//The left/right wheel velocities.
	private Vector2D velocity = new Vector2D(0, 0);
	//The target left/right wheel velocities
	private Vector2D targetVelocity = new Vector2D(0, 0);
	private double baseVelocity = 3;
	private double width = 1;
	
	public Vehicle(Path path) {
		this.path = path;
		t = new Engine();
		t.start();
	}
	
	/**
	 * The total distance traveled along the path.
	 * 
	 * @return - the distance
	 */
	public double distance() {
		return distance;
	}
	
	/**
	 * Returns a clone of the position array.
	 * Array is in the form [x, y]
	 * 
	 * @return - the position
	 */
	public Vector2D position() {
		return new Vector2D(position.getX(), position.getY());
	}
	
	/**
	 * Returns the left wheel velocity.
	 * 
	 * @return - the velocity in inches/second
	 */
	public double leftVelocity() {
		return velocity.getX();
	}
	
	/**
	 * Returns the right wheel velocity.
	 * 
	 * @return - the velocity in inches/second
	 */
	public double rightVelocity() {
		return velocity.getY();
	}
	
	/**
	 * Returns the velocity of the robot as a whole.
	 * 
	 * Taking the left and right wheel velocities, the robot will be moving in an arc where the angular velocity is the same for both sides.
	 * Therefore, as the radius and circumference of this arc in its full form have a direct linear relationship, the center velocity of the robot is merely
	 * the average of the left and right velocities.
	 * 
	 * @return - the velocity in inches/second
	 */
	public double velocity() {
		return (leftVelocity() + rightVelocity()) / 2;
	}
	
	/**
	 * Sets the left and right target velocities.
	 * 
	 * @param left - the target left velocity in inches/second
	 * @param right - the target right velocity in inches/second
	 */
	public void setTargetVelocity(double left, double right) {
		targetVelocity = new Vector2D(left, right);
	}
	
	/**
	 * Finds the appropriate target velocity for the left wheels given the current distance traveled.
	 * 
	 * @return - the left target velocity.
	 */
	public double calcLeftVelocity() {
		double radius = path.getOsculatingRadius(distance());
		Path.TurningDirection dir = path.getTurningDirection(distance());
		if (dir == Path.TurningDirection.LEFT) {
			return baseVelocity * (radius - width/2.0) / radius;
		}
		else {
			return baseVelocity * (radius + width/2.0) / radius;
		}
	}
	
	/**
	 * Finds the appropriate target velocity for the right wheels given the current distance traveled.
	 * 
	 * @return - the right target velocity.
	 */
	public double calcRightVelocity() {
		double radius = path.getOsculatingRadius(distance());
		Path.TurningDirection dir = path.getTurningDirection(distance());
		if (dir == Path.TurningDirection.LEFT) {
			return baseVelocity * (radius + width/2.0) / radius;
		}
		else {
			return baseVelocity * (radius - width/2.0) / radius;
		}
	}
	
	/**
	 * Finds the radius of curvature using the current left and right velocities.
	 * 
	 * @return - the radius of curvature
	 */
	public double calcRadius() {
		double leftVel = leftVelocity();
		double rightVel = rightVelocity();
		if (leftVel == rightVel) {
			return Integer.MAX_VALUE;
		}
		double outerVel;
		double innerVel;
		if (leftVel > rightVel) {
			outerVel = leftVel;
			innerVel = rightVel;
		}
		else {
			outerVel = rightVel;
			innerVel = leftVel;
		}
		// r = ((-1 - Vo/Vi) / (1 - Vo/Vi)) * halfWidth
		return (-1 - outerVel/innerVel) / (1 - outerVel / innerVel) * (width/2.0);
	}
	
	/**
	 * Terminates the update thread of this vehicle.
	 */
	public void terminate() {
		t.interrupt();
	}
	
	/**
	 * A thread that handles updating position and velocity of the vehicle according to targets and time.
	 * 
	 * @author Evan McCoy
	 *
	 */
	private class Engine extends Thread {
		public double TIME_PER_CYCLE = 0.02;
		@Override
		public void run() {
			//Run at 50hz
			while (true) {
				//Terminate thread
				if (Thread.currentThread().isInterrupted()) {
					return;
				}
				
				long time = Calendar.getInstance().getTimeInMillis();
				
				double left = leftVelocity();
				double right = rightVelocity();
				
				//Update distance
				distance += velocity() * TIME_PER_CYCLE;
				if (path.isFinished(distance())) {
					//path.recordSimulationComplete(position, distance(), WHEEL_ACCELERATION);
					distance = 0;
					position = new Vector2D(-4, -4);
					velocity = new Vector2D(0, 0);
					targetVelocity = new Vector2D(0, 0);
					return;
				}
				
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Simulation Updates																				  //
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				
				//Update position
				position = path.getRotatedPosition(distance(), position(), calcRadius(), velocity(), TIME_PER_CYCLE);
				path.updateFeatures(distance(), position(), calcRadius());
				
				//Set target velocities based on distance traveled - not a simulation update, but must happen after position update.
				setTargetVelocity(calcLeftVelocity(), calcRightVelocity());
				//Artificially accelerate wheels
				if (left > targetVelocity.getX()) {
					left -= Math.min(WHEEL_ACCELERATION*TIME_PER_CYCLE, left - targetVelocity.getX());
				}
				else {
					left += Math.min(WHEEL_ACCELERATION*TIME_PER_CYCLE, targetVelocity.getX() - left);
				}
				if (right > targetVelocity.getY()) {
					right -= Math.min(WHEEL_ACCELERATION*TIME_PER_CYCLE, right - targetVelocity.getY());
				}
				else {
					right += Math.min(WHEEL_ACCELERATION*TIME_PER_CYCLE, targetVelocity.getY() - right);
				}
				//Limit body accceleration
				if (Math.abs(velocity() - ((left + right) / 2)) > ACCELERATION*TIME_PER_CYCLE) {
					//(Old velocity + acceleration) / new velocity
					double scalar = (Math.abs(velocity()) + ACCELERATION*TIME_PER_CYCLE) / Math.abs((left + right) / 2);
					left *= scalar;
					right *= scalar;
				}
				
				velocity = new Vector2D(left, right);
				
				/////////////////////////////////////////////////////////////////////////////////////////////////////////
				try {
					Thread.sleep(Math.max(0, (long)(1000*TIME_PER_CYCLE - (Calendar.getInstance().getTimeInMillis() - time))));
				} catch (InterruptedException e) {e.printStackTrace();}
			}
		}
		
	}
}
