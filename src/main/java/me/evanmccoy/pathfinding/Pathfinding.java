package me.evanmccoy.pathfinding;

import java.awt.Dimension;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import me.evanmccoy.pathfinding.path.Point;
import me.evanmccoy.pathfinding.path.Spline;
import me.evanmccoy.pathfinding.path.natural.NaturalCubicInterpolator;
import me.evanmccoy.pathfinding.path.natural.NaturalCubicSpline;

/**
 * A software controlling the interpolation, vehicle pathfinding, and visualization of paths defined by cubic splines.
 * Created - February 2018
 * 
 * @author Evan McCoy
 *
 */
public class Pathfinding {
	static Grapher grapher;
	static double[][] points = {{-4, -4}, {-2, 6}, {0, 8}, {2, -2}, {4, -1}, {6, 2}, {8, 3}, {10, 8}, {12, 16}, {14, 25}, {16, 26}, {18, 20}};
	
	public static class Path {
		private Vehicle vehicle;
		private NaturalCubicSpline[] path;
		
		public Path() {
			Point[] pointObjs = new Point[points.length];
			for (int i = 0;i < points.length;i++) {
				pointObjs[i] = new Point(points[i][0], points[i][1]);
			}
			path = new NaturalCubicInterpolator(pointObjs, 0.1, 0).calculate();
			vehicle = new Vehicle(this);
		}
		
		/**
		 * Returns the array of splines that compose this path.
		 * 
		 * @return - the spline array.
		 */
		public NaturalCubicSpline[] getPath() {
			return path;
		}
		
		/**
		 * The vehicle that is traversing this path.
		 * 
		 * @return - the vehicle.
		 */
		public Vehicle getVehicle() {
			return vehicle;
		}
		
		/**
		 * Gets the current radius of curvature of the path given a distance traveled.
		 * 
		 * @param distance - the current distance traveled.
		 * @return - the radius.
		 */
		public double getOsculatingRadius(double distance) {
			Spline r = MathUtils.activeSpline(path, distance);
			double t = MathUtils.findT(path, distance);
			return MathUtils.osculatingRadius(r, t);
		}
		
		/**
		 * Gets the LEFT or RIGHT turning direction given a distance traveled.
		 * 
		 * @param distance - the current distance traveled.
		 * @return - the turning direction.
		 */
		public TurningDirection getTurningDirection(double distance) {
			Spline r = MathUtils.activeSpline(path, distance);
			double t = MathUtils.findT(path, distance);
			return (MathUtils.d2ydx2(r).apply(t) > 0) ? TurningDirection.LEFT : TurningDirection.RIGHT;
		}
		
		/**
		 * Used only for simulation, given the current properties of movement of the vehicle, this generates an updated
		 * position vector.
		 * 
		 * @param distance - the current distance traveled.
		 * @param position - the current calculated position.
		 * @param radius - the current calculated position.
		 * @param velocity - the current calculated velocity.
		 * @param time - the elapsed time between updates.
		 * @return - the re-calculated position vector.
		 */
		public Vector2D getRotatedPosition(double distance, Vector2D position, double radius, double velocity, double time) {
			if (radius == 0) {
				return position;
			}
			Spline r = MathUtils.activeSpline(path, distance);
			double t = MathUtils.findT(path, distance);
			double angularVelocity = MathUtils.angularVelocity(velocity, radius);
			Vector2D newPosition = MathUtils.rotate(r, t, position, radius, angularVelocity * time);
			return newPosition;
		}
		
		/**
		 * Updates the Grapher object with the latest properties of the vehicle.  Used for visualization.
		 * 
		 * @param distance - the current distance traveled.
		 * @param position - the current calculated positon.
		 * @param radius - the current calculated radius.
		 */
		public void updateFeatures(double distance, Vector2D position, double radius) {
			Spline r = MathUtils.activeSpline(path, distance);
			double t = MathUtils.findT(path, distance);
			Vector2D center = MathUtils.centerOfCurvature(r, t, position, radius);
			Vector2D tangent = MathUtils.unitTangent(r, t);
			Vector2D normal = MathUtils.unitNormal(r, t);
			grapher.renderer.setPosition(position);
			grapher.renderer.setRotation(center, radius);
			grapher.renderer.setTangent(new Vector2D[] {
					r.r(t), new Vector2D(r.x(t) + tangent.getX(), r.y(t) + tangent.getY())
			});
			grapher.renderer.setNormal(new Vector2D[] {
					r.r(t), new Vector2D(r.x(t) + normal.getX(), r.y(t) + normal.getY())
			});
		}
		
		/**
		 * Whether or not the path has completed.
		 * 
		 * @param distance - the current distance traveled.
		 * @return - is finished.
		 */
		public boolean isFinished(double distance) {
			return MathUtils.activeSpline(path, distance) == null;
		}
		
		/**
		 * Represents left/right turning directions of a vehicle.
		 * @author Evan McCoy
		 *
		 */
		public enum TurningDirection {
			LEFT, RIGHT;
		}
		
		/**
		 * Records data into a CSV file for post-experiment data analysis.
		 * 
		 * @param position - the current vehicle position.
		 * @param distance - the current distance traveled.
		 * @param independent - the independent variable of the experiment.
		 */
		public void recordSimulationComplete(Vector2D position, double distance, double independent) {
			NaturalCubicSpline s = path[path.length - 1];
			Vector2D rExpected = s.r(1.0);
			double totalDistance = s.getStartingDistance() + s.getArcLength();
			double dispersion = Math.sqrt(Math.pow(position.getX()-rExpected.getX(), 2) + Math.pow(position.getY()-rExpected.getY(), 2));
			double percentDispersion = dispersion / totalDistance * 100;
			try {
				BufferedWriter writer = new BufferedWriter(new FileWriter("C:/Users/mccoy/Desktop/wheelAccel.csv", true));
				writer.write(independent + ", " + dispersion + ", " + totalDistance + ", " + percentDispersion + "\r\n");
				writer.close();
			} catch (IOException e) {e.printStackTrace();}
		}
	}
	
	public static void main(String[] args) {
		grapher = new Grapher(new Dimension(1000, 1000));
		grapher.renderer.setConstraints(new int[] {-10, 30}, new int[] {-10, 30});
		grapher.renderer.setPoints(points);
		
		Path path = new Path();
		NaturalCubicSpline[] splines = path.getPath();
		
		double divisions = 50.0;
		for (NaturalCubicSpline s : splines) {
			for (int i = 0;i < divisions;i++) {
				grapher.renderer.addLinePoint(new double[] {s.x(i / divisions), s.y(i / divisions)});
			}
		}
		
		for (double d = 0;d < splines[splines.length-1].getStartingDistance() + splines[splines.length-1].getArcLength();d+=0.25) {
			long startTime = Calendar.getInstance().getTimeInMillis();
			double t = MathUtils.findT(splines, d);
			long endTime = Calendar.getInstance().getTimeInMillis();
			try {
				BufferedWriter writer = new BufferedWriter(new FileWriter("C:/Users/mccoy/Desktop/findT.csv", true));
				writer.write(t + ", " + (endTime - startTime) + "\r\n");
				writer.close();
			} catch (IOException e) {e.printStackTrace();}
		}
	}
}
