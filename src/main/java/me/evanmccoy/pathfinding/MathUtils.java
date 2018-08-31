package me.evanmccoy.pathfinding;

import java.util.function.Function;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.integration.SimpsonIntegrator;
import org.apache.commons.math3.analysis.integration.UnivariateIntegrator;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import me.evanmccoy.pathfinding.path.Spline;
import me.evanmccoy.pathfinding.path.hermite.HermiteSpline;
import me.evanmccoy.pathfinding.path.natural.NaturalCubicSpline;

/**
 * A calculation utilities file for common mathematical functions used in spline pathfinding.
 * Created - February 2018
 * 
 * @author Evan McCoy
 *
 */
public class MathUtils {

	/**
	 * Builds a function useful for calculating the differential of the spline magnitude with respect to a t parameter.
	 * 
	 * dr/dt = sqrt(  (dx/dt)^2 + (dy/dt)^2  )
	 * 
	 * @param r - the spline to analyze.
	 * @return - a lambda function for finding r'(t).
	 */
	public static Function<Double, Double> drdt(Spline r) {
		return (t) -> {
			return Math.sqrt(  pow(r.dxdt(t), 2) + pow(r.dydt(t), 2)  );
		};
	}
	
	/**
	 * Builds a function for calculating the gradient of the spline at any value t.
	 * 
	 * gradient = dy/dx = (dy/dt)*(dt/dx)
	 * 
	 * @param r - the spline.
	 * @return - a lambda function for finding y'(x).
	 */
	public static Function<Double, Double> dydx(Spline r) {
		return (t) -> {
			return r.dydt(t)*r.dtdx(t);
		};
	}
	
	/**
	 * Builds a function for calculating the rate of change of the gradient at any value t.
	 * 
	 * Dgradient = d2y/dx2 = (d2y/dt2)*(dt/dx)^2
	 * 
	 * @param r - the spline.
	 * @return - a lambda function for finding y''(x).
	 */
	public static Function<Double, Double> d2ydx2(Spline r) {
		return (t) -> {
			return r.d2ydt2(t)*pow(r.dtdx(t), 2);
		};
	}
	
	/**
	 * Finds the arc length of a spline between two points using Simpson integration.
	 * 
	 * @param r - the spline to analyze
	 * @param start - the starting t value.
	 * @param end - the end t value.
	 * @return - the arc length
	 */
	public static double arcLength(Spline r, double start, double end) {
		UnivariateIntegrator integrator = new SimpsonIntegrator(9, 10);
		return integrator.integrate(Integer.MAX_VALUE, new UnivariateFunction() {
			@Override
			public double value(double t) {
				return Math.abs(drdt(r).apply(t));
			}
		}, start, end);
	}
	
	/**
	 * Builds a function for finding the curvature of spline r at a given point t.
	 * 
	 * k = (  (dx/dt)(d2y/dt2) - (dy/dt)(d2x/dt2)  ) / (  (ds/dt)^3  )
	 * 
	 * @param r - the spline
	 * @return - the curvature
	 */
	public static Function<Double, Double> k(Spline r) {
		return (t) -> {
			return (  r.dxdt(t)*r.d2ydt2(t) - r.dydt(t)*r.d2xdt2(t)  ) / (  pow(drdt(r).apply(t), 3)  );
		};
	}
	
	/**
	 * Returns the osculating radius for a spline at a given point t.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @return - the radius
	 */
	public static double osculatingRadius(Spline r, double t) {
		return 1 / k(r).apply(t);
	}
	
	/**
	 * Returns the 2D Euclidean coordinate of the center of curvature for any value t.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @return - the center of curvature
	 */
	public static Vector2D centerOfCurvature(Spline r, double t) {
		Vector2D position = new Vector2D(r.x(t), r.y(t));
		Vector2D unitNormal = unitNormal(r, t);
		double radius = Math.abs(osculatingRadius(r, t));
		
		position = position.add(unitNormal.scalarMultiply(radius));

		return new  Vector2D(position.getX(), position.getY());
	}
	
	/**
	 * Returns the 2D Euclidean coordinate of the center of curvature for any value t and a manually provided curve position.
	 * 
	 * This implementation is mainly used for computer simulated paths to account for wheel-acceleration-induced error.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @param position - the current position
	 * @param radius - the current radius based on measured velocities
	 * @return - the center of curvature
	 */
	public static Vector2D centerOfCurvature(Spline r, double t, Vector2D position, double radius) {
		Vector2D unitNormal = unitNormal(r, t);
		Vector2D scaledNormal = unitNormal.scalarMultiply(radius);
		position = new Vector2D(position.getX() + scaledNormal.getX(), position.getY() + scaledNormal.getY());
		return position;
	}
	
	/**
	 * Returns the tangent vector of the spline r at value t.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @return - the tangent vector
	 */
	public static Vector2D tangent(Spline r, double t) {
		return new Vector2D(r.dxdt(t), r.dydt(t));
	}
	
	/**
	 * Returns the normalized tangent vector of the spline r at value t.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @return - the unit tangent vector
	 */
	public static Vector2D unitTangent(Spline r, double t) {
		return tangent(r, t).normalize();
	}
	
	/**
	 * Returns the normal vector of the spline r at value t.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @return - the normal vector
	 */
	public static Vector2D normal(Spline r, double t) {
		Vector2D tangent = tangent(r, t);
		Vector2D normal;
		double k = k(r).apply(t);
		double direction = k / Math.abs(k);
		normal = new Vector2D(-direction * tangent.getY(), direction * tangent.getX());
		return normal;
	}
	
	/**
	 * Returns the normalized normal vector of the spline r at value t.
	 * 
	 * @param r - the spline
	 * @param t - the t parameter
	 * @return - the unit normal vector
	 */
	public static Vector2D unitNormal(Spline r, double t) {
		return normal(r, t).normalize();
	}
	
	/**
	 * The angular velocity in degrees per second of a body given its velocity around the arc and radius of curvature.
	 * 
	 * @param velocity - the velocity
	 * @param radius - the radius of curvature
	 * @return - the angular velocity in radians per second
	 */
	public static double angularVelocity(double velocity, double radius) {
		return velocity / Math.abs(radius);
	}
	
	/**
	 * Rotates a given position around the current circle of curvature by deltaTheta degrees.
	 * 
	 * This implementation is used for computer simulated paths to recalculate position based on movement parameters.
	 * 
	 * Where C is the center, P is the current position, and dTheta is the rotation in radians:
	 * x = (Px - Cx)cos(dTheta) - (Py - Cy)sin(dTheta)
	 * y = (Px - Cx)sin(dTheta) + (Py - Cy)cos(dTheta)
	 * 
	 * @param r - the active spline
	 * @param t - the t parameter
	 * @param position - the current position vector
	 * @param radius - the current radius based on measured velocity
	 * @param deltaTheta - the change in angle of the rotation
	 * @return
	 */
	public static Vector2D rotate(Spline r, double t, Vector2D position, double radius, double deltaTheta) {
		Vector2D center = centerOfCurvature(r, t, position, radius);
		double k = k(r).apply(t);
		if (k < 0) {
			deltaTheta *= -1;
		}
		double xComp = position.getX() - center.getX();
		double yComp = position.getY() - center.getY();
		double x = xComp*Math.cos(deltaTheta) - yComp*Math.sin(deltaTheta) + center.getX();
		double y = xComp*Math.sin(deltaTheta) + yComp*Math.cos(deltaTheta) + center.getY();
		return new Vector2D(x, y);
	}
	
	/**
	 * Returns the base risen to the exponent.
	 * 
	 * @param base - the base.
	 * @param exponent - the exponent.
	 * @return - the value.
	 */
	public static double pow(double base, double exponent) {
		return Math.pow(base, exponent);
	}
	
	/**
	 * Returns the spline the defined arc distance lies on. Returns the first spline if no valid spline is found.
	 * 
	 * @param splines - the array of splines.
	 * @param distance - the arc distance.
	 * @return - the active spline.
	 */
	public static Spline activeSpline(Spline[] splines, double distance) {
		for (Spline s : splines) {
			if (s.getStartingDistance() <= distance && s.getStartingDistance() + s.getArcLength() > distance) {
				return s;
			}
		}
		return null;
	}
	
	/**
	 * Finds an approximated value of t given a collection of splines and the distance traveled.
	 * 
	 * @return - the t value
	 */
	public static double findT(Spline[] splines, double distance) {
		Spline s = activeSpline(splines, distance);
		if (s == null) {
			return 1.0;
		}
		distance -= s.getStartingDistance();
	    if (distance < 0) {
	        return -1;
	    }
	    int intervals = 15;
	    double[] t = {0.25, 0.75};
	    int approximationIndex = 0;
	    for (int i = 0;i < intervals;i++) {
	        double low = arcLength(s, 0, t[0]);
	        double high = arcLength(s, 0, t[1]);
	        if (Math.abs(distance - low) < Math.abs(distance - high)) {
	            if (i != intervals - 1) {
	                t = new double[] {t[0] - pow(0.5, (i+3)), t[0] + pow(0.5, (i+3))};
	            }
	        }
	        else {
	            if (i == intervals - 1) {
	                approximationIndex = 1;
	            }
	            else {
	                t = new double[] {t[1] - pow(0.5, (i+3)), t[1] + pow(0.5, (i+3))};
	            }
	        }
	    }
	    return t[approximationIndex];
	}
}
