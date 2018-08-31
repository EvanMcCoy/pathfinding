package me.evanmccoy.pathfinding.path;

import java.util.function.Function;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class Spline {
	//Components
	protected Function<Double, Double> x;
	protected Function<Double, Double> y;
	
	//Component derivatives
	protected Function<Double, Double> dxdt;
	protected Function<Double, Double> dtdx;
	protected Function<Double, Double> dydt;
	
	//Component second derivatives
	protected Function<Double, Double> d2xdt2;
	protected Function<Double, Double> d2ydt2;
	
	//The length of this spline between t=0 and t=1
	protected double arcLength;
	//The total distance of the path at t=0 for this spline.
	protected double startingDistance;
	
	/**
	 * x(t)
	 * 
	 * @param t - the t parameter.
	 * @return - x(t) evaluated at t.
	 */
	public double x(double t) {
		return x.apply(t);
	}
	
	/**
	 * x'(t)
	 * 
	 * @param t - the t parameter.
	 * @return - x'(t) evaluated at t.
	 */
	public double dxdt(double t) {
		return dxdt.apply(t);
	}
	
	/**
	 * 1 / x'(t)
	 * 
	 * @param t - the t parameter.
	 * @return - 1 / x'(t) evaluated at t.
	 */
	public double dtdx(double t) {
		return dtdx.apply(t);
	}
	
	/**
	 * x''(t)
	 * 
	 * @param t - the t parameter.
	 * @return - x''(t) evaluated at t.
	 */
	public double d2xdt2(double t) {
		return d2xdt2.apply(t);
	}
	
	/**
	 * y(t)
	 * 
	 * @param t - the t parameter.
	 * @return - y(t) evaluated at t.
	 */
	public double y(double t) {
		return y.apply(t);
	}
	
	/**
	 * y'(t)
	 * 
	 * @param t - the t parameter.
	 * @return - y'(t) evaluated at t.
	 */
	public double dydt(double t) {
		return dydt.apply(t);
	}
	
	/**
	 * y''(t)
	 * 
	 * @param t - the t parameter.
	 * @return - y''(t) evaluated at t.
	 */
	public double d2ydt2(double t) {
		return d2ydt2.apply(t);
	}
	
	/**
	 * A vector-valued function, r(t), containing <x(t), y(t)>
	 * 
	 * @param t - the t parameter.
	 * @return - r(t) evaluated at t.
	 */
	public Vector2D r(double t) {
		return new Vector2D(x(t), y(t));
	}
	
	/**
	 * The arc length of this spline section.
	 * 
	 * @return - the arc length.
	 */
	public double getArcLength() {
		return arcLength;
	}
	
	/**
	 * The overall path arc distance at the starting endpoint of this spline section.
	 * 
	 * @return - the starting distance.
	 */
	public double getStartingDistance() {
		return this.startingDistance;
	}
	
	/**
	 * Shorthand function for Math.pow(base, exponent)
	 * 
	 * @param base - the exponential base.
	 * @param power - the exponent.
	 * @return - the base risen to the power.
	 */
	public double pow(double base, double power) {
		return Math.pow(base, power);
	}
}
