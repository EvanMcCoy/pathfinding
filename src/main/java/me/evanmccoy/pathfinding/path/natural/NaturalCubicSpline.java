package me.evanmccoy.pathfinding.path.natural;

import me.evanmccoy.pathfinding.MathUtils;
import me.evanmccoy.pathfinding.path.Spline;

/**
 * Represents a cubic spline parameterized by t.
 * Created - February 2018
 * 
 * @author Evan McCoy
 */
public class NaturalCubicSpline extends Spline {
	
	public NaturalCubicSpline(double x1, double x2, double a, double b, double c, double d, double startingDistance) {
		double deltaX = x2 - x1;
		x = (t) -> {
			return deltaX * t + x1;};
		dxdt = (t) -> {return deltaX;};
		dtdx = (t) -> {return 1 / deltaX;};
		d2xdt2 = (t) -> {return 0.0;};
		
		y = (t) -> {return a + b*t + c*pow(t, 2) + d*pow(t, 3);};
		dydt = (t) -> {return b + 2*c*t + 3*d*pow(t, 2);};
		d2ydt2 = (t) -> {return 2*c + 6*d*t;};
		
		arcLength = MathUtils.arcLength(this, 0, 1);
		this.startingDistance = startingDistance;
		
	}
	
}
