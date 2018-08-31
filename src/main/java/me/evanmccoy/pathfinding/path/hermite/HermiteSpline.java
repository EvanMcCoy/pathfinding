package me.evanmccoy.pathfinding.path.hermite;

import me.evanmccoy.pathfinding.MathUtils;
import me.evanmccoy.pathfinding.path.Spline;

public class HermiteSpline extends Spline {
	private HermitePoint p1, p2;
	
	public HermiteSpline(HermitePoint p1, HermitePoint p2, 
			double ax, double bx, double cx, double dx, 
			double ay, double by, double cy, double dy, 
			double startingDistance) {
		this.p1 = p1;
		this.p2 = p2;
		
		x = (t) -> {
			return ax + bx*t + cx*pow(t, 2) + dx*pow(t, 3);
		};
		y = (t) -> {
			return ay + by*t + cy*pow(t, 2) + dy*pow(t, 3);
		};
		dxdt = (t) -> {
			return bx + 2*cx*t + 3*dx*pow(t, 2);
		};
		dydt = (t) -> {
			return by + 2*cy*t + 3*dy*pow(t, 2);
		};
		dtdx = (t) -> {
			return 1 / dxdt.apply(t);
		};
		
		arcLength = MathUtils.arcLength(this, 0, 1);
	}
	
	public HermitePoint getStartPoint() {
		return this.p1;
	}
	public HermitePoint getEndPoint() {
		return this.p2;
	}
}
