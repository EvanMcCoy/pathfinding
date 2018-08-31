package me.evanmccoy.pathfinding.path.hermite;

import me.evanmccoy.pathfinding.path.Point;

public class HermitePoint extends Point {
	public double m;
	
	public HermitePoint(double x, double y, double m) {
		super(x, y);
		this.m = m;
	}
}
