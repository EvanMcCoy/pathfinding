package me.evanmccoy.pathfinding.path;

public abstract class Interpolator<T extends Point, U extends Spline> {
	protected T[] points;
	
	public Interpolator(T[] points) {
		this.points = points;
	}
	
	public abstract U[] calculate();
	
	protected abstract void buildMatricies(T... points);
	
	public double pow(double base, double power) {
		return Math.pow(base, power);
	}
}
