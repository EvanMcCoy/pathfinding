package me.evanmccoy.pathfinding.path.hermite;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealMatrix;

import me.evanmccoy.pathfinding.path.Interpolator;

public class HermiteInterpolator extends Interpolator<HermitePoint, HermiteSpline> {
	private HermiteSpline[] splines;
	
	//This is constant for Hermite interpolation because the
	//calculation is always done with two points (one spline).
	private final double m[][] = new double[][] {
			{1, 0, 0, 0},
			{1, 1, 1, 1},
			{0, 1, 0, 0},
			{0, 1, 1, 1}
	};
	private double rhs[][];
	
	public HermiteInterpolator(HermitePoint[] points) {
		super(points);
	}
	
	public HermiteSpline[] calculate() {
		splines = new HermiteSpline[points.length - 1];
		for (int i = 0;i < splines.length;i++) {
			HermitePoint p1 = points[i];
			HermitePoint p2 = points[i + 1];
			buildMatricies(p1, p2);
			DecompositionSolver solver = new LUDecomposition(new Array2DRowRealMatrix(m)).getSolver();
			RealMatrix coeff = solver.solve(new Array2DRowRealMatrix(rhs));
			
			splines[i] = new HermiteSpline(p1, p2, coeff.getEntry(0, 0),
					coeff.getEntry(1, 0),
					coeff.getEntry(2, 0),
					coeff.getEntry(3, 0),
					coeff.getEntry(0, 1),
					coeff.getEntry(1, 1),
					coeff.getEntry(2, 1),
					coeff.getEntry(3, 1),
					(i == 0) ? 0 : splines[i-1].getStartingDistance() + splines[i-1].getArcLength());
		}
		
		return splines;
	}
	
	/**
	 * Adds a point to this interpolation.
	 * 
	 * @param point - the new point.
	 * @return - the newly created spline.
	 */
	public HermiteSpline addPoint(HermitePoint point) {
		HermitePoint[] tempPoints = new HermitePoint[points.length + 1];
		for (int i = 0;i < points.length;i++) {
			tempPoints[i] = points[i];
		}
		tempPoints[points.length] = point;
		points = tempPoints;
		
		int lenPoints = points.length;
		//A spline can only be formed with two or more points available.
		if (lenPoints >= 2) {
			//Add new point to point array.
			HermitePoint p1 = points[lenPoints - 2];
			HermitePoint p2 = points[lenPoints - 1];
			buildMatricies(p1, p2);
			DecompositionSolver solver = new LUDecomposition(new Array2DRowRealMatrix(m)).getSolver();
			RealMatrix coeff = solver.solve(new Array2DRowRealMatrix(rhs));
			
			int lenSplines = splines.length;
			HermiteSpline newSpline =  new HermiteSpline(p1, p2, coeff.getEntry(0, 0),
					coeff.getEntry(1, 0),
					coeff.getEntry(2, 0),
					coeff.getEntry(3, 0),
					coeff.getEntry(0, 1),
					coeff.getEntry(1, 1),
					coeff.getEntry(2, 1),
					coeff.getEntry(3, 1),
					(lenSplines > 0) ? splines[lenSplines-1].getStartingDistance() + splines[lenSplines-1].getArcLength() : 0);
			
			//Add new spline to spline array.
			HermiteSpline[] tempSplines = new HermiteSpline[splines.length + 1];
			for (int i = 0;i < splines.length;i++) {
				tempSplines[i] = splines[i];
			}
			tempSplines[splines.length] = newSpline;
			splines = tempSplines;
			
			return newSpline;
		}
		return null;
	}
	
	protected void buildMatricies(HermitePoint... points) {
		if (points.length < 2) {
			return;
		}
		HermitePoint p1 = points[0];
		HermitePoint p2 = points[1];
		
		rhs = new double[][] {
			{p1.x, p1.y},
			{p2.x, p2.y},
			//Expecting slopes as change in Y per one unit X.
			{1, p1.m},
			{1, p2.m}
		};
	}
	
}
