package me.evanmccoy.pathfinding.path.natural;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.RealVector;

import me.evanmccoy.pathfinding.path.Interpolator;
import me.evanmccoy.pathfinding.path.Point;

/**
 * A class which interpolates cubic splines to a set of control points.
 * Uses the Apache Commons math3 library for matrix computations.
 * Created - February 2018
 * 
 * @author Evan McCoy
 * 
 */
public class NaturalCubicInterpolator extends Interpolator<Point, NaturalCubicSpline> {
	private double s0;
	private double s1;
	private double[] ref;
	
	private double[][] m;
	private double rhs[];
	
	/**
	 * Builds an Interpolator object with the needed inputs to solve.
	 * 
	 * @param points - the control points as an array of [x, y] coordinates
	 * @param startSlope - the slope (dy/dx) to fit to the beginning of the domain
	 * @param endSlope - the slope (dy/dx) to fit to the end of the domain
	 */
	public NaturalCubicInterpolator(Point[] points, double startSlope, double endSlope) {
		super(points);
		ref = new double[points.length];
		for (int i = 0;i < points.length;i++) {
			ref[i] = points[i].y;
		}
		this.s0 = startSlope;
		this.s1 = endSlope;
	}
	
	public NaturalCubicSpline[] calculate() {
		NaturalCubicSpline[] splines = new NaturalCubicSpline[points.length - 1];
		
		buildMatricies(points);
		DecompositionSolver solver = new LUDecomposition(new Array2DRowRealMatrix(m)).getSolver();
		RealVector coeff = solver.solve(new ArrayRealVector(rhs));
		for (int i = 0; i < splines.length;i++) {
			double x1 = points[i].x;
			double x2 = points[i+1].x;
			
			double a = coeff.getEntry(i*4);
			double b = coeff.getEntry(i*4 + 1);
			double c = coeff.getEntry(i*4 + 2);
			double d = coeff.getEntry(i*4 + 3);
			if (i == 0) {
				splines[i] = new NaturalCubicSpline(x1, x2, a, b, c, d, 0);
			}
			else {
				splines[i] = new NaturalCubicSpline(x1, x2, a, b, c, d, splines[i-1].getStartingDistance() + splines[i-1].getArcLength());
			}
		}
		
		return splines;
	}
	
	protected void buildMatricies(Point... points) {
		int functionCount = points.length - 1;
		
		//Create a matrix with rows for a, b, c, and d terms for all functions.
		m = new double[functionCount*4][];
		rhs = new double[functionCount*4];
		
		int row = 0;
		for (int i = 0;i < functionCount;i++) {
			//The range of x values in this spline.
			double deltaX = points[i + 1].x - points[i].x;
			//x'(t) for this spline.
			double dxdt = deltaX;
			double t;
			
			//Fit to starting slope s0 where t=0 on the first spline.
			if (i == 0) {
				t = 0;
				fitSlope(row, i, t, dxdt, s0);
				row++;
			}
			
			//Fit to first point - y(0) = y1 = y[i]
			t = 0;
			double y1 = ref[i];
			fitPoint(row, i, t, y1);
			row++;
			
			//Fit to second point - y(1) = y2 = y[i + 1]
			t = 1;
			double y2 = ref[i + 1];
			fitPoint(row, i, t, y2);
			row++;
			
			//Fit to starting slope s1 where t=1 on the last spline.
			if (i == functionCount - 1) {
				t = 1;
				fitSlope(row, i, t, dxdt, s1);
				row++;
			}
			else {
				//1 / x'(t) for the next spline. (Used in slope and curvature matching)
				double dtdxP1 = 1 / (points[i + 2].x - points[i + 1].x);
				//Match slopes of spline endpoints where dx/dt = x'(t) for spline 1 and dt/dx = 1 / x'(t) for spline 2
				matchdydx(row, i, dxdt, dtdxP1);
				row++;
				//Match curvature of spline endpoints where dx/dt = x'(t) for spline 1 and dt/dx = 1 / x'(t) for spline 2
				matchd2ydx2(row, i, dxdt, dtdxP1);
				row++;
			}
		}
	}
	
	/**
	 * Populates a row of the square matrix to fit the current spline to one of its endpoints (t=0 and t=1).
	 * 
	 * Given a point (t, y) = (0, y) -- the starting or ending point in the spline -- we want that
	 * y(t) = a + bt + ct^2 + dt^3 = y
	 * 
	 * This provides:
	 * 		rhs[r] = y
	 * 		m[r][i*4] through m[r][i*4 + 3] = [1 + 1*t + 1*t^2 + 1*t^3] = [1, t, t, t]
	 *
	 */
	public void fitPoint(int r, int i, double t, double y) {
		int functionCount = points.length - 1;
		rhs[r] = y;
		m[r] = new double[functionCount*4];
		for (int n = 0;n < functionCount;n++) {
			if (i == n) {
				m[r][n*4] = 1;
				m[r][n*4 + 1] = t;
				m[r][n*4 + 2] = t;
				m[r][n*4 + 3] = t;
			}
			else {
				m[r][n*4] = 0;
				m[r][n*4 + 1] = 0;
				m[r][n*4 + 2] = 0;
				m[r][n*4 + 3] = 0;
			}
		}
	}
	
	/**
	 * Populates a row of the square matrix to match the slopes of two neighbor splines.
	 * 
	 * Given a point (x, y) = (0, y) and (1, y) for (t, y) of splines f1 and f2, we want that
	 * f1'(1) = f2'(0) = dy/dx = (dy/dt)/(dx/dt)
	 * 
	 * Thus, (dy1/dt)*(dt1/dx)|t=1 - (dy2/dt)*(dt2/dx)|t=0 = 0 and
	 * 		(dy1/dt)|t=1 - (dy2/dt)|t=0*(dx1/dt)*(dt2/dx) = 0
	 * 
	 * This provides:
	 * 		rhs[r] = 0
	 * 		m[r][i*4] through m[r][i*4 + 3] = [0, 1, 2*t, 3*t^2] = [0, 1, 2, 3]
	 * 		m[r][i*4 + 4] through m[r][i*4 + 7] = [0, -(dxdt1 * dtdx2), -2*t, -3*t^2] = [0, -(dxdt1 * dtdx2), 0, 0]
	 */
	public void matchdydx(int r, int i, double dxdt1, double dtdx2) {
		int functionCount = points.length - 1;
		rhs[r] = 0;
		m[r] = new double[functionCount*4];
		for (int n = 0;n < functionCount;n++) {
			if (i == n) {
				m[r][n*4 + 1] = 1;
				m[r][n*4 + 2] = 2;
				m[r][n*4 + 3] = 3;
			}
			else if (i + 1 == n) {
				m[r][n*4 + 1] = -(dxdt1 * dtdx2);
				m[r][n*4 + 2] = 0;
				m[r][n*4 + 3] = 0;
			}
			else {
				m[r][n*4 + 1] = 0;
				m[r][n*4 + 2] = 0;
				m[r][n*4 + 3] = 0;
			}
			m[r][n*4] = 0;
		}
	}
	
	/**
	 * Populates a row of the square matrix to match the curvatures of two neighbor splines.
	 * 
	 * Given a point (x, y) = (0, y) and (1, y) for (t, y) of splines f1 and f2, we want that
	 * f1''(1) = f2''(0) = d2y/dx2 = (d^2y/dt^2)/(dx/dt)^2
	 * 
	 * Thus, (d^2y1/dt^2)(dt1/dx)^2|t=1 - (d^2y2/dt^2)*(dt2/dx)^2|t=0 = 0 and
	 * 		(d^2y1/dt^2)|t=1 - (d^2y2/dt^2)|t=0*(dx1/dt)^2*(dt2/dx)^2 = 0
	 * 
	 * This provides:
	 * 		rhs[r] = 0
	 * 		m[r][i*4] through m[r][i*4 + 3] = [0, 0, 2, 6*t] = [0, 0, 2, 6]
	 * 		m[r][i*4 + 4] through m[r][i*4 + 7] = [0, 0, -2((dxdt1)^2 * (dtdx2)^2), -6*t^2] = [0, 0, -2((dxdt1)^2 * (dtdx2)^2), 0]
	 */
	public void matchd2ydx2(int r, int i, double dxdt1, double dtdx2) {
		int functionCount = points.length - 1;
		rhs[r] = 0;
		m[r] = new double[functionCount*4];
		for (int n = 0;n < functionCount;n++) {
			if (i == n) {
				m[r][n*4 + 2] = 2;
				m[r][n*4 + 3] = 6;
			}
			else if (i + 1 == n) {
				m[r][n*4 + 2] = -2*(pow(dxdt1, 2) * pow(dtdx2, 2));
				m[r][n*4 + 3] = 0;
			}
			else {
				m[r][n*4 + 2] = 0;
				m[r][n*4 + 3] = 0;
			}
			m[r][n*4] = 0;
			m[r][n*4 + 1] = 0;
		}
	}
	
	/*
	 * Populates a row of the square matrix to fit the endpoints of the spline domain with a user-defined slope.
	 * 
	 * Given an endpoint slope, we want that (dy/dt)/(dx/dt) = dy/dx = s at t of the first or last spline.
	 * Thus, dy/dt = b + 2ct + 3dt^2 = s * dx/dt
	 * 
	 * This provides:
	 * 		rhs[r] = s * dxdt
	 * 		m[r][i*4] through m[i*4 + 3][i*4 + 3] = [0, 1, 2*t, 3*t^2] = [0, 1, 2*t, 3*t]
	 */
	public void fitSlope(int r, int i, double t, double dxdt, double s) {
		int functionCount = points.length - 1;
		rhs[r] = s * dxdt;
		m[r] = new double[functionCount*4];
		for (int n = 0;n < functionCount;n++) {
			if (i == n) {
				m[r][n*4 + 1] = 1;
				m[r][n*4 + 2] = 2*t;
				m[r][n*4 + 3] = 3*t;
			}
			else {
				m[r][n*4 + 1] = 0;
				m[r][n*4 + 2] = 0;
				m[r][n*4 + 3] = 0;
			}
			m[r][n*4] = 0;
		}
	}
}
