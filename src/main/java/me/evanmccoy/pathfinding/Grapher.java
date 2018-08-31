package me.evanmccoy.pathfinding;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JFrame;
import javax.swing.JPanel;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

/**
 * A class that controls the visualization of functions and points.
 * Created - February 2018
 * 
 * @author Evan McCoy
 *
 */
public class Grapher {
	public Dimension dimensions;
	public GraphRenderer renderer;

	/**
	 * Build an active, periodically updating, window displaying a graph of functions and points.
	 * 
	 * @param dimensions - the screen dimensions of the window.
	 */
	public Grapher(Dimension dimensions) {
		this.dimensions = dimensions;
		JFrame frame = new JFrame();
		frame.setBounds(0, 0, dimensions.width, dimensions.height+30);
		
		renderer = new GraphRenderer(dimensions.width, dimensions.height, this);
		renderer.setConstraints(new int[] {-10, 10}, new int[] {-10, 30});
		
		frame.setContentPane(renderer);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setVisible(true);
		
		Timer timer = new Timer();
		timer.scheduleAtFixedRate(new TimerTask() {
			public void run() {
				renderer.update(renderer.getGraphics());
			}
		}, 0, 10);
	}
	
	/**
	 * A class to control the rendering actions of the Grapher class.
	 * 
	 * @author Evan McCoy
	 *
	 */
	@SuppressWarnings("serial")
	public static class GraphRenderer extends JPanel {
		private Grapher parent;
		private int[] xConstraints;
		private int[] yConstraints;
		public List<Double> xVal = new ArrayList<Double>();
		public List<Double> yVal = new ArrayList<Double>();
		public List<Double> xValLine = new ArrayList<Double>();
		public List<Double> yValLine = new ArrayList<Double>();
		public Vehicle vehicle;
		public Vector2D position = new Vector2D(0, 0);
		public Vector2D center = new Vector2D(0, 0);
		public Vector2D[] tangent = new Vector2D[] {new Vector2D(0, 0), new Vector2D(0, 0)};
		public Vector2D[] normal = new Vector2D[] {new Vector2D(0, 0), new Vector2D(0, 0)};
		public double radius = 0;
		
		/**
		 * Builds a renderer for a graph with defined parameters.
		 * 
		 * @param width - the width of the window in pixels.
		 * @param height - the height of the window in pixels.
		 * @param parent - the parent Grapher object.
		 */
		public GraphRenderer(int width, int height, Grapher parent) {
			this.setPreferredSize(new Dimension(width, height));
			this.parent = parent;
		}
		
		/**
		 * Renders graph components in their current state.
		 * 
		 * @param g - the graphics object of the JPanel.
		 */
		public void paintComponent(Graphics graphics) {
			BufferedImage img = new BufferedImage(this.getWidth(), this.getHeight(), BufferedImage.TYPE_INT_ARGB);
			Graphics g = img.getGraphics();
			
			g.clearRect(0, 0, parent.dimensions.width, parent.dimensions.height);
			//Draw control points
			g.setColor(Color.BLUE);
			for (int i = 0;i < xVal.size();i++) {
				double x1 = xVal.get(i);
				double y1 = (yVal.size() > i) ? yVal.get(i) : 0;
				int[] c1 = getPixelCoordinate(new Vector2D(x1, y1));
				g.drawOval(c1[0]-3, c1[1]-3, 6, 6);
			}
			
			//Draw vehicles
			g.setColor(Color.GREEN);
			if (vehicle != null) { 
				int[] c = getPixelCoordinate(vehicle.position());
				g.fillOval(c[0]-3, c[1]-3, 6, 6);
			}
			int[] c = getPixelCoordinate(center);
			int[] size = getScaledSize(new Vector2D(Math.abs(radius*2), Math.abs(radius*2)));
			g.setColor(Color.YELLOW);
			if (radius > 0) {
				g.setColor(Color.GREEN);
			}
			g.drawOval(c[0] - size[0]/2, c[1]-size[1]/2, size[0], size[1]);
			g.fillOval(c[0] - 2, c[1]-2, 4, 4);
			
			//Draw connected functions
			g.setColor(Color.BLUE);
			for (int i = 0;i < xValLine.size() - 1;i++) {
				double x1 = xValLine.get(i);
				double x2 = xValLine.get(i + 1);
				double y1 = (yValLine.size() > i) ? yValLine.get(i) : 0;
				double y2 = (yValLine.size() > i + 1) ? yValLine.get(i + 1) : 0;
				int[] c1 = getPixelCoordinate(new Vector2D(x1, y1));
				int[] c2 = getPixelCoordinate(new Vector2D(x2, y2));
				g.drawLine(c1[0], c1[1], c2[0], c2[1]);
			}
			
			//Draw tangent line
			int[] c1 = getPixelCoordinate(tangent[0]);
			int[] c2 = getPixelCoordinate(tangent[1]);
			g.setColor(Color.PINK);
			g.drawLine(c1[0], c1[1], c2[0], c2[1]);
			
			//Draw normal line
			c1 = getPixelCoordinate(normal[0]);
			c2 = getPixelCoordinate(normal[1]);
			g.setColor(Color.PINK);
			g.drawLine(c1[0], c1[1], c2[0], c2[1]);
			
			//Draw axes
			g.setColor(Color.LIGHT_GRAY);
			g.drawLine(parent.dimensions.width/2, 0, parent.dimensions.width/2, parent.dimensions.height);
			g.drawLine(0, parent.dimensions.height/2, parent.dimensions.width, parent.dimensions.height/2);
			
			graphics.drawImage(img, 0, 0, this);
		}
		
		/**
		 * Sets the coordinate constraints of the rendered window.
		 * 
		 * @param xConstraints - the minimum and maximum x values.
		 * @param yConstraints - the minimum and maximum y values.
		 */
		public void setConstraints(int[] xConstraints, int[] yConstraints) {
			this.xConstraints = xConstraints;
			this.yConstraints = yConstraints;
		}
		
		/**
		 * Adds a set of points to the graph's rendering.  These points are independent and not connected.
		 * 
		 * @param points - an array of coordinate pairs.
		 */
		public void setPoints(double[][] points) {
			for (int i = 0;i < points.length;i++) {
				xVal.add(points[i][0]);
				yVal.add(points[i][1]);
			}
		}
		
		/**
		 * Add a point that will be connected by a line to the last point in the current list when rendered.
		 * 
		 * @param point - the coordinate pair.
		 */
		public void addLinePoint(double[] point) {
			xValLine.add(point[0]);
			yValLine.add(point[1]);
		}
		
		/**
		 * Sets the position of the vehicle on the graph.
		 * 
		 * @param position - the 2D position of the vehicle.
		 */
		public void setPosition(Vector2D position) {
			this.position = position;
		}
		
		/**
		 * Sets the parameters of the circle of rotation of the vehicle.
		 * 
		 * @param center - the 2D position of the center of rotation.
		 * @param radius - the radius of curvature.
		 */
		public void setRotation(Vector2D center, double radius) {
			this.center = center;
			this.radius = radius;
		}
		
		/**
		 * Sets the tangent vector.
		 * 
		 * @param tangent - the tangent vector.
		 */
		public void setTangent(Vector2D[] tangent) {
			this.tangent = tangent;
		}
		
		/**
		 * Sets the normal vector.
		 * 
		 * @param normal - the normal vector.
		 */
		public void setNormal(Vector2D[] normal) {
			this.normal = normal;
		}
		
		/**
		 * Convert x and y cartesian coordinates to a pixel location on the window.
		 * 
		 * @param coordinate - the input coordinate pair
		 * @return - the pixel coordinate pair
		 */
		public int[] getPixelCoordinate(Vector2D coordinate) {
			//If the coordinate is out of the x or y constriant, force to 0.
			double x = (coordinate.getX() >= xConstraints[0] && coordinate.getX() <= xConstraints[1]) ? coordinate.getX() : coordinate.getX();
			double y = (coordinate.getY() >= yConstraints[0] && coordinate.getY() <= xConstraints[1]) ? coordinate.getY() : coordinate.getY();
			int width = parent.dimensions.width;
			int height = parent.dimensions.height;
			double xPercent = (x - xConstraints[0]) / (xConstraints[1] - xConstraints[0]);
			double yPercent = (y - yConstraints[0]) / (yConstraints[1] - yConstraints[0]);
			
			int xPix = (int)(width * xPercent);
			int yPix = (int)(height - yPercent * height);
			//return new int[] {Math.max(0, xPix), Math.max(0, yPix)};
			return new int[] {xPix, yPix};
		}
		
		/**
		 * Returns dimensions scaled from x/y size to pixel size.
		 * 
		 * @param widthheight - a 2D vector containing width and height in x/y units.
		 * @return - the scaled size in pixels.
		 */
		public int[] getScaledSize(Vector2D widthheight) {
			int width = parent.dimensions.width;
			int height = parent.dimensions.height;
			double xPercent = widthheight.getX() / (xConstraints[1] - xConstraints[0]);
			double yPercent = widthheight.getY()/ (yConstraints[1] - yConstraints[0]);
			
			return new int[] {(int)(width * xPercent), (int)(height * yPercent)};
		}
		
		/**
		 * Resets the function display of the graph.
		 */
		public void reset() {
			xVal = new ArrayList<Double>();
			yVal = new ArrayList<Double>();
			xValLine = new ArrayList<Double>();
			yValLine = new ArrayList<Double>();
		}
	}
}
