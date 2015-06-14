

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Stack;

import javax.swing.JComponent;

public class FloodfillCanvas extends JComponent{
	private static final long serialVersionUID = 1L;

	private int sleepDuration = 10;
	
	private int columns;
	private int rows;
	private int dotW;
	private int dotH;

	private LinkedList<Point> points = new LinkedList<Point>();
	private LinkedList<Line> lines = new LinkedList<Line>();
	private LinkedList<Circle> circles = new LinkedList<Circle>();
	private LinkedList<Point> fillPoints = new LinkedList<Point>();

	// Array that saves the dots that are set
	private boolean[][] dots;

	public FloodfillCanvas(int columns, int rows) {
		this.columns = columns;
		this.rows = rows;
		dots = new boolean[columns][rows];
		// setDoubleBuffered(true);
	}

	// Repaints the canvas
	public void paint(Graphics g) {
		// 1. Clears background
		int width = getWidth();
		int height = getHeight();

		g.clearRect(0, 0, width, height);

		// 2. Draws grid
		g.setColor(Color.black);
		
		drawGrid(g);

		// 3. Draws all stored points, calls drawDot(...)
		dotW = width / columns;
		dotH = height / rows;

		for (Point p : points) {
			drawDot(g, p.x, p.y);
		}

		for (Line l : lines) {
			drawLine(g, l);
		}

		for (Circle c : circles) {
			drawCircle(g, c);
		}

		for (Point p : fillPoints) {
			drawDot(g, p.x, p.y, Color.orange);
		}

	}

	// Adds a point to internal point list.
	public void addPoint(int gridX, int gridY) {
		points.add(new Point(gridX, gridY));
	}

	// Draws the grid. Calls drawGridDot for each grid point.
	private void drawGrid(Graphics g) {
		for (int i = 0; i < columns; i++) {
			for (int j = 0; j < rows; j++) {
				drawGridDot(g, i, j);
			}

		}

	}

	// Draws a single grid point.
	// Encapsulates the conversion of grid coordinates (e.g. 2/2) into pixel
	// coordinates (e.g. 125/125)
	private void drawGridDot(Graphics g, int gridX, int gridY) {
		int x = (int) (getWidth() * 0.5 / columns) + convertX(gridX);
		int y = (int) (getHeight() * 0.5 / rows) + convertY(gridY);

		g.fillOval(x, y, 2, 2);
	}

	// Draws a single point (heavy dot) onto the canvas.
	// Encapsulates the conversion of grid coordinates (e.g. 2/2) into pixel
	// coordinates (e.g. 125/125)
	private void drawDot(Graphics g, int gridX, int gridY) {
		drawDot(g, gridX, gridY, Color.black);
		dots[gridX][gridY] = true;
	}

	private void drawDot(Graphics g, int gridX, int gridY, Color c) {
		g.setColor(c);
		g.fillOval(convertX(gridX), convertY(gridY), dotW, dotH);
	}

	private int convertX(int gridX) {
		return getWidth() / columns * gridX;
	}

	private int convertY(int gridY) {
		return getHeight() / rows * gridY;
	}

	void addLine(Line l) {
		lines.add(l);

	}

	void drawLine(Graphics g, Line l) {
		int x1 = l.startPoint.x;
		int x2 = l.endPoint.x;

		int y1 = l.startPoint.y;
		int y2 = l.endPoint.y;

		// check validity
		if (x1 < 0 || y1 < 0 || x1 >= rows || y1 >= columns ||
			x2 < 0 || y2 < 0 || x2 >= rows || y2 >= columns) {
			System.out.println("Line could not be drawn (out of range)");
			return;
		}

		int dx = x2 - x1;
		int dy = y2 - y1;

		int multX = 1;
		// mirror the negative octants
		if (x2 < x1) {
			multX = -1;
			dx = -dx;
		}

		int multY = 1;
		if (y2 < y1) {
			multY = -1;
			dy = -dy;
		}

		// draw first dot
		drawDot(g, x1, y1);
			
		// calculate other dots
		if (Math.abs(dx) >= Math.abs(dy)) {
			// x-axis is the one to increase/decrease
			int e = 2 * dy - dx;
			int incrE = 2 * dy;
			int incrNE = 2 * (dy - dx);

			int y = y1;
			for (int x = x1 + multX; (x * multX) <= (x2 * multX); x += multX) {
				if (e <= 0) {
					e = e + incrE;
				} else {
					e = e + incrNE;
					y += multY;
				}
				drawDot(g, x, y);
			}
		} else {
			// y-axis is the one to increase/decrease
			int e = 2 * dx - dy;
			int incrE = 2 * dx;
			int incrNE = 2 * (dx - dy);
			int x = x1;
			for (int y = y1 + multY; (y * multY) <= (y2 * multY); y += multY) {
				if (e <= 0) {
					e = e + incrE;
				} else {
					e = e + incrNE;
					x += multX;
				}
				drawDot(g, x, y);
			}

		}

	}

	void addCircle(Circle c) {
		circles.add(c);

	}

	void drawCircle(Graphics g, Circle c) {

		int x = 0;
		int y = c.radius;
		double d = 5.0 / 4.0 - c.radius;
		drawCircleDot(g, c, x, y);
		drawCircleDot(g, c, x+1, y);

		while (y > x) {
			if (d < 0) {
				d += 2.0 * x + 3.0;
			} else {
				d += 2.0 * (x - y) + 5.0;
				y--;
			}
			x++;
			drawCircleDot(g, c, x, y);
			drawCircleDot(g, c, x+1, y);
		}

	}

	void drawCircleDot(Graphics g, Circle c, int x, int y) {
		// mirror the drawn coordinates to every octant
		drawDot(g, c.center.x + x, c.center.y + y);
		drawDot(g, c.center.x + x, c.center.y - y);
		drawDot(g, c.center.x - x, c.center.y + y);
		drawDot(g, c.center.x - x, c.center.y - y);
		drawDot(g, c.center.x + y, c.center.y + x);
		drawDot(g, c.center.x + y, c.center.y - x);
		drawDot(g, c.center.x - y, c.center.y - x);
		drawDot(g, c.center.x - y, c.center.y + x);
		
		drawDot(g, c.center.x + x+1, c.center.y + y);
		drawDot(g, c.center.x + x+1, c.center.y - y);
		drawDot(g, c.center.x - x-1, c.center.y + y);
		drawDot(g, c.center.x - x-1, c.center.y - y);
		drawDot(g, c.center.x + y+1, c.center.y + x);
		drawDot(g, c.center.x + y+1, c.center.y - x);
		drawDot(g, c.center.x - y-1, c.center.y - x);
		drawDot(g, c.center.x - y-1, c.center.y + x);
	}

	void addFillPoint(Point p) {
		fillPoints.add(p);
		dots[p.x][p.y] = true;
		try {
			Thread.sleep(sleepDuration);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	void floodFill_BreadthFirst(int x, int y) {
		LinkedList<Node> q = new LinkedList<Node>();
		q.addFirst(new Node(x, y));
		int maxQueueSize = 0;
		while (!q.isEmpty()) {
			Node n = (Node) q.removeLast();
			if ((n.x >= 0) && (n.x < columns) && (n.y>=0) && (n.y < rows) && (dots[n.x][n.y] == false)) {
				// putPixel
				addFillPoint(new Point(n.x, n.y));
				repaint();
				q.addFirst(new Node(n.x+1, n.y));
				q.addFirst(new Node(n.x, n.y+1));
				q.addFirst(new Node(n.x, n.y-1));
				q.addFirst(new Node(n.x-1, n.y));				
			}
			if (q.size() > maxQueueSize) {
				maxQueueSize = q.size();
			}
		}
		System.out.println("\n MAX QUEUE SIZE (breadth first): " + maxQueueSize);
	}
	
	void floodFill_DepthFirst(int x, int y) {
		Stack<Node> s = new Stack<Node>();
		s.add(new Node(x, y));
		int maxStackSize = 0;
		while (!s.isEmpty()) {
			Node n = (Node) s.pop();
			if ((n.x >= 0) && (n.x < columns) && (n.y>=0) && (n.y < rows) && (dots[n.x][n.y] == false)) {
				// putPixel
				addFillPoint(new Point(n.x, n.y));
				repaint();
				s.push(new Node(n.x+1, n.y));
				s.push(new Node(n.x, n.y+1));
				s.push(new Node(n.x, n.y-1));
				s.push(new Node(n.x-1, n.y));				
			}
			if (s.size() > maxStackSize) {
				maxStackSize = s.size();
			}
		}
		System.out.println("\n MAX STACK SIZE (depth first): " + maxStackSize);
	}

	static int stackCounterFloodfill = 0;
	static int maxStackCounterFloodfill = 0;
	
	void floodfill(int x, int y) {
		stackCounterFloodfill++;
		if (stackCounterFloodfill > maxStackCounterFloodfill) {
			maxStackCounterFloodfill = stackCounterFloodfill;
		}
		
		if (x >= 0 && y >= 0 && x < columns && y < rows) {
			if (dots[x][y] == true) {
				return;
			}
			addFillPoint(new Point(x, y));
			repaint();
			try {
				Thread.sleep(sleepDuration);
			} catch (InterruptedException ex) {
				ex.printStackTrace();
			}
			floodfill(x + 1, y);
			floodfill(x, y + 1);
			floodfill(x - 1, y);
			floodfill(x, y - 1);
			
			stackCounterFloodfill--;
		}
	}
	
	static int stackCounterLinefill = 0;
	static int maxStackCounterLinefill = 0;
	
	void linefill_codeProj(int x1, int x2, int y)
	{
		stackCounterLinefill++;
		if (stackCounterLinefill > maxStackCounterLinefill) {
			maxStackCounterLinefill = stackCounterLinefill;
		}
		
		int xL, xR;
		if (y < 0 || y >= rows || x1 < 0 || x1 >= columns || x2 < 0 || x2 >= columns) {
			return;
		}
		//scan left
		for (xL = x1; xL >= 0; --xL) {
			if (dots[xL][y] == true) {
				break;
			}
			else {
				addFillPoint(new Point(xL, y));
				repaint();
			}
		}
		//fill children
		if (xL < x1) {
			linefill_codeProj(xL, x1, y - 1);
			linefill_codeProj(xL, x1, y + 1);
			++x1;
		}
		
		//scan right
		for (xR = x2; xR < columns; ++xR) {
			if (dots[xR][y] == true) {
				break;
			}
			else {
				addFillPoint(new Point(xR, y));
				repaint();
			}
		}
		//fill children
		if (xR > x2) {
			linefill_codeProj(x2, xR, y - 1);
			linefill_codeProj(x2, xR, y + 1);
			--x2;
		}
		
		// alg for in between
		for (xR = x1; xR <= x2 && xR < columns; ++xR) {
			if (dots[xR][y] == false) {
				addFillPoint(new Point(xR, y));
				repaint();
			}
			else {
				if (x1 < xR) {
					//fill children
					linefill_codeProj(x1, xR-1, y-1);
					linefill_codeProj(x1, xR-1, y+1);
					x1 = xR;
				}
				
				for (; xR <= x2 && xR < columns; ++xR) {
					if (dots[xR][y] == false) {
						x1 = xR--;
						break;
					}
				}
			}
		}
		stackCounterLinefill--;
	}
	
	
	// ###################################################
	/*
	static FloodFillRangeQueue ranges = null;
	/// <summary>
	/// Fills the specified point on the bitmap with the currently selected 
	/// fill color.
	/// </summary>
	/// <param name="pt">The starting point for the fill.</param>
	public void QueueLinearFloodFill(int x, int y)
	{
	    //***Prepare for fill.
	    PrepareForFloodFill(pt);

	    ranges = new FloodFillRangeQueue(((columns+rows)/2)*5;
	    //***Get starting color.
	    int x = pt.X; int y = pt.Y;
	    int idx = CoordsToByteIndex(ref x, ref y);
	    startColor = new byte[] { bitmap.Bits[idx], bitmap.Bits[idx + 1], 
	                        bitmap.Bits[idx + 2] };


	    //***Do first call to floodfill.
	    LinearFill(ref x, ref y);

	    //***Call floodfill routine while floodfill ranges still exist 
	    //on the queue
	    while (ranges.Count > 0)
	    {
	        //**Get Next Range Off the Queue
	        FloodFillRange range = ranges.Dequeue();

	        //**Check Above and Below Each Pixel in the Floodfill Range
	        int downPxIdx = (bitmapWidth * (range.Y + 1)) + range.StartX;
	                    //CoordsToPixelIndex(lFillLoc,y+1);
	        int upPxIdx = (bitmapWidth * (range.Y - 1)) + range.StartX;
	                    //CoordsToPixelIndex(lFillLoc, y - 1);
	        int upY=range.Y - 1;//so we can pass the y coord by ref
	        int downY = range.Y + 1;
	        int tempIdx;
	        for (int i = range.StartX; i <= range.EndX; i++)
	        {
	            //*Start Fill Upwards
	            //if we're not above the top of the bitmap and the pixel 
	        //above this one is within the color tolerance
	            tempIdx = CoordsToByteIndex(ref i, ref upY);
	            if (range.Y > 0 && (!pixelsChecked[upPxIdx]) && 
	                    CheckPixel(ref tempIdx))
	                LinearFill(ref i, ref upY);

	            //*Start Fill Downwards
	            //if we're not below the bottom of the bitmap and 
	        //the pixel below this one is
	            //within the color tolerance
	            tempIdx = CoordsToByteIndex(ref i, ref downY);
	            if (range.Y < (bitmapHeight - 1) && (!pixelsChecked[downPxIdx]) 
	                        && CheckPixel(ref tempIdx))
	                LinearFill(ref i, ref downY);
	            downPxIdx++;
	            upPxIdx++;
	        }

	    }
	}

	/// <summary>
	/// Finds the furthermost left and right boundaries of the fill area
	/// on a given y coordinate, starting from a given x coordinate, 
	/// filling as it goes.
	/// Adds the resulting horizontal range to the queue of floodfill ranges,
	/// to be processed in the main loop.
	/// </summary>
	/// <param name="x">The x coordinate to start from.</param>
	/// <param name="y">The y coordinate to check at.</param>
	void LinearFill(ref int x, ref int y)
	{
	   //cache some bitmap and fill info in local variables for 
	   //a little extra speed
	   byte[] bitmapBits=this.bitmapBits;
	   bool[] pixelsChecked=this.pixelsChecked;
	   byte[] byteFillColor= this.byteFillColor;
	   int bitmapPixelFormatSize=this.bitmapPixelFormatSize;
	   int bitmapWidth=this.bitmapWidth;

	    //***Find Left Edge of Color Area
	    int lFillLoc = x; //the location to check/fill on the left
	    int idx = CoordsToByteIndex(ref x, ref y); 
	                //the byte index of the current location
	    int pxIdx = (bitmapWidth * y) + x;//CoordsToPixelIndex(x,y);
	    while (true)
	    {
	        //**fill with the color
	        bitmapBits[idx] = byteFillColor[0];
	        bitmapBits[idx+1] = byteFillColor[1];
	        bitmapBits[idx+2] = byteFillColor[2];
	        //**indicate that this pixel has already been checked and filled
	        pixelsChecked[pxIdx] = true;
	        //**screen update for 'slow' fill
	        if (slow) UpdateScreen(ref lFillLoc, ref y);
	        //**de-increment
	        lFillLoc--;     //de-increment counter
	        pxIdx--;        //de-increment pixel index
	        idx -= bitmapPixelFormatSize;//de-increment byte index
	        //**exit loop if we're at edge of bitmap or color area
	        if (lFillLoc <= 0 || (pixelsChecked[pxIdx]) || !CheckPixel(ref idx))
	            break;

	    }
	    lFillLoc++;

	    //***Find Right Edge of Color Area
	    int rFillLoc = x; //the location to check/fill on the left
	    idx = CoordsToByteIndex(ref x, ref y);
	    pxIdx = (bitmapWidth * y) + x;
	    while (true)
	    {
	        //**fill with the color
	        bitmapBits[idx] = byteFillColor[0];
	        bitmapBits[idx + 1] = byteFillColor[1];
	        bitmapBits[idx + 2] = byteFillColor[2];
	        //**indicate that this pixel has already been checked and filled
	        pixelsChecked[pxIdx] = true;
	        //**screen update for 'slow' fill
	        if (slow) UpdateScreen(ref rFillLoc, ref y);
	        //**increment
	        rFillLoc++;     //increment counter
	        pxIdx++;        //increment pixel index
	        idx += bitmapPixelFormatSize;//increment byte index
	        //**exit loop if we're at edge of bitmap or color area
	        if (rFillLoc >= bitmapWidth || pixelsChecked[pxIdx] || 
	                        !CheckPixel(ref idx))
	            break;

	    }
	    rFillLoc--;

	   //add range to queue
	   FloodFillRange r = new FloodFillRange(lFillLoc, rFillLoc, y);
	   ranges.Enqueue(ref r);
	}
	// ###################################################
	*/
	
	Stack<Point> stackLeft = new Stack<Point>();
	Stack<Point> stackRight = new Stack<Point>();
	void linefill_own(int x, int y) {
		//out of bounds
		if (y < 0 || y > rows) {
			return;
		}
		
		//boundary or already visited
		if (dots[x][y] == true) {
			return;
		}
		
		//Point seed = new Point(x, y);
	}
}
