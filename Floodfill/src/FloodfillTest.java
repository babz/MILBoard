

import java.awt.Point;

import javax.swing.JFrame;

public class FloodfillTest {
	public static void main(String[] args) {
		//    JFrame frame = new JFrame("Floodfill");
		//    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		//    frame.setSize(470, 490);
		//    FloodfillCanvas floodFillTest = new FloodfillCanvas(50, 50);

		//    floodFillTest.addPoint(2,2);
		//    floodFillTest.addPoint(2,3);
		//    floodFillTest.addPoint(3,2);
		//    floodFillTest.addPoint(3,3);
		//    floodFillTest.addPoint(0,0);
		//
		//    floodFillTest.addLine(new Line(new Point(5,5),new Point(15,5)));
		//    floodFillTest.addLine(new Line(new Point(5,5),new Point(5,15)));
		//    floodFillTest.addLine(new Line(new Point(5,15),new Point(15,15)));
		//    floodFillTest.addLine(new Line(new Point(15,5),new Point(15,15)));
		//    floodFillTest.addLine(new Line(new Point(15,5),new Point(5,15)));
		//    
		//    floodFillTest.addLine(new Line(new Point(40,40),new Point(15,40)));
		//    floodFillTest.addLine(new Line(new Point(40,40),new Point(40,30)));
		//    floodFillTest.addLine(new Line(new Point(40,30),new Point(35,30)));
		//    floodFillTest.addLine(new Line(new Point(35,30),new Point(35,35)));
		//    floodFillTest.addLine(new Line(new Point(35,35),new Point(20,35)));
		//    floodFillTest.addLine(new Line(new Point(20,35),new Point(20,30)));
		//    floodFillTest.addLine(new Line(new Point(20,30),new Point(15,30)));
		//    floodFillTest.addLine(new Line(new Point(15,30),new Point(15,40)));


		//    floodFillTest.addCircle(new Circle(8, new Point(20,20)));
		//    floodFillTest.addCircle(new Circle(6, new Point(40,10)));
		//       
		//    frame.getContentPane().add(floodFillTest);
		//    
		//    frame.setVisible(true);
		//    
		//    try {
		//		Thread.sleep(1000);
		//	} catch (InterruptedException e) {
		//		// TODO Auto-generated catch block
		//		e.printStackTrace();
		//	}

		//    bresenhamCanvas.floodfill(7, 7);  
		//    bresenhamCanvas.floodfill(16, 39);
		//    bresenhamCanvas.floodfill(20, 20); 
		//    floodFillTest.linefill_codeProj(20, 20, 20);
		//    System.out.print("\n StackCounter for lineFloodFill_CodeProject: " + FloodfillCanvas.maxStackCounter);
		//bresenhamCanvas.linefill_codeProj(16, 16, 39);
		//bresenhamCanvas.floodFill_BreadthFirst(20, 20);
		//    bresenhamCanvas.floodFill_DepthFirst(20, 20);

		JFrame frame = new JFrame("Floodfill");

		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.setSize(800, 800);

		FloodfillCanvas canvas = new FloodfillCanvas(30, 30);

		boolean tutorTest = false;

		if (tutorTest) {
			// Line test
			// Rectangle test
			canvas.addLine(new Line(new Point(5, 25), new Point(25, 25))); // top
			canvas.addLine(new Line(new Point(5, 5), new Point(25, 5))); // bottom
			canvas.addLine(new Line(new Point(5, 25), new Point(5, 5))); // left
			canvas.addLine(new Line(new Point(25, 25), new Point(25, 5))); // right

			// Diagonal test
			canvas.addLine(new Line(new Point(5, 5), new Point(25, 25))); // bottom-left to top-right
			canvas.addLine(new Line(new Point(5, 25), new Point(25, 5))); // top-left to bottom-right

			// Border tests
			canvas.addLine(new Line(new Point(0, 25), new Point(5, 30))); // left-top
			canvas.addLine(new Line(new Point(30, 25), new Point(25, 30))); // right-top
			canvas.addLine(new Line(new Point(0, 5), new Point(5, 0))); // left-bottom
			canvas.addLine(new Line(new Point(30, 5), new Point(25, 0))); // right-bottom

			// Inner circle tests
			// left & right
			canvas.addLine(new Line(new Point(9, 12), new Point(21, 18))); // 5-1
			canvas.addLine(new Line(new Point(9, 18), new Point(21, 12))); // 4-8
			// top & bottom
			canvas.addLine(new Line(new Point(12, 9), new Point(18, 21))); // 6-2
			canvas.addLine(new Line(new Point(18, 9), new Point(12, 21))); // 7-3

			//bresenhamCanvas.addLine(new Line(new Point(-7, 7), new Point(10, -10))); // top-left to bottom-right

			// Circle test
			canvas.addCircle(new Circle(7, new Point(15, 15)));

			canvas.addCircle(new Circle(2, new Point(15, 25))); // top
			canvas.addCircle(new Circle(2, new Point(15, 5))); // bottom
			canvas.addCircle(new Circle(2, new Point(5, 15))); // left
			canvas.addCircle(new Circle(2, new Point(25, 15))); // right
			
		} else {
			canvas.addCircle(new Circle(5, new Point(7, 7)));
			canvas.addCircle(new Circle(5, new Point(21, 7)));
			canvas.addCircle(new Circle(5, new Point(7, 21)));
			canvas.addCircle(new Circle(5, new Point(21, 21)));
			
		}

		frame.getContentPane().add(canvas);
		frame.setVisible(true);

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		if (tutorTest) {
			// Fill test
			//	canvas.linefill_codeProj(1, 1, 29); // left-top
			//	canvas.linefill_codeProj(29, 29, 29); // right-top
			//	canvas.linefill_codeProj(1, 1, 1); // left-bottom
			//	canvas.linefill_codeProj(29, 19, 1); // right-bottom

			//	canvas.linefill_codeProj(9, 9, 16); // left
			//	canvas.linefill_codeProj(24, 24, 14); // right

			canvas.linefill_codeProj(14, 14, 12);
			System.out.print("\n StackCounter for lineFloodFill_CodeProject: " + FloodfillCanvas.maxStackCounterLinefill);
			
		} else {
			canvas.floodfill(7, 7);
			System.out.print("\n StackCounter for recursive floodFill: " + FloodfillCanvas.maxStackCounterFloodfill);
			canvas.linefill_codeProj(21, 21, 7);
			System.out.print("\n StackCounter for lineFloodFill_CodeProject: " + FloodfillCanvas.maxStackCounterLinefill);
			canvas.floodFill_BreadthFirst(7, 21);
		    canvas.floodFill_DepthFirst(21, 21);
		}

	}
}
