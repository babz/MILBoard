

import java.awt.Point;

import javax.swing.JFrame;

public class FloodfillTest {
  public static void main(String[] args) {
    JFrame frame = new JFrame("Floodfill");
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
    frame.setSize(470, 490);
    FloodfillCanvas floodFillTest = new FloodfillCanvas(50, 50);
    
    floodFillTest.addPoint(2,2);
    floodFillTest.addPoint(2,3);
    floodFillTest.addPoint(3,2);
    floodFillTest.addPoint(3,3);
    floodFillTest.addPoint(0,0);

    floodFillTest.addLine(new Line(new Point(5,5),new Point(15,5)));
    floodFillTest.addLine(new Line(new Point(5,5),new Point(5,15)));
    floodFillTest.addLine(new Line(new Point(5,15),new Point(15,15)));
    floodFillTest.addLine(new Line(new Point(15,5),new Point(15,15)));
    floodFillTest.addLine(new Line(new Point(15,5),new Point(5,15)));
    
    floodFillTest.addLine(new Line(new Point(40,40),new Point(15,40)));
    floodFillTest.addLine(new Line(new Point(40,40),new Point(40,30)));
    floodFillTest.addLine(new Line(new Point(40,30),new Point(35,30)));
    floodFillTest.addLine(new Line(new Point(35,30),new Point(35,35)));
    floodFillTest.addLine(new Line(new Point(35,35),new Point(20,35)));
    floodFillTest.addLine(new Line(new Point(20,35),new Point(20,30)));
    floodFillTest.addLine(new Line(new Point(20,30),new Point(15,30)));
    floodFillTest.addLine(new Line(new Point(15,30),new Point(15,40)));

    
    floodFillTest.addCircle(new Circle(8, new Point(20,20)));
    floodFillTest.addCircle(new Circle(6, new Point(40,10)));
       
    frame.getContentPane().add(floodFillTest);
    
    frame.setVisible(true);
    
    try {
		Thread.sleep(1000);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    
//    bresenhamCanvas.floodfill(7, 7);  
//    bresenhamCanvas.floodfill(16, 39);
//    bresenhamCanvas.floodfill(20, 20); 
    floodFillTest.linefill_codeProj(20, 20, 20);
    System.out.print("\n StackCounter for lineFloodFill_CodeProject: " + FloodfillCanvas.maxStackCounter);
    //bresenhamCanvas.linefill_codeProj(16, 16, 39);
    //bresenhamCanvas.floodFill_BreadthFirst(20, 20);
//    bresenhamCanvas.floodFill_DepthFirst(20, 20);
  }
}
