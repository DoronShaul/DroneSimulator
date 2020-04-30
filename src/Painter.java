import java.awt.*;

import javax.swing.JComponent;


public class Painter extends JComponent{
	AutoAlgo2 algo;
//	AutoAlgo1 algo;
//	
//	
//	public Painter(AutoAlgo1 algo) {
//		this.algo = algo;
//	}
	
	public Painter(AutoAlgo2 algo2) {
		this.algo = algo2;
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		algo.paint(g);
	}
}
