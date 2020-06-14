import java.awt.*;

import javax.swing.JComponent;


public class Painter extends JComponent{
	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	AutoAlgo2 algo;
	
	public Painter(AutoAlgo2 algo2) {
		this.algo = algo2;
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		algo.paint(g);
	}
}
