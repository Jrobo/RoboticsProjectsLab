package visual;

import javax.swing.JDialog;
import javax.swing.JLabel;

import core.Chromosome;

public class ScheduleDialog extends JDialog{

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;
	
	public ScheduleDialog(Chromosome c) {
		JLabel label = new JLabel();
		label.setIcon(ViewManager.getSchedule(c));
		getContentPane().add(label);
		pack();
	}
	

}
