package visual;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Arrays;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableModel;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class Application extends JFrame {

	private static final long serialVersionUID = 1L;

	private JTable table;

	/*
	 * Need to set Problem to Schedule Manager Need to set new population to
	 * EvolutionManager
	 */

	public Application() {
		setTitle("Genetic Algorithm for Open-Shop Scheduling");
		// setSize(size);
		// setLayout(new BorderLayout());

		getContentPane().add(createSettingPanel());

		pack();
		setVisible(true);

		setDefaultCloseOperation(EXIT_ON_CLOSE);
	}

	private JPanel createSettingPanel() {
		JPanel settingsPanel = new JPanel();
		settingsPanel.setLayout(new BorderLayout());

		settingsPanel.add(createTopPanel(), BorderLayout.NORTH);
		settingsPanel.add(createCenterPanel(), BorderLayout.CENTER);
		settingsPanel.add(createBottomPanel(), BorderLayout.SOUTH);

		settingsPanel.setVisible(true);
		return settingsPanel;
	}

	private JPanel createTopPanel() {
		JPanel topPanel = new JPanel();
		topPanel.setLayout(new FlowLayout());
		JLabel topLabel = new JLabel("Number of Machines: ");
		final JTextField topText = new JTextField();
		topText.setPreferredSize(new Dimension(30, 25));
		JButton topButton = new JButton("Apply");
		topButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				int columnNumber = Integer.parseInt(topText.getText());
				if (columnNumber > 0)
					updateTable(columnNumber);
			}
		});

		topPanel.add(topLabel);
		topPanel.add(topText);
		topPanel.add(topButton);
		topPanel.setVisible(true);
		return topPanel;
	}

	private JPanel createCenterPanel() {
		JPanel centerPanel = new JPanel();
		centerPanel.setLayout(new BorderLayout());

		table = new JTable();
		updateTable(3);

		centerPanel.add(new JScrollPane(table), BorderLayout.CENTER);
		return centerPanel;
	}

	private void updateTable(int n) {
		if (table.getModel().getColumnCount() == n)
			return;
		String columns[] = new String[n];
		for (int i = 0; i < n; i++) {
			columns[i] = "M-" + String.valueOf(i + 1);
		}
		TableModel tm = new DefaultTableModel(columns, 0);
		table.setModel(tm);
		table.setVisible(true);
	}

	private JPanel createBottomPanel() {
		JPanel bottomPanel = new JPanel();
		bottomPanel.setLayout(new FlowLayout());

		JButton addRow = new JButton("Add Job");
		addRow.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				DefaultTableModel model = (DefaultTableModel) table.getModel();
				int n = model.getColumnCount();
				Integer[] row = new Integer[n];
				Arrays.fill(row, 0);
				model.addRow(row);
			}
		});

		JLabel label = new JLabel("Population size:");
		JTextField text = new JTextField();
		text.setPreferredSize(new Dimension(30, 25));
		JPanel subPanel = new JPanel();

		subPanel.setLayout(new FlowLayout());
		subPanel.add(label);
		subPanel.add(text);

		JButton start = new JButton("Start Evolution!");
		start.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				// TODO start evolution

			}
		});

		bottomPanel.add(addRow);
		bottomPanel.add(subPanel);
		bottomPanel.add(start);
		bottomPanel.setVisible(true);
		return bottomPanel;
	}

	public static void main(String arg[]) {
		setLookAndFeel();
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				Application app = new Application();
				app.setVisible(true);
			}
		});
	}

	private static void setLookAndFeel() {
		try {
			UIManager
					.setLookAndFeel("com.sun.java.swing.plaf.nimbus.NimbusLookAndFeel");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
