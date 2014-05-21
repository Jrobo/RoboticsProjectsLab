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

import problem.Problem;
import problem.ScheduleManager;
import core.EvolutionManager;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class Application extends JFrame {

	private static final long serialVersionUID = 1L;

	JLabel navLabel;
	private JTable table;
	private JLabel img;

	private JButton left = new JButton("Prev");
	private JButton right = new JButton("Next");

	/*
	 * Need to set Problem to Schedule Manager Need to set new population to
	 * EvolutionManager
	 */

	public Application() {
		setTitle("Genetic Algorithm for Open-Shop Scheduling");
		setLayout(new BorderLayout());

		getContentPane().add(createSettingPanel(), BorderLayout.WEST);
		getContentPane().add(createViewPanel(), BorderLayout.CENTER);

		pack();
		setVisible(true);

		setDefaultCloseOperation(EXIT_ON_CLOSE);
	}

	private JPanel createViewPanel() {
		JPanel viewPanel = new JPanel();
		viewPanel.setLayout(new BorderLayout());

		viewPanel.add(createImage(), BorderLayout.CENTER);
		viewPanel.add(createNavPanel(), BorderLayout.SOUTH);

		return viewPanel;
	}

	private JPanel createImage() {
		JPanel imagePanel = new JPanel();

		img = new JLabel();
		imagePanel.add(img, BorderLayout.CENTER);
		img.setIcon(ViewManager.getView());

		return imagePanel;
	}

	private JPanel createNavPanel() {
		JPanel navPanel = new JPanel();

		navPanel.setLayout(new FlowLayout());

		navLabel = new JLabel();

		ActionListener nav = new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				if (e.getSource() == left)
					ViewManager.prevStep();
				else
					ViewManager.nextStep();
				if (ViewManager.isFirst())
					left.setEnabled(false);
				else
					left.setEnabled(true);
				if (ViewManager.isLast())
					right.setEnabled(false);
				else
					right.setEnabled(true);

				if (ViewManager.getIterationIndex() >= 0) {
					SwingUtilities.invokeLater(new Runnable() {
						
						@Override
						public void run() {
							String generation = "Generation "
									+ ViewManager.getIterationIndex();
							String step = ViewManager.getCurrent().getType().name();
							navLabel.setText(generation + ": " + step);	
							img.setIcon(ViewManager.getView());
						}
					});
				}
				
			}
		};

		left.setEnabled(false);
		right.setEnabled(false);
		left.addActionListener(nav);
		right.addActionListener(nav);

		navPanel.add(left);
		navPanel.add(navLabel);
		navPanel.add(right);

		return navPanel;
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
		final JTextField textP = new JTextField();
		textP.setPreferredSize(new Dimension(30, 25));
		JPanel subPanel = new JPanel();

		subPanel.setLayout(new FlowLayout());
		subPanel.add(label);
		subPanel.add(textP);

		label = new JLabel("Generations: ");
		final JTextField textG = new JTextField();
		textG.setPreferredSize(new Dimension(30, 25));
		JPanel subPanel1 = new JPanel();

		subPanel1.setLayout(new FlowLayout());
		subPanel1.add(label);
		subPanel1.add(textG);

		JButton start = new JButton("Start Evolution!");
		start.addActionListener(new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent arg0) {
				int population = Integer.parseInt(textP.getText());
				int generationNum = Integer.parseInt(textG.getText());

				int machines = table.getModel().getColumnCount();
				int jobs = table.getModel().getRowCount();

				int[][] operations = new int[jobs][machines];

				for (int i = 0; i < jobs; i++) {
					for (int j = 0; j < machines; j++) {
						try {
							operations[i][j] = Integer.parseInt((String) table
									.getValueAt(i, j));
						} catch (ClassCastException e) {
							operations[i][j] = (Integer) table.getValueAt(i, j);
						}
					}
				}

				Problem problem = new Problem(machines, jobs, operations);
				ScheduleManager.setProblem(problem);

				EvolutionManager.startEvolution(population);
				EvolutionManager.iterate(generationNum);

				left.setEnabled(false);
				right.setEnabled(true);
				String generation = "Generation "
						+ ViewManager.getIterationIndex();
				if (ViewManager.getIterationIndex() >= 0) {
					String step = ViewManager.getCurrent().getType().name();
					navLabel.setText(generation + ": " + step);
					img.setIcon(ViewManager.getView());
				}
				
				
			}
		});

		bottomPanel.add(addRow);
		bottomPanel.add(subPanel);
		bottomPanel.add(subPanel1);
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
