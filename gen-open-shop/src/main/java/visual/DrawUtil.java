package visual;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Random;

import logger.CrossoverStep;
import logger.MutationStep;
import logger.PopulationStep;
import logger.SelectionStep;

import org.javatuples.Pair;

import problem.Job;
import problem.Machine;
import problem.Schedule;
import problem.ScheduleManager;
import core.Chromosome;

public class DrawUtil {

	private static final int MAX_WIDTH = 700;
	private static final int MAX_HEIGHT = 500;

	public static Color[] colors = new Color[] { Color.BLUE, Color.CYAN,
			Color.GREEN, Color.LIGHT_GRAY, Color.MAGENTA, Color.ORANGE,
			Color.PINK, Color.RED, Color.YELLOW, Color.BLUE, Color.CYAN,
			Color.GREEN, Color.LIGHT_GRAY, Color.MAGENTA, Color.ORANGE,
			Color.PINK, Color.RED, Color.YELLOW };

	private static Random r = new Random();

	private static void drawChromosome(Graphics g, Chromosome c, int posx,
			int posy) {
		Graphics2D g2 = (Graphics2D) g;
		int x = posx;
		for (int i = 0; i < c.getLength(); i++) {
			String v = String.valueOf(c.getGen(i));
			g2.setColor(Color.BLACK);
			g2.drawString(v, x + 2, posy);
			g2.setColor(Color.BLUE);
			g2.drawRect(x, posy - 12, v.length() * 10, 15);
			x += v.length() * 10;
		}
	}

	public static void drawPopulation(PopulationStep step, Graphics g) {

		for (Chromosome c : step.getPopulation().getIndividuals()) {
			int x = 20 + r.nextInt(MAX_WIDTH
					- ScheduleManager.getNumberofOperations() * 15);
			int y = 20 + r.nextInt(MAX_HEIGHT - 50);
			drawChromosome(g, c, x, y);
		}

	}

	public static void drawCrossover(CrossoverStep step, Graphics g) {
		Graphics2D g2 = (Graphics2D) g;
		for (int i = 0; i < step.getChilds().size(); i++) {
			int x = 20 + r.nextInt(MAX_WIDTH
					- ScheduleManager.getNumberofOperations() * 30);
			int y = 20 + r.nextInt(MAX_HEIGHT - 50);

			Pair<Chromosome, Chromosome> parents = step.getParents().get(i);
			Pair<Chromosome, Chromosome> childs = step.getChilds().get(i);

			drawChromosome(g, parents.getValue0(), x, y);
			g2.drawString("+", x + parents.getValue0().getLength() * 4, y + 15);
			drawChromosome(g, parents.getValue1(), x, y + 30);

			x += Math.max(parents.getValue0().getLength(), parents.getValue1()
					.getLength()) * 10 + 7;
			g2.drawString(" => ", x, y + 15);
			x += 20;

			drawChromosome(g, childs.getValue0(), x, y);
			g2.drawString("+", x + parents.getValue0().getLength() * 4, y + 15);
			drawChromosome(g, childs.getValue1(), x, y + 30);
		}
	}

	public static void drawMutation(MutationStep step, Graphics g) {
		Graphics2D g2 = (Graphics2D) g;
		for (Pair<Chromosome, Integer> pair : step.getMutation()) {
			int x = 20 + r.nextInt(MAX_WIDTH
					- ScheduleManager.getNumberofOperations() * 20);
			int y = 20 + r.nextInt(MAX_HEIGHT - 40);

			drawChromosome(g, pair.getValue0(), x, y);

			x += pair.getValue0().getLength() * 10 + 7;
			g2.drawString(" => ", x, y);
			x += 20;

			Chromosome c = new Chromosome(pair.getValue0().getGenom());
			int gen1 = pair.getValue1() == 0 ? c.getLength() - 1 : pair
					.getValue1() - 1;
			int gen2 = pair.getValue1() == c.getLength() - 1 ? 0 : pair
					.getValue1() + 1;
			c.swapGenes(gen1, gen2);

			drawChromosome(g, c, x, y);
		}

	}

	private static void drawDeleted(Graphics g, Chromosome c, int posx, int posy) {
		Graphics2D g2 = (Graphics2D) g;
		int x = posx;
		for (int i = 0; i < c.getLength(); i++) {
			String v = String.valueOf(c.getGen(i));
			g2.setColor(Color.BLACK);
			g2.drawString(v, x + 2, posy);
			g2.setColor(Color.BLUE);
			g2.drawRect(x, posy - 12, v.length() * 10, 15);
			x += v.length() * 10;

		}
		g2.setColor(Color.RED);
		g2.drawLine(posx, posy - 15, x + 5, posy + 5);
	}

	public static void drawSelection(SelectionStep step, Graphics g) {
		for (Chromosome c : step.getPopulation().getIndividuals()) {
			int x = 20 + r.nextInt(MAX_WIDTH
					- ScheduleManager.getNumberofOperations() * 15);
			int y = 20 + r.nextInt(MAX_HEIGHT - 40);
			drawChromosome(g, c, x, y);
		}
		for (Chromosome c : step.getDeleted()) {
			int x = 20 + r.nextInt(MAX_WIDTH
					- ScheduleManager.getNumberofOperations() * 15);
			int y = 10 + r.nextInt(MAX_HEIGHT - 40);
			drawDeleted(g, c, x, y);
		}

	}

	public static void drawSchedule(Schedule schedule, Graphics g) {

		g.setColor(Color.WHITE);
		g.fillRect(0, 0, MAX_WIDTH, MAX_HEIGHT);

		Graphics2D g2 = (Graphics2D) g;

		int x0 = 20;
		int y0 = MAX_HEIGHT - 20;
		g2.setBackground(Color.WHITE);
		g2.setStroke(new BasicStroke(2));
		g2.setColor(Color.DARK_GRAY);

		g2.drawLine(x0, y0, MAX_WIDTH - 20, y0);
		g2.drawLine(x0, y0, x0, 20);
		g2.drawLine(MAX_WIDTH - 25, y0 - 3, MAX_WIDTH - 20, y0);
		g2.drawLine(MAX_WIDTH - 25, y0 + 3, MAX_WIDTH - 20, y0);

		int m = schedule.getMachines().size();
		int n = schedule.getJobs().size();

		int dy = (MAX_HEIGHT - 60) / m;
		for (int i = 1; i <= m; i++) {
			g2.drawLine(x0, y0 - i * dy, MAX_WIDTH - 20, y0 - i * dy);
			g2.drawString("M" + i, x0 - 20, y0 - i * dy);
		}

		int max_x = schedule.getTime() + 10;
		int nx = max_x / 10;
		max_x = 10 * nx;

		for (int i = 1; i <= 30; i++) {
			g2.drawLine(x0 + (MAX_WIDTH - 60) * i / 10, y0 - 4, x0
					+ (MAX_WIDTH - 60) * i / 10, y0 + 4);
			g2.drawString(String.valueOf(nx * i), x0 + (MAX_WIDTH - 60) * i
					/ 10, y0 + 15);
		}

		for (Machine mac : schedule.getMachines()) {
			int y1 = y0 - (mac.getIndex() + 1) * dy;
			int x = x0;
			for (Integer time : mac.getSchedule().keySet()) {
				Job j = mac.getSchedule().get(time);
				g2.setColor(colors[j.getIndex()]);

				g2.fillRect(x + (time * (MAX_WIDTH - 60) / max_x), y1 - 10,
						j.getOperationLength(mac.getIndex()) * (MAX_WIDTH - 60)
								/ max_x, 20);

				g2.setColor(Color.BLACK);
				if (j.getOperationLength(mac.getIndex()) != 0) {
					g2.drawString(
							"J" + j.getIndex(),
							x + (time * (MAX_WIDTH - 60) / max_x)
									+ j.getOperationLength(mac.getIndex())
									* (MAX_WIDTH - 60) / max_x / 2, y1 + 3);
				}
			}
		}

		g2.drawString("Total time: " + schedule.getTime(), MAX_WIDTH - 100, 30);
	}

}
