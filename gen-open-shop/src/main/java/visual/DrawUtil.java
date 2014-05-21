package visual;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.Random;

import org.javatuples.Pair;

import logger.CrossoverStep;
import logger.MutationStep;
import logger.PopulationStep;
import logger.SelectionStep;
import core.Chromosome;

public class DrawUtil {

	private static final int MAX_WIDTH = 700;
	private static final int MAX_HEIGHT = 500;

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
			drawChromosome(g, c, 10 + r.nextInt(MAX_WIDTH - 50),
					10 + r.nextInt(MAX_HEIGHT - 10));
		}

	}

	public static void drawCrossover(CrossoverStep step, Graphics g) {
		Graphics2D g2 = (Graphics2D) g;
		for (int i = 0; i < step.getChilds().size(); i++) {
			int x = 10 + r.nextInt(MAX_WIDTH -150);
			int y = 10 + r.nextInt(MAX_HEIGHT - 50);

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
			int x = 10 + r.nextInt(MAX_WIDTH - 100);
			int y = 10 + r.nextInt(MAX_HEIGHT - 40);

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
		g2.drawLine(posx, posy - 15, x+5, posy + 5);
	}

	public static void drawSelection(SelectionStep step, Graphics g) {
		for (Chromosome c : step.getPopulation().getIndividuals()) {
			int x = 10 + r.nextInt(MAX_WIDTH - 50);
			int y = 10 + r.nextInt(MAX_HEIGHT - 40);
			drawChromosome(g, c, x, y);
		}
		for (Chromosome c : step.getDeleted()) {
			int x = 10 + r.nextInt(MAX_WIDTH - 100);
			int y = 10 + r.nextInt(MAX_HEIGHT - 40);
			drawDeleted(g, c, x, y);
		}

	}
}
