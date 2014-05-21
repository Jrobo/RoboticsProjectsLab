package visual;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;

import logger.CrossoverStep;
import logger.MutationStep;
import logger.PopulationStep;
import logger.SelectionStep;
import logger.Step;
import logger.StepLogger;
import logger.Type;

public class ViewManager {

	private static final int MAX_WIDTH = 800;
	private static final int MAX_HEIGHT = 500;

	private static int iterationIndex = -1;
	private static Step current;

	public static int getIterationIndex() {
		return iterationIndex;
	}

	public static Step getCurrent() {
		return current;
	}

	public static void setIterationIndex(int iterationIndex) {
		ViewManager.iterationIndex = iterationIndex;
	}

	public static void setCurrent(Step current) {
		ViewManager.current = current;
	}

	public static void nextStep() {
		switch (current.getType()) {
		case POPULATION:
			current = StepLogger.getIteration(iterationIndex).getCrossover();
			break;
		case CROSSOVER:
			current = StepLogger.getIteration(iterationIndex).getMutation();
			break;
		case MUTATION:
			current = StepLogger.getIteration(iterationIndex).getSelection();
			break;
		case SELECTION:
			current = StepLogger.getIteration(++iterationIndex).getPopulation();
			break;
		}
	}

	public static void prevStep() {
		switch (current.getType()) {
		case POPULATION:
			current = StepLogger.getIteration(--iterationIndex).getSelection();
			break;
		case CROSSOVER:
			current = StepLogger.getIteration(iterationIndex).getPopulation();
			break;
		case MUTATION:
			current = StepLogger.getIteration(iterationIndex).getCrossover();
			break;
		case SELECTION:
			current = StepLogger.getIteration(iterationIndex).getMutation();
			break;
		}
	}

	public static boolean isLast() {
		if (iterationIndex < 0 || current == null)
			return true;
		if (iterationIndex == StepLogger.getLogSize() - 1
				&& current.getType().equals(Type.POPULATION))
			return true;
		return false;
	}

	public static boolean isFirst() {
		if (iterationIndex < 0 || current == null)
			return true;
		if (iterationIndex == 0 && current.getType().equals(Type.POPULATION))
			return true;
		return false;
	}

	public static ImageIcon getView() {

		Image img = new BufferedImage(MAX_WIDTH, MAX_HEIGHT,
				BufferedImage.TYPE_INT_RGB);
		Graphics g = img.getGraphics();
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, MAX_WIDTH, MAX_HEIGHT);

		if (iterationIndex < 0) {
			return new ImageIcon(img);
		}

		switch (current.getType()) {
		case POPULATION:
			DrawUtil.drawPopulation((PopulationStep) current,g);
			break;
		case CROSSOVER:
			DrawUtil.drawCrossover((CrossoverStep) current, g);
			break;
		case MUTATION:
			DrawUtil.drawMutation((MutationStep)current, g);
			break;
		case SELECTION:
			DrawUtil.drawSelection((SelectionStep) current, g);
			break;
		}

		g.dispose();
		return new ImageIcon(img);
	}
}
