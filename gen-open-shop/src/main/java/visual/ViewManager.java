package visual;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;

import javax.swing.ImageIcon;

import logger.Step;
import logger.StepLogger;
import logger.Type;

public class ViewManager {

	private static final int MAX_WIDTH = 500;
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
			current = StepLogger.getIteration(++iterationIndex).getMutation();
			break;
		}
	}

	public static boolean isLast() {
		if (iterationIndex < 0)
			return true;
		if (iterationIndex == StepLogger.getLogSize() - 1
				&& current.getType().equals(Type.POPULATION))
			return true;
		return false;
	}

	public static boolean isFirst() {
		if (iterationIndex < 0)
			return true;
		if (iterationIndex == 0 && current.getType().equals(Type.POPULATION))
			return true;
		return false;
	}

	public static ImageIcon getView() {

		Image img = new BufferedImage(MAX_WIDTH, MAX_HEIGHT,
				BufferedImage.TYPE_INT_RGB);
		Graphics g = img.getGraphics();

		if (iterationIndex < 0) {
			g.setColor(Color.WHITE);
			g.fillRect(0, 0, MAX_WIDTH, MAX_HEIGHT);
			return new ImageIcon(img);
		}

		/* Draw!!! */

		return new ImageIcon(img);
	}
}
