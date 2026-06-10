package visual;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import javax.swing.ImageIcon;

import logger.CrossoverStep;
import logger.MutationStep;
import logger.PopulationStep;
import logger.SelectionStep;
import logger.Step;
import logger.StepLogger;
import logger.Type;

import org.javatuples.Pair;

import problem.Schedule;
import problem.ScheduleManager;
import core.Chromosome;

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
	
	public static List<Chromosome> getAllChromosomes() {
		switch (current.getType()) {
		case POPULATION:
			return ((PopulationStep) current).getPopulation().getIndividuals();
		case CROSSOVER:
			List<Chromosome> list = new ArrayList<Chromosome>();
			for (Pair<Chromosome, Chromosome> pair: ((CrossoverStep) current).getParents()) {
				list.add(pair.getValue0());
				list.add(pair.getValue1());
			}
			for (Pair<Chromosome, Chromosome> pair: ((CrossoverStep) current).getChilds()) {
				list.add(pair.getValue0());
				list.add(pair.getValue1());
			}
			return list;
		case MUTATION:
			List<Chromosome> list1 = new ArrayList<Chromosome>();
			for (Pair<Chromosome, Integer> pair: ((MutationStep) current).getMutation()) {
				list1.add(pair.getValue0());
			}
			return list1;
		case SELECTION:
			List<Chromosome> list2 = new ArrayList<Chromosome>();
			list2.addAll(((SelectionStep) current).getPopulation().getIndividuals());
			list2.addAll(((SelectionStep) current).getDeleted());
			return list2;
		}
		return Collections.EMPTY_LIST;
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
	
	public static ImageIcon getSchedule(Chromosome c) {
		
		Schedule schedule = ScheduleManager.getSchedule(c);

		Image img = new BufferedImage(MAX_WIDTH, MAX_HEIGHT,
				BufferedImage.TYPE_INT_RGB);
		Graphics g = img.getGraphics();
		g.setColor(Color.WHITE);
		g.fillRect(0, 0, MAX_WIDTH, MAX_HEIGHT);

		DrawUtil.drawSchedule(schedule, g);
		
		g.dispose();
		return new ImageIcon(img);
	}

}
