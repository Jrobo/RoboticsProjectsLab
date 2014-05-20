package logger;

import java.util.List;

import core.Chromosome;
import core.Population;

public class StepLogger {

	private static List<Iteration> log;
	private static Iteration current;
	
	public static void clear() {
		log.clear();
		current = new Iteration();
		log.add(current);
	}
	
	public static void next() {
		current = new Iteration();
		log.add(current);
	}
	
	public static void logPopulationStep(Population pop) {
		current.addPopulation(pop);
	}
	
	public static void logCrossoverParents(Chromosome p1, Chromosome p2) {
		current.addCrossoverParents(p1, p2);
	}
	
	public static void logCrossoverChilds(Chromosome c1, Chromosome c2) {
		current.addCrossoverChilds(c1, c2);
	}
	
	public static void logCrossoverAlone(Chromosome c) {
		current.addCrossoverAlone(c);
	}
	
	public static void logMutation(Chromosome c, int mutation) {
		current.addMutation(c, mutation);
	}
	
	public static void logSelectionPassed(List<Chromosome> success) {
		current.logSelectionSuccess(success);
	}
	
	public static void logSelectionFailure(Chromosome c) {
		current.logSelectionFailure(c);
	}
	
	public static void logSelectionFailed(Chromosome c) {
		
	}
}
