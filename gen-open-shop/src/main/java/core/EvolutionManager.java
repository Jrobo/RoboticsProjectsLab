package core;

import logger.StepLogger;
import problem.ScheduleManager;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class EvolutionManager {

    private static Population population;

    public static void startEvolution(int size) {
        population = new Population(size, ScheduleManager.getNumberofOperations());
    }

    public static void iterate(int times) {
        for (int i = 0; i < times; i++) {
            population.iterate();
        }
        StepLogger.stop();
    }

}
