package core;

import problem.ScheduleManager;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class EvolutionManager {

    private int populationSize;
    private Population population;


    public EvolutionManager() {
    }

    public void startEvolution(int size) {
        populationSize = size;
        population = new Population(size, ScheduleManager.getNumberofOperations());
    }

    public void iterate(int times) {
        for (int i = 0; i < times; i++) {
            population.iterate();
        }
    }

}
