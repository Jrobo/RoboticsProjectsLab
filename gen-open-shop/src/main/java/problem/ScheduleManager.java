package problem;

import core.Chromosome;

public class ScheduleManager {
    private static Problem problem = new Problem(0,0);

    public static Problem getProblem() {
        return problem;
    }

    public static void setProblem(Problem problem) {
        ScheduleManager.problem = problem;
    }

    public static int makespan( Chromosome chromosome) {
        return getSchedule(chromosome).getTime();
	}

    public static Schedule getSchedule(Chromosome chromosome) {
        Schedule schedule = new Schedule(problem);
        for (int i = 0; i< chromosome.getLength(); i++) {
            int gen = chromosome.getGen(i);
            schedule.shedule(getOperationMachine(gen),getOperationJob(gen));
        }
        return schedule;
    }
	
	public static int getOperationJob( int index) {
		return index / problem.getNumberOfJobs();
	}
	
	public static int getOperationMachine( int index) {
		return index % problem.getNumberOfJobs();
	}
	
	public static int getOperationLength( int machine, int job) {
		return problem.getOperation(job,machine);
	}

    public static int getNumberofOperations() {
        return problem.getNumberOfJobs() * problem.getNumberOfMachines();
    }
}
