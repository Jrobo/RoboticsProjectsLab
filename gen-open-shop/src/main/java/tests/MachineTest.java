package tests;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import problem.Job;
import problem.Machine;

import java.util.Arrays;
import java.util.Map;
import java.util.TreeMap;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */
public class MachineTest {

    Machine test1;
    Job testJob1;
    Job testJob2;
    Job testJob3;

    @Before
    public void init() {
        test1 = new Machine(0);
        testJob1 = new Job(Arrays.asList(1,3));
        testJob2 = new Job(Arrays.asList(3,5));
        testJob3 = new Job(Arrays.asList(1,1));
        testJob2.processOperation(1,0);
    }

    @Test
    public void testAddingJobs() {
        Map<Integer, Job> testMap = new TreeMap<Integer, Job>();

        test1.addJob(testJob1);
        testMap.put(0,testJob1);
        Assert.assertEquals("", testMap, test1.getSchedule());
        Assert.assertEquals("", 1, test1.getTime());

        test1.addJob(testJob2);
        testMap.put(5,testJob2);
        Assert.assertEquals("", testMap, test1.getSchedule());
        Assert.assertEquals("", 8, test1.getTime());

        test1.addJob(testJob3);
        testMap.put(1,testJob3);
        Assert.assertEquals("", testMap, test1.getSchedule());
        Assert.assertEquals("", 8, test1.getTime());
    }
}
