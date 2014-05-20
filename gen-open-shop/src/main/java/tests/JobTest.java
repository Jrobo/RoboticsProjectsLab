package tests;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import problem.Job;

import java.util.Arrays;
import java.util.List;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */

public class JobTest {

    private static final Integer[] TEST_ARRAY = {1, 2, 3, 4, 5};

    private Job test1;
    private Job test2;
    private Job test3;

    @Before
    public void init() {
        List<Integer> operations = Arrays.asList(1, 2, 3, 4, 5);
        test1 = new Job(operations);
        test2 = new Job(operations);
        test3 = new Job(operations);
    }

    @Test
    public void testCreation() {
        Assert.assertArrayEquals("Creation failed! ", TEST_ARRAY, test1.getOperations().toArray());
    }

    @Test
    public void testProcess() {
        test2.processOperation(1, 12);
        Assert.assertEquals("Size is not equal! ", 1, test2.getProcessed().keySet().size());
        Assert.assertEquals("Time is not equal! ", 12, test2.getProcessed().keySet().toArray()[0]);
        Assert.assertEquals("Index is not equal! ", 1, test2.getProcessed().values().toArray()[0]);
    }

    @Test
    public void testFindingGap() {
        test3.processOperation(0, 0);
        test3.processOperation(3, 4);
        test3.processOperation(1, 7);

        int index = test3.findGap(0,7,2);
        Assert.assertEquals("", 1, index);

        index = test3.findGap(1,7,5);
        Assert.assertEquals("", -1, index);

        index = test3.findGap(1,5);
        Assert.assertEquals("", 9, index);

        index = test3.findGap(10,5);
        Assert.assertEquals("", 10, index);
    }
}
