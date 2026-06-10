package tests;

import core.Chromosome;
import core.CrossoverWheel;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * Created by Tatiyana Domanova on 5/20/14.
 */

public class CrossoverWheelTest {

    private final static Chromosome test1 = new Chromosome(Arrays.asList(3,1,2,4,5));
    private final static Chromosome test2 = new Chromosome(Arrays.asList(1,2,3,4,5));
    private final static Chromosome test3 = new Chromosome(Arrays.asList(4,1,2,3,5));

    private static Map<Integer, Chromosome> testMap;

    CrossoverWheel wheel;

    @Before
    public void init() {
        test1.setValue(1);
        test2.setValue(3);
        test3.setValue(5);
        wheel = new CrossoverWheel(Arrays.asList(test1,test2,test3));

        testMap = new TreeMap<Integer, Chromosome>();
        testMap.put(1, test1);
        testMap.put(4, test2);
        testMap.put(9, test3);
    }



    @Test
    public void testWheel() {
        Assert.assertEquals(testMap,wheel.getWheel());
        Assert.assertEquals(9, wheel.getSize());

        for (int i = 0; i<15; i++)
            wheel.getParent();
    }
}
