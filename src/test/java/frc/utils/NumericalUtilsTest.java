package frc.utils;

import org.junit.Assert;
import org.junit.Test;

public class NumericalUtilsTest {

    private void withinRangeCases(NumericalUtils.ToleranceType toleranceType, final boolean desiredResult) {
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(10, 0, 1000, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(-10, 0, 1000, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(999, 0, 1000, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(0, 0, 1000, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(0, 0, 1000, toleranceType));
    }

    private void greaterThanRangeCases(NumericalUtils.ToleranceType toleranceType, final boolean desiredResult) {
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(10, 0, 0, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(10, 0, 9, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(10, 0, 9.99, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(1001, 0, 1000, toleranceType));
    }

    private void lessThanRangeCases(NumericalUtils.ToleranceType toleranceType, final boolean desiredResult) {
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(-10, 0, 0, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(10, 989, 0, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(-10, 0, 9, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(-10, 0, 9.99, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(-1001, 0, 1000, toleranceType));
        Assert.assertEquals(desiredResult, NumericalUtils.isWithinTolerance(10, 1000, 989, toleranceType));
    }

    @Test
    public void isWithinToleranceGreaterThan() {
        final NumericalUtils.ToleranceType toleranceType = NumericalUtils.ToleranceType.GREATER_ALLOWED;
        withinRangeCases(toleranceType, true);
        greaterThanRangeCases(toleranceType, true);
        lessThanRangeCases(toleranceType, false);
    }

    @Test
    public void isWithinToleranceLessThan() {
        final NumericalUtils.ToleranceType toleranceType = NumericalUtils.ToleranceType.LESS_ALLOWED;
        withinRangeCases(toleranceType, true);
        greaterThanRangeCases(toleranceType, false);
        lessThanRangeCases(toleranceType, true);
    }

    @Test
    public void isWithinToleranceOnlyInRange() {
        final NumericalUtils.ToleranceType toleranceType = NumericalUtils.ToleranceType.ONLY_WITHIN_TOLERANCE;
        withinRangeCases(toleranceType, true);
        greaterThanRangeCases(toleranceType, false);
        lessThanRangeCases(toleranceType, false);
    }
}
