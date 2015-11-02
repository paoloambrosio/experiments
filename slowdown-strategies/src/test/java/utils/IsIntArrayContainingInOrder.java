package utils;

import org.apache.commons.lang3.ArrayUtils;
import org.hamcrest.Description;
import org.hamcrest.Factory;
import org.hamcrest.Matcher;
import org.hamcrest.TypeSafeMatcher;
import org.hamcrest.collection.IsIterableContainingInOrder;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static java.util.Arrays.asList;
import static org.hamcrest.core.IsEqual.equalTo;

public class IsIntArrayContainingInOrder extends TypeSafeMatcher<int[]> {

    private final Collection<Matcher<? super Integer>> matchers;
    private final IsIterableContainingInOrder<Integer> iterableMatcher;

    private IsIntArrayContainingInOrder(List<Matcher<? super Integer>> matchers) {
        this.iterableMatcher = new IsIterableContainingInOrder<Integer>(matchers);
        this.matchers = matchers;
    }

    @Override
    public boolean matchesSafely(int[] item) {
        return iterableMatcher.matches(asList(ArrayUtils.toObject(item)));
    }

    @Override
    public void describeMismatchSafely(int[] item, Description mismatchDescription) {
        Integer[] boxedArray = ArrayUtils.toObject(item);
        mismatchDescription.appendText("was ").appendValue(boxedArray).appendText(" -> ");
        iterableMatcher.describeMismatch(asList(boxedArray), mismatchDescription);
    }

    @Override
    public void describeTo(Description description) {
        description.appendList("[", ", ", "]", matchers);
    }

    @Factory
    public static Matcher<int[]> intArrayContaining(int... items) {
        List<Matcher<? super Integer>> matchers = new ArrayList<Matcher<? super Integer>>();
        for (int item : items) {
            matchers.add(equalTo(item));
        }
        return new IsIntArrayContainingInOrder(matchers);
    }
}