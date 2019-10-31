package ftc.evlib.util;

import java.util.ArrayList;
import java.util.List;

public class ImmutableList {
    public static <E> List<E> of(E e1, E e2) {
        return construct(e1, e2);
    }

    static Object[] checkElementsNotNull(Object... array) {
        return checkElementsNotNull(array, array.length);
    }

    private static <E> List<E> construct(Object... elements) {
        return asList(checkElementsNotNull(elements));
    }


    static Object[] checkElementsNotNull(Object[] array, int length) {
        for (int i = 0; i < length; i++) {
            checkElementNotNull(array[i], i);
        }
        return array;
    }

    // We do this instead of Preconditions.checkNotNull to save boxing and array
    // creation cost.
    static Object checkElementNotNull(Object element, int index) {
        if (element == null) {
            throw new NullPointerException("at index " + index);
        }
        return element;
    }

    static <E> List<E> asList(Object[] elements) {
        return asList(elements, elements.length);
    }

    /**
     * Views the array as an immutable list. Copies if the specified range does not cover the complete
     * array. Does not check for nulls.
     */
    static <E> List<E> asList(Object[] elements, int length) {

        List<E> list = new ArrayList<>(length);

        for (Object o : elements) {
            list.add((E) o);
        }
        return list;
    }




}

