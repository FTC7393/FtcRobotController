package ftc.evlib.util;

import java.util.ArrayList;
import java.util.List;

public class ImmutableList {
    public static <E> List<E> of(E e1, E e2) {
        return construct(e1, e2);
    }

    public static <E> List<E> of(E e1) {
        return construct(e1);
    }

    public static <E> List<E> of(E e1, E e2, E e3) {
        return construct(e1, e2, e3);
    }

    public static <E> List<E> of(E e1, E e2, E e3, E e4) {
        return construct(e1, e2, e3, e4);
    }

    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5) {
        return construct(e1, e2, e3, e4, e5);
    }

    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6) {
        return construct(e1, e2, e3, e4,  e5,  e6);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7, E e8) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7, e8);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7, E e8, E e9) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7, e8, e9);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7, E e8, E e9, E e10) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7, e8, e9, e10);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7, E e8, E e9, E e10, E e11) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7, e8, e9, e10, e11);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7, E e8, E e9, E e10, E e11, E e12) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7, e8, e9, e10, e11, e12);
    }
    public static <E> List<E> of(E e1, E e2, E e3, E e4, E e5, E e6, E e7, E e8, E e9, E e10, E e11, E e12, E e13) {
        return construct(e1, e2, e3, e4,  e5,  e6, e7, e8, e9, e10, e11, e12, e13);
    }


    private static <E> List<E> construct(Object... elements) {
        return asList(checkElementsNotNull(elements));
    }

    static Object[] checkElementsNotNull(Object... array) {
        return checkElementsNotNull(array, array.length);
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

