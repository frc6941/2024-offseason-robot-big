package org.frcteam6941.utils;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.Interpolatable;

public class InterpolatingTreeMap<K extends Number, V extends Interpolatable<V>> extends TreeMap<K, V> {
    final int max_;

    public InterpolatingTreeMap(int maximumSize) {
        max_ = maximumSize;
    }

    public InterpolatingTreeMap() {
        this(0);
    }

    /**
     * Inserts a key value pair, and trims the tree if a max size is specified
     *
     * @param key   Key for inserted data
     * @param value Value for inserted data
     * @return the value
     */
    @Override
    public V put(K key, V value) {
        if (max_ > 0 && max_ <= size()) {
            // "Prune" the tree if it is oversize
            K first = firstKey();
            remove(first);
        }

        super.put(key, value);

        return value;
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> map) {
        System.out.println("Unimplemented Method");
    }

    /**
     * @param key Lookup for a value (does not have to exist)
     * @return V or null; V if it is Interpolable or exists, null if it is at a bound and cannot average
     */
    public V getInterpolated(K key, double interpolant) {
        Optional<V> gotval = Optional.ofNullable(get(key));
        return gotval.orElseGet(() -> {
            // get surrounding keys for interpolation
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // if attempting interpolation at ends of tree, return the nearest data point
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // get surrounding values for interpolation
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);
            return bottomElem.interpolate(topElem, interpolant);
        });
    }
}
