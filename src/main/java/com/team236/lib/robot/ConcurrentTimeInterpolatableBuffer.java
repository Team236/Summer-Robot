package com.team236.lib.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.ConcurrentNavigableMap;
import java.util.concurrent.ConcurrentSkipListMap;

/**
 * The {@code ConcurrentTimeInterpolatableBuffer} provides a concurrent version of WPILib's
 * TimeInterpolatableBuffer class to avoid the need for explicit synchronization in robot code.
 *
 * @param <T> The type stored in this buffer.
 */
public final class ConcurrentTimeInterpolatableBuffer<T> {
    private final double mHistorySize;
    private final Interpolator<T> mInterpolatingFunc;
    private final ConcurrentNavigableMap<Double, T> mPastSnapshots = new ConcurrentSkipListMap<>();

    private ConcurrentTimeInterpolatableBuffer(
            Interpolator<T> interpolateFunction, double historySizeSeconds) {
        this.mHistorySize = historySizeSeconds;
        this.mInterpolatingFunc = interpolateFunction;
    }

    /**
     * Create a new TimeInterpolatableBuffer.
     *
     * @param interpolateFunction The function used to interpolate between values.
     * @param historySizeSeconds The history size of the buffer.
     * @param <T> The type of data to store in the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static <T> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
            Interpolator<T> interpolateFunction, double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(interpolateFunction, historySizeSeconds);
    }

    /**
     * Create a new TimeInterpolatableBuffer that stores a given subclass of {@link Interpolatable}.
     *
     * @param historySizeSeconds The history size of the buffer.
     * @param <T> The type of {@link Interpolatable} to store in the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static <T extends Interpolatable<T>> ConcurrentTimeInterpolatableBuffer<T> createBuffer(
            double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(
                Interpolatable::interpolate, historySizeSeconds);
    }

    /**
     * Create a new TimeInterpolatableBuffer to store Double values.
     *
     * @param historySizeSeconds The history size of the buffer.
     * @return The new TimeInterpolatableBuffer.
     */
    public static ConcurrentTimeInterpolatableBuffer<Double> createDoubleBuffer(
            double historySizeSeconds) {
        return new ConcurrentTimeInterpolatableBuffer<>(MathUtil::interpolate, historySizeSeconds);
    }

    /**
     * Add a sample to the buffer.
     *
     * @param timeSeconds The timestamp of the sample.
     * @param sample The sample object.
     */
    public void addSample(double timeSeconds, T sample) {
        mPastSnapshots.put(timeSeconds, sample);
        cleanUp(timeSeconds);
    }

    /** Clear all old samples. */
    public void clear() {
        mPastSnapshots.clear();
    }

    /**
     * Sample the buffer at the given time. If the buffer is empty, an empty Optional is returned.
     *
     * @param timeSeconds The time at which to sample.
     * @return The interpolated value at that timestamp or an empty Optional.
     */
    public Optional<T> getSample(double timeSeconds) {
        if (mPastSnapshots.isEmpty()) {
            return Optional.empty();
        }

        // Special case for when the requested time is the same as a sample
        var nowEntry = mPastSnapshots.get(timeSeconds);
        if (nowEntry != null) {
            return Optional.of(nowEntry);
        }

        var bottomBound = mPastSnapshots.floorEntry(timeSeconds);
        var topBound = mPastSnapshots.ceilingEntry(timeSeconds);

        // Return empty if neither sample exists
        if (topBound == null && bottomBound == null) {
            return Optional.empty();
        } else if (topBound == null) {
            return Optional.of(bottomBound.getValue());
        } else if (bottomBound == null) {
            return Optional.of(topBound.getValue());
        } else {
            // Otherwise, interpolate.
            // Ratio of (time difference between current time and bottom bound) to (time difference
            // between top and bottom bounds).
            return Optional.of(
                    mInterpolatingFunc.interpolate(
                            bottomBound.getValue(),
                            topBound.getValue(),
                            (timeSeconds - bottomBound.getKey())
                                    / (topBound.getKey() - bottomBound.getKey())));
        }
    }

    /**
     * Returns the latest sample in the buffer.
     *
     * @return The latest sample in the buffer.
     */
    public Entry<Double, T> getLatest() {
        return mPastSnapshots.lastEntry();
    }

    /**
     * Grant access to the internal sample buffer. Used in Pose Estimation to replay odometry inputs
     * stored within this buffer.
     *
     * @return The internal sample buffer.
     */
    public ConcurrentNavigableMap<Double, T> getInternalBuffer() {
        return mPastSnapshots;
    }

    /**
     * Removes samples older than our current history size.
     *
     * @param time The current timestamp.
     */
    private void cleanUp(double time) {
        mPastSnapshots.headMap(time - mHistorySize, false).clear();
    }
}
