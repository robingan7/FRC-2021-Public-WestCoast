package frc.lib.other;

import frc.lib.math.Interpolable;
import frc.lib.math.InterpolablePair;

public class CircularQueue<T extends Interpolable<T>> {
	/**
	 * NOT thread safe. This class assumes the data added is sorted! It has
	 * O(1)insertion (only able to insert at end) and O(n log n) search
	 */

	private InterpolablePair<T>[] queue;
	private long back;
	public final int size;

	@SuppressWarnings("unchecked")
	public CircularQueue(int size) {
		queue = new InterpolablePair[size];
		back = 0;
		this.size = size;
	}

	/**
	 *
	 * @param t
	 *            Add new a new value to the end of the queue
	 */
	public void add(InterpolablePair<T> t) {
		queue[(int) back % size] = t;
		back++;
	}

	/**
	 * Get InterpolableValue<T> from the queue that is a specified distance from
	 * the back
	 *
	 * @param position
	 *            Distance from back of queue
	 * @return InterpolableValue<T> from position in argument
	 */
	public InterpolablePair<T> getFromQueue(int position) {
		position %= size;
		return queue[(int) (back - position - 1) % size];
	}

	/**
	 * Get T with given key. If an exact match isn't found it will interpolate
	 * the value always. Keys outside the range will return the front or end of
	 * the queue.
	 *
	 * @param key
	 *            Key of wanted T
	 * @return Matching interpolated T from key in argument
	 */
	public T getInterpolatedKey(long key) {
		int low = 0;
		int high = queue.length - 1;
		while (low <= high) {
			int mid = (low + high) / 2;
			double midVal = getFromQueue(mid).getKey();
			if (midVal < key) {
				low = mid + 1;
			} else if (midVal > key) {
				high = mid - 1;
			} else {
				return getFromQueue(mid).getValue();
			}
		}
		double difference = key - getFromQueue(low).getKey();
		double total = getFromQueue(high).getKey() - getFromQueue(low).getKey();
		return getFromQueue(low).getValue().interpolate(getFromQueue(high).getValue(), difference / total);
	}
}