package frc.lib.other;

public class RunningAverageQueue {
	private int end;
	private final int size;
	private double[] data;
	private double total;

	public RunningAverageQueue(int size) {
		this.size = size;
		data = new double[size];
		end = 0;
		total = 0;
	}

	public void push(double num) {
		total += num;
		end = (end + 1) % size;
		total -= data[end];
		data[end] = num;
	}

	public double getAverage() {
		return total / size;
	}

	public void print() {
		for (int i = 0; i < size; i++)
			System.out.print(data[i] + " ");
		System.out.println();
	}
}
