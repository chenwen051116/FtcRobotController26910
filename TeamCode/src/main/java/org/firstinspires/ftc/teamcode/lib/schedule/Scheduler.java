package org.firstinspires.ftc.teamcode.lib.schedule;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.concurrent.Callable;

/**
 * A scheduler to save and execute tasks in given time
 */
public class Scheduler {
    private final ElapsedTime timer;
    private final PriorityQueue<ScheduledTask> tasks;

    public Scheduler() {
        this.timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        this.tasks = new PriorityQueue<>(new Comparator<ScheduledTask>() {
            @Override
            public int compare(ScheduledTask t1, ScheduledTask t2) {
                return Double.compare(t1.milliseconds, t2.milliseconds);
            }
        });
    }

    public void addTaskAfter(double millis, Runnable task) {
        ScheduledTask newTask = new ScheduledTask(this.timer.milliseconds() + millis, task);
        this.tasks.add(newTask);
    }

    /**
     * Execute all tasks that should be executed
     */
    public void elapse() {
        while (!this.tasks.isEmpty()) {
            if (this.tasks.peek().milliseconds > this.timer.milliseconds()) break;
            ScheduledTask task = this.tasks.poll();
            assert task != null;
            task.task.run();
        }
    }
}
