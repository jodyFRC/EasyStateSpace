public class TrapezodialMotionProfile {

    MotionProfileConstraints constraints_;
    MotionProfilePosition initial_;
    MotionProfilePosition goal_;

    double direction_;
    double end_accel_;
    double end_full_speed_;
    double end_deccel_;

    public TrapezodialMotionProfile(MotionProfileConstraints constraints,
                                    MotionProfilePosition goal,
                                    MotionProfilePosition initial) {
        direction_ = (ShouldFlipAcceleration(initial, goal, constraints) ? -1 : 1);
        constraints_ = constraints;
        initial_ = Direct(initial);
        goal_ = Direct(goal);

        double cutoff_begin =
                initial_.velocity / constraints_.max_acceleration;
        double cutoff_dist_begin =
                cutoff_begin * cutoff_begin * constraints_.max_acceleration / 2.0;

        double cutoff_end = goal_.velocity / constraints_.max_acceleration;
        double cutoff_dist_end =
                cutoff_end * cutoff_end * constraints_.max_acceleration / 2.0;

        double full_trapezoid_dist =
                cutoff_dist_begin + (goal_.position - initial_.position) +
                        cutoff_dist_end;
        double acceleration_time =
                constraints_.max_velocity / constraints_.max_acceleration;

        double full_speed_dist =
                full_trapezoid_dist -
                        acceleration_time * acceleration_time * constraints_.max_acceleration;

        // Handle the case where the profile never reaches full speed
        if (full_speed_dist < 0.0) {
            acceleration_time =
                    Math.sqrt(full_trapezoid_dist / constraints_.max_acceleration);
            full_speed_dist = 0.0;
        }

        end_accel_ = acceleration_time - cutoff_begin;
        end_full_speed_ = end_accel_ + full_speed_dist / constraints_.max_velocity;
        end_deccel_ = end_full_speed_ + acceleration_time - cutoff_end;
    }

    public MotionProfilePosition Calculate(double t) {
        MotionProfilePosition result = initial_;

        if (t < end_accel_) {
            result.velocity += t * constraints_.max_acceleration;
            result.position +=
                    (initial_.velocity + t * constraints_.max_acceleration / 2.0) * t;
        } else if (t < end_full_speed_) {
            result.velocity = constraints_.max_velocity;
            result.position +=
                    (initial_.velocity + end_accel_ * constraints_.max_acceleration / 2.0) *
                            end_accel_ +
                            constraints_.max_velocity * (t - end_accel_);
        } else if (t <= end_deccel_) {
            result.velocity =
                    goal_.velocity + (end_deccel_ - t) * constraints_.max_acceleration;
            double time_left = end_deccel_ - t;
            result.position =
                    goal_.position -
                            (goal_.velocity + time_left * constraints_.max_acceleration / 2.0) *
                                    time_left;
        } else {
            result = goal_;
        }

        return Direct(result);
    }

    public double TimeLeftUntil(double target) {
        double position = initial_.position * direction_;
        double velocity = initial_.velocity * direction_;

        double end_accel = end_accel_ * direction_;
        double end_full_speed = end_full_speed_ * direction_ - end_accel;

        if (target < position) {
            end_accel *= -1.;
            end_full_speed *= -1.;
            velocity *= -1.;
        }

        end_accel = Math.max(end_accel, 0.);
        end_full_speed = Math.max(end_full_speed, 0.);
        double end_deccel = end_deccel_ - end_accel - end_full_speed;
        end_deccel = Math.max(end_deccel, 0.);

        double acceleration = constraints_.max_acceleration;
        double decceleration = -constraints_.max_acceleration;

        double dist_to_target = Math.abs(target - position);

        if (dist_to_target < 1e-6) {
            return 0.;
        }

        double accel_dist =
                velocity * end_accel + 0.5 * acceleration * end_accel * end_accel;

        double deccel_velocity;
        if (end_accel > 0) {
            deccel_velocity =
                    Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accel_dist));
        } else {
            deccel_velocity = velocity;
        }

        double deccel_dist =
                deccel_velocity * end_deccel +
                        0.5 * decceleration * end_deccel * end_deccel;

        deccel_dist = Math.max(deccel_dist, 0.);

        double full_speed_dist =
                constraints_.max_velocity * end_full_speed;

        if (accel_dist > dist_to_target) {
            accel_dist = dist_to_target;
            full_speed_dist = 0.;
            deccel_dist = 0.;
        } else if (accel_dist + end_full_speed > dist_to_target) {
            full_speed_dist = dist_to_target - accel_dist;
            deccel_dist = 0.;
        } else {
            deccel_dist = dist_to_target - full_speed_dist - accel_dist;
        }

        double accel_time =
                (-velocity +
                        Math.sqrt(Math.abs(velocity * velocity + 2 * acceleration * accel_dist))) /
        acceleration;

        double deccel_time =
                (-deccel_velocity + Math.sqrt(Math.abs(deccel_velocity * deccel_velocity +
                2 * decceleration * deccel_dist))) /
        decceleration;

        double full_speed_time =
                full_speed_dist / constraints_.max_velocity;

        return accel_time + full_speed_time + deccel_time;
    }

    public boolean ShouldFlipAcceleration(MotionProfilePosition initial, MotionProfilePosition goal, MotionProfileConstraints constraints) {
        double velocity_change = goal.velocity - initial.velocity;
        double distance_change = goal.position - initial.position;

        double t = Math.abs(velocity_change) / constraints.max_acceleration;
        boolean is_acceleraiton_flipped = t * (velocity_change / 2 + initial.velocity) > distance_change;
        return is_acceleraiton_flipped;
    }

    public MotionProfilePosition Direct(MotionProfilePosition in) {
        MotionProfilePosition result = in;
        result.position *= direction_;
        result.velocity *= direction_;
        return result;
    }

}

class MotionProfileConstraints {
    public double max_velocity;
    public double max_acceleration;

    public MotionProfileConstraints(double max_velocity, double max_acceleration) {
        this.max_velocity = max_velocity;
        this.max_acceleration = max_acceleration;
    }
}

class MotionProfilePosition {
    public double position;
    public double velocity;

    public MotionProfilePosition(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }
}