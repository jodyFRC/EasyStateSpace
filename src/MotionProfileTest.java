import org.junit.Assert;
import org.junit.Test;

public class MotionProfileTest {

    MotionProfileConstraints constraints;
    MotionProfilePosition goal;
    MotionProfilePosition initial_position;

    private void SetUp() {
        constraints = new MotionProfileConstraints(0, 0);
        constraints.max_velocity = 1; // m/s
        constraints.max_acceleration = 1; //m/s/s
    }

    private void RunTest() {
        TrapezodialMotionProfile profile = new TrapezodialMotionProfile(constraints, goal,
                initial_position);
        double dt = 0.005;
        double discrete_error = 0.0026; // m/s

        boolean timing_this_test = !(initial_position.velocity != 0. &&
                initial_position.position == goal.position) &&
                (Math.abs(goal.velocity) < 1e-10);

        // Discrete time differentiation leaves a bit of over/undershoot.

        Assert.assertEquals(profile.Calculate(0).position, initial_position.position,
                1e-6);
        Assert.assertEquals(profile.Calculate(0).velocity, initial_position.velocity,
                1e-6);
        for (double t = 0; t < profile.total_time(); t += dt) {  // NOLINT
            double estimated_acceleration =
                    (profile.Calculate(t).velocity - profile.Calculate(t - dt).velocity) /
                            (dt);
            double estimated_velocity =
                    (profile.Calculate(t).position - profile.Calculate(t - dt).position) /
                            (dt);

            if ((Math.abs(profile.Calculate(t).position - goal.position) < 1e-3) &&
                    timing_this_test) {
                Assert.assertEquals(profile.TimeLeftUntil(profile.Calculate(t).position), t,
                        5e-2);
            }

            Assert.assertTrue(constraints.max_velocity >=
                    Math.abs(profile.Calculate(t).velocity));
            Assert.assertTrue(constraints.max_acceleration + (discrete_error) >
                    Math.abs(estimated_acceleration));
            Assert.assertEquals(profile.Calculate(t).velocity, estimated_velocity,
                    discrete_error);

            t += dt;
        }

        Assert.assertEquals(profile.Calculate(profile.total_time()).position, goal.position,
                1e-5);
        Assert.assertEquals(profile.Calculate(profile.total_time()).velocity, goal.velocity,
                1e-5);
    }

    @Test
    public void PositiveGoal() {
        initial_position = new MotionProfilePosition(0,0);
        goal = new MotionProfilePosition(3, 0);
        SetUp();
        RunTest();
    }

    @Test
    public void NegativeGoal() {
        initial_position = new MotionProfilePosition(0,0);
        goal = new MotionProfilePosition(-3, 0);
        SetUp();
        RunTest();
    }

    @Test
    public void OnGoal() {
        initial_position = new MotionProfilePosition(2,0);
        goal = new MotionProfilePosition(2, 0);
        SetUp();
        RunTest();
    }

    @Test
    public void InitialVelocity() {
        initial_position = new MotionProfilePosition(0,0.5);
        goal = new MotionProfilePosition(3, 0);
        SetUp();
        RunTest();
    }

    @Test
    public void ZeroGoalPositiveGoalVelocity() {
        initial_position = new MotionProfilePosition(1,0);
        goal = new MotionProfilePosition(0, 0.6);
        SetUp();
        RunTest();
    }

}
