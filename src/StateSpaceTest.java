import jeigen.DenseMatrix;
import org.junit.Assert;
import org.junit.Test;

import javax.swing.plaf.nimbus.State;

public class StateSpaceTest {
    @Test
    public void testInit() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        StateSpaceController controller = new StateSpaceController(1, 2, 1);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                if (i == j) {
                    Assert.assertEquals(plant.A_.get(i, j), 1, 0.0001);
                    Assert.assertEquals(controller.A_.get(i, j), 1, 0.0001);
                } else {
                    Assert.assertEquals(plant.A_.get(i, j), 0, 0.0001);
                    Assert.assertEquals(controller.A_.get(i, j), 0, 0.0001);
                }
            }
        }

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                plant.A_.set(i, j, 2 * i + j);
            }
            plant.B_.set(i, 0, i + 4);
            plant.C_.set(0, i, i + 6);
        }

        StateSpacePlant plant2 = new StateSpacePlant(plant.A_, plant.B_, plant.C_);

        // Ensure that the plant uses the gains given to it
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                Assert.assertEquals(plant2.A_.get(i, j), plant.A_.get(i, j), 0.0001);
            }
            Assert.assertEquals(plant2.B_.get(i, 0), plant.B_.get(i, 0), 0.0001);
            Assert.assertEquals(plant2.C_.get(0, i), plant.C_.get(0, i), 0.0001);
        }
    }

    // Ensure that a mathematically stable plant converges to zero
    @Test
    public void testConvergeStability() {
        DenseMatrix A = new DenseMatrix("1.0 .01; -.05 .95");
        StateSpacePlant plant = new StateSpacePlant(A, DenseMatrix.zeros(2, 1),
                DenseMatrix.zeros(1, 2), DenseMatrix.zeros(1, 1),
                new DenseMatrix("1; 1")
        );
        DenseMatrix u = DenseMatrix.zeros(1, 1);

        for (int i = 0; i < 2000; i++) {
            plant.Update(u);
        }

        for (int i = 0; i < 2; i++) {
            Assert.assertEquals(plant.x_.get(i, 0), 0, 1e-6);
        }

    }

    // Ensure that a mathematically unstable plant diverges to infinity
    @Test
    public void testDivergeUnstable() {
        DenseMatrix A = new DenseMatrix("1.0 .01; .05 0.99");

        StateSpacePlant plant = new StateSpacePlant(A, DenseMatrix.zeros(2, 1),
                DenseMatrix.zeros(1, 2), DenseMatrix.zeros(1, 1),
                new DenseMatrix("1; 0")
        );

        DenseMatrix u = DenseMatrix.zeros(1, 1);

        for (int i = 0; i < 1000; i++) {
            plant.Update(u);
        }

        for (int i = 0; i < 2; i++) {
            Assert.assertTrue(plant.x_.get(i, 0) > 1e6);
        }
    }

    @SuppressWarnings("Duplicates")
    private void goalTest(DenseMatrix r) {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 0.01; 0 0.98");
        plant.B_ = new DenseMatrix("1e-5; 0.02");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("1; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("10.0 1.0");
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = r;

        for (int t = 0; t < 2000; t++) {
            DenseMatrix u = controller.Update(plant.x_);
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), controller.r_.get(0, 0), 1e-6);
        Assert.assertEquals(plant.x_.get(1, 0), controller.r_.get(1, 0), 1e-6);
    }

    // Ensure that adding a controller causes a plant to converge to zero
    @Test
    public void testControlConvergeZero() {
        goalTest(new DenseMatrix("0; 0"));
    }

    // Ensure that adding a controller causes a plant to converge on a defined goal
    @Test
    public void testControlConvergeGoal() {
        goalTest(new DenseMatrix("6; 0"));
    }

    // Ensure the controller will hold a correct steady-state goal even if the goal
    // requires a nonzero control signal (via feedforward)
    @Test
    @SuppressWarnings("Duplicates")
    public void testControlSteadyState() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 0.01; -.03 0.98");
        plant.B_ = new DenseMatrix("1e-5; 0.02");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("1; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("10.0 1.0");
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("3.0; 0.0");

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = controller.Update(plant.x_);
            //plant.Update(new DenseMatrix("0")); //system will converge on zero provided zero input
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), controller.r_.get(0, 0), 1e-2);
        Assert.assertEquals(plant.x_.get(1, 0), controller.r_.get(1, 0), 1e-2);
    }

    // Ensure that the control signal from the controller stays within the specified
    // bounds
    @Test
    @SuppressWarnings("Duplicates")
    public void testControlInputConstraint() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 0.01; 0 0.98");
        plant.B_ = new DenseMatrix("1e-5; 0.02");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("-5; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("10.0 1.0");
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("1.0; 0.0");

        controller.u_max_ = new DenseMatrix("12");
        controller.u_min_ = new DenseMatrix("-12");

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = controller.Update(plant.x_);
            Assert.assertTrue(u.get(0, 0) <= controller.u_max_.get(0, 0));
            Assert.assertTrue(u.get(0, 0) >= controller.u_min_.get(0, 0));
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), controller.r_.get(0, 0), 1e-6);
        Assert.assertEquals(plant.x_.get(1, 0), controller.r_.get(1, 0), 1e-6);
    }

    // Ensure that the controller correctly tracks a moving goal
    @Test
    @SuppressWarnings("Duplicates")
    public void testControlTrackMovingGoal() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 9.9502e-3; 0 9.9005e-1");
        plant.B_ = new DenseMatrix("4.9834e-5; 9.9502e-3");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("0; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("0.0 0.0"); //Zero gains so feedback is eliminated
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("0.0; 0.0");

        DenseMatrix r = controller.r_;
        DenseMatrix expected_u = new DenseMatrix("1.0");

        for (int t = 0; t < 1000; t++) {
            r = plant.A_.mmul(r).add(plant.B_.mmul(expected_u));

            DenseMatrix u = controller.Update(plant.x_, r);
            plant.Update(u);

            Assert.assertEquals(plant.x_.get(0, 0), controller.r_.get(0, 0), 1e-3);
            Assert.assertEquals(plant.x_.get(1, 0), controller.r_.get(1, 0), 1e-3);

            Assert.assertEquals(u.get(0, 0), expected_u.get(0, 0), 1e-3);
        }
    }

    // Ensure that the observer's estimation converges to the correct state after a
    // large initial error in a static system
    @Test
    @SuppressWarnings("Duplicates")
    public void testObserverConvergeInit() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 9.9502e-3; 0 9.9005e-1");
        plant.B_ = new DenseMatrix("4.9834e-5; 9.9502e-3");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("0; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("0.0 0.0");
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("0.0; 0.0");

        DenseMatrix L = new DenseMatrix("1e-1; 1");
        StateSpaceObserver observer = new StateSpaceObserver(plant, L);

        plant.x_.set(0, 0, 1);

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = DenseMatrix.zeros(1, 1);
            plant.Update(u);
            observer.Update(u, plant.y());
        }

        Assert.assertEquals(plant.x_.get(0, 0), observer.plant_.x_.get(0, 0), 0.00001);
        Assert.assertEquals(plant.x_.get(1, 0), observer.plant_.x_.get(1, 0), 0.00001);
    }

    // Ensure that the observer's estimation converges to the correct state after a
    // large initial error in a moving system
    @Test
    @SuppressWarnings("Duplicates")
    public void testObserverMovingConvergeRecover() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 9.9502e-3; 0 9.9005e-1");
        plant.B_ = new DenseMatrix("4.9834e-5; 9.9502e-3");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("0; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("0.0 0.0");
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("0.0; 0.0");

        DenseMatrix L = new DenseMatrix("1e-1; 1");
        StateSpaceObserver observer = new StateSpaceObserver(plant, L);

        plant.x_.set(0, 0, 1);

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = new DenseMatrix("1.0");
            observer.Update(u, plant.y());
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), observer.plant_.x_.get(0, 0), 0.00001);
        Assert.assertEquals(plant.x_.get(1, 0), observer.plant_.x_.get(1, 0), 0.00001);
    }

    // Ensure that the observer's estimation converges to the correct state when its
    // model is slightly incorrect
    @Test
    @SuppressWarnings("Duplicates")
    public void testObserverRecoverIncorrectModel() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 9.9502e-3; 0 9.9005e-1");
        plant.B_ = new DenseMatrix("4.9834e-5; 9.9502e-3");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("0; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K_ = new DenseMatrix("0.0 0.0");
        controller.A_ = plant.A_;
        controller.Kff_ = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r_ = new DenseMatrix("0.0; 0.0");

        DenseMatrix L = new DenseMatrix("2e-1; 3");
        StateSpaceObserver observer = new StateSpaceObserver(plant, L);

        plant.A_.set(1, 1, plant.A_.get(1, 1) * 0.985);
        plant.A_.set(0, 1, plant.A_.get(0, 1) * 0.995);

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = new DenseMatrix("1.0");
            System.out.println(plant.x_);
            observer.Update(u, plant.y());
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), observer.plant_.x_.get(0, 0), 0.00001);
        Assert.assertEquals(plant.x_.get(1, 0), observer.plant_.x_.get(1, 0), 0.00001);
    }
}


