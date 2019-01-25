import jeigen.DenseMatrix;
import org.junit.Assert;
import org.junit.Test;

public class StateSpaceTest {
    @Test
    public void testInit() {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        StateSpaceController controller = new StateSpaceController(1, 2, 1);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                if (i == j) {
                    Assert.assertEquals(plant.A_.get(i, j), 1, 0.0001);
                    Assert.assertEquals(controller.A.get(i, j), 1, 0.0001);
                } else {
                    Assert.assertEquals(plant.A_.get(i, j), 0, 0.0001);
                    Assert.assertEquals(controller.A.get(i, j), 0, 0.0001);
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
    public void goalTest(DenseMatrix r) {
        StateSpacePlant plant = new StateSpacePlant(1, 2, 1);
        plant.A_ = new DenseMatrix("1 0.01; 0 0.98");
        plant.B_ = new DenseMatrix("1e-5; 0.02");
        plant.C_ = new DenseMatrix("1 0");
        plant.x_ = new DenseMatrix("1; 0");

        StateSpaceController controller = new StateSpaceController(1, 2, 1);
        controller.K = new DenseMatrix("10.0 1.0");
        controller.A = plant.A_;
        controller.Kff = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r = r;

        for (int t = 0; t < 2000; t++) {
            DenseMatrix u = controller.Update(plant.x_);
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), controller.r.get(0, 0), 1e-6);
        Assert.assertEquals(plant.x_.get(1, 0), controller.r.get(1, 0), 1e-6);
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
        controller.K = new DenseMatrix("10.0 1.0");
        controller.A = plant.A_;
        controller.Kff = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r = new DenseMatrix("3.0; 0.0");

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = controller.Update(plant.x_);
            //plant.Update(new DenseMatrix("0")); //system will converge on zero provided zero input
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), controller.r.get(0, 0), 1e-2);
        Assert.assertEquals(plant.x_.get(1, 0), controller.r.get(1, 0), 1e-2);
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
        controller.K = new DenseMatrix("10.0 1.0");
        controller.A = plant.A_;
        controller.Kff = (plant.B_.t().mmul(plant.B_)).recpr().mmul(plant.B_.t());
        controller.r = new DenseMatrix("1.0; 0.0");

        controller.u_max = new DenseMatrix("12");
        controller.u_min = new DenseMatrix("-12");

        for (int t = 0; t < 1000; t++) {
            DenseMatrix u = controller.Update(plant.x_);
            Assert.assertTrue(u.get(0,0) <= controller.u_max.get(0, 0));
            Assert.assertTrue(u.get(0,0) >= controller.u_min.get(0, 0));
            plant.Update(u);
        }

        Assert.assertEquals(plant.x_.get(0, 0), controller.r.get(0, 0), 1e-6);
        Assert.assertEquals(plant.x_.get(1, 0), controller.r.get(1, 0), 1e-6);
    }
}


