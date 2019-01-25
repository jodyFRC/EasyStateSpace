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

    @Test
    public void testConvergeZero() {
        DenseMatrix A = new DenseMatrix("1.0 .01; -.05 .95");
        StateSpacePlant plant = new StateSpacePlant(A, DenseMatrix.zeros(2, 1),
                DenseMatrix.zeros(1, 2), DenseMatrix.zeros(1, 1),
                new DenseMatrix("1; 1")
        );
        DenseMatrix u = DenseMatrix.zeros(1, 1);

        for (int i = 0; i < 100; i++) {
            plant.Update(u);
        }

        for (int i = 0; i < 2000; i++) {
            plant.Update(u);
        }

        for (int i = 0; i < 2; i++) {
            Assert.assertEquals(plant.x_.get(i, 0), 0, 1e-6);
        }

    }
}
