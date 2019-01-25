import jeigen.DenseMatrix;

public class StateSpaceController {
    public DenseMatrix K_;
    public DenseMatrix Kff_;
    public DenseMatrix A_;
    public DenseMatrix r_;

    public DenseMatrix u_min_;
    public DenseMatrix u_max_;

    public StateSpaceController(int kNumInputs, int kNumStates, int kNumOutputs) {
        K_ = DenseMatrix.zeros(kNumInputs, kNumStates);
        Kff_ = DenseMatrix.zeros(kNumInputs, kNumStates);
        A_ = DenseMatrix.eye(kNumStates);

        r_ = DenseMatrix.zeros(kNumStates, 1);

        //Set bounds to +-infinite and hope it doesn't break!
        u_min_ = new DenseMatrix(kNumInputs, 1);
        u_min_.set(0,0, Double.NEGATIVE_INFINITY);
        u_max_ = new DenseMatrix(kNumInputs, 1);
        u_max_.set(0,0, Double.POSITIVE_INFINITY);

    }

    public DenseMatrix Update(DenseMatrix x) {
        DenseMatrix u = K_.mmul(r_.sub(x)).add(Kff_.mmul(r_.sub(A_.mmul(r_))));
        u = MathUtils.CapMatrix(u, u_min_, u_max_);
        return u;
    }

    public DenseMatrix Update(DenseMatrix x, DenseMatrix r) {
        DenseMatrix u = K_.mmul(r_.sub(x)).add(Kff_.mmul(r.sub(A_.mmul(r_))));
        r_ = r;
        u = MathUtils.CapMatrix(u, u_min_, u_max_);
        return u;
    }

}
