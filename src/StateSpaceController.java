import jeigen.DenseMatrix;

public class StateSpaceController {
    public DenseMatrix K;
    public DenseMatrix Kff;
    public DenseMatrix A;
    public DenseMatrix r;

    public DenseMatrix u_min;
    public DenseMatrix u_max;

    public StateSpaceController(int kNumInputs, int kNumStates, int kNumOutputs) {
        K = DenseMatrix.zeros(kNumInputs, kNumStates);
        Kff = DenseMatrix.zeros(kNumInputs, kNumStates);
        A = DenseMatrix.eye(kNumStates);

        r = DenseMatrix.zeros(kNumStates, 1);

        u_min = new DenseMatrix(kNumInputs, 1);
        u_max = new DenseMatrix(kNumInputs, 1);
    }

    public DenseMatrix Update(DenseMatrix x) {
        DenseMatrix u = K.mul(r.sub(x)).add(Kff.mul(r.sub(A.mul(r))));
        //TODO: cap u min and max
        return u;
    }
}
