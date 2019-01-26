import jeigen.DenseMatrix;

public class StateSpacePlant {
    public DenseMatrix A_;
    public DenseMatrix B_;
    public DenseMatrix C_;
    public DenseMatrix D_;
    public DenseMatrix x_;

    public StateSpacePlant(int kNumInputs, int kNumStates, int kNumOutputs) {
        A_ = DenseMatrix.eye(kNumStates);
        B_ = DenseMatrix.zeros(kNumStates, kNumInputs);
        C_ = DenseMatrix.zeros(kNumOutputs, kNumStates);
        D_ = DenseMatrix.zeros(kNumOutputs, kNumInputs);
        x_ = DenseMatrix.zeros(kNumStates, 1);
    }

    public StateSpacePlant(DenseMatrix A, DenseMatrix B, DenseMatrix C, DenseMatrix D, DenseMatrix x_0) {
        A_ = A;
        B_ = B;
        C_ = C;
        D_ = D;
        x_ = x_0;
    }

    public StateSpacePlant(DenseMatrix A, DenseMatrix B, DenseMatrix C, DenseMatrix D) {
        A_ = A;
        B_ = B;
        C_ = C;
        D_ = D;
        x_ = DenseMatrix.zeros(A_.rows, 1);
    }

    public StateSpacePlant(DenseMatrix A, DenseMatrix B, DenseMatrix C) {
        A_ = A;
        B_ = B;
        C_ = C;
        D_ = DenseMatrix.zeros(A_.rows, B_.cols);
        x_ = DenseMatrix.zeros(A_.rows, 1);
    }

    public DenseMatrix y() {
        return  C_.mmul(x_);
    }

    public void Update(DenseMatrix u) {
        x_ = A_.mmul(x_).add(B_.mmul(u));
    }
}
