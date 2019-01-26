import jeigen.DenseMatrix;

public class StateSpaceObserver {
    public StateSpacePlant plant_;
    public DenseMatrix L_;

    public StateSpaceObserver(StateSpacePlant plant, DenseMatrix L) {
        plant_ = plant;
        L_ = L;
    }

    public StateSpaceObserver(int kNumInputs, int kNumStates, int kNumOutputs) {
        L_ = DenseMatrix.zeros(kNumStates, kNumOutputs);
    }

    public void Update(DenseMatrix u, DenseMatrix y) {
        DenseMatrix x_add = plant_.x_.add(L_.mmul(y.sub(plant_.y())));
        plant_.x_ = plant_.x_.add(x_add);
        plant_.Update(u);
    }
}
