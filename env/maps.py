
from scipy.interpolate import UnivariateSpline, Rbf, griddata
from scipy.interpolate import NearestNDInterpolator
import numpy as np
class ParamToFeatureMap:
    # map from hidden param X to features Y(if not reward) from data
    def __init__(self, X, Y, fit = 'linear', vis = True):
        assert X.shape[0] == Y.shape[0], "X and Y must have the same number of samples"
        self.X = X
        self.Y = Y
        self.n_data = X.shape[0]
        self.n_features = Y.shape[1]
        self.n_params = X.shape[1]
        self.coef_ = None
        self.fit = fit
        if fit == 'linear':
            self.linear_fit()
        elif fit == 'poly':
            self.poly_fit()
        elif fit == 'NN':
            self.NN_fit()
        elif fit == 'spline':
            self.spline_fit()
        else:
            raise ValueError(f"Unknown fit type: {fit}")

    def linear_fit(self):
        X_aug = np.hstack([np.ones((self.n_data, 1)), self.X])
        self.coef_ = np.linalg.lstsq(X_aug, self.Y, rcond=None)[0]

    def poly_fit(self, n=2):
        assert n > 1, "Polynomial degree must be greater than 1"
        self.poly_degree = n
        X_poly = np.hstack([self.X**i for i in range(n+1)])
        self.coef_ = np.linalg.lstsq(X_poly, self.Y, rcond=None)[0]

    def NN_fit(self):
        self.nn = NearestNDInterpolator(self.X, self.Y)

    def spline_fit(self): # TODO: fix
        # Assumes 1D X and Y for UnivariateSpline
        if self.X.shape[1] == 1:
            self.spline = UnivariateSpline(self.X.flatten(), self.Y, s=0)
        else:
            # For higher dimensions, use Rbf
            self.spline = [Rbf(*[self.X[:,i] for i in range(self.X.shape[1])], self.Y[:,j]) for j in range(self.Y.shape[1])]

    def __call__(self, x):
        # Map the input x to the feature space using the learned coefficients
        # x = np.atleast_2d(x)
        if self.fit == 'linear':
            x_aug = np.hstack([np.ones((x.shape[0], 1)), x])
            return x_aug @ self.coef_
        elif self.fit == 'poly':
            x_poly = np.hstack([x**i for i in range(self.poly_degree + 1)])
            return x_poly @ self.coef_
        elif self.fit == 'NN':
            return self.nn(x).flatten()
        elif self.fit == 'spline':
            return np.array([spline(*[x[:,i] for i in range(x.shape[1])]) for spline in self.spline])

class Feature2RewardQuadMap:
    # map from features Y to reward R
    def __init__(self, Q, R, target):
        self.n_feature = Q.shape[0]
        self.n_action = R.shape[0]
        self.Q = Q
        self.R = R
        self.target = target

    def __call__(self, s, a, target = None):
        # quadratic reward
        target = self.target if target is None else target
        s,a, target = s.flatten(), a.flatten(), target.flatten()
        assert s.shape[0] == self.n_feature, "State dimension mismatch"
        assert a.shape[0] == self.n_action, "Action dimension mismatch"
        r = -(s - target)[:,None].T @ self.Q @ (s - target)[:,None] - a[:,None].T @ self.R @ a[:,None]
        r = r.flatten()
        assert r.shape[0] == 1
        return r