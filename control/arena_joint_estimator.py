"""Optional 16-error-state ESKF: navigation plus jointly estimated barometer bias.

Right-multiplicative attitude errors; state order p,v,theta,ba,bg,baro_bias.
The legacy 15-state filter remains unchanged. Fixed-lag replay is inherited.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from control.arena_estimator import NavigationFilter, _qmul, _skew


def right_jacobian(rotation_vector):
    vector = np.asarray(rotation_vector, dtype=float)
    theta = np.linalg.norm(vector); W = _skew(vector)
    if theta < 1e-5:
        return np.eye(3)-.5*W+(1./6.)*W@W
    return np.eye(3)-(1.-np.cos(theta))/theta**2*W+(theta-np.sin(theta))/theta**3*(W@W)


def discretize_error_model(A, spectral_density, dt):
    """Second-order transition and PSD Simpson quadrature of process covariance."""
    transition = lambda t: np.eye(len(A))+A*t+.5*(A@A)*t*t
    F, half = transition(dt), transition(.5*dt)
    Q = dt/6.*(spectral_density+4.*half@spectral_density@half.T+F@spectral_density@F.T)
    return F, (Q+Q.T)*.5


class JointNavigationFilter(NavigationFilter):
    def __init__(self, x0_17, profile, gravity=9.81):
        super().__init__(x0_17, profile, gravity)
        P = np.zeros((16,16)); P[:15,:15] = self.P
        P[15,15] = float(self.profile['estimator'].get('initial_baro_bias_sigma_m', 1.))**2
        self.P = P
        self._initial_snapshot = self._snapshot()

    def set_initial_baro_bias(self, value):
        if self._events: raise RuntimeError('preflight calibration must precede filter events')
        e = self.profile['estimator']
        count = int(e.get('preflight_baro_samples', 0))
        if count < 1: raise ValueError('joint preflight calibration requires a positive sample count')
        # Independent preflight white noise and reference uncertainty. Shared
        # reference/navigation errors require a cross-covariance model instead.
        self.P[15,15] = (float(e.get('preflight_baro_sigma_m', self.profile['barometer']['sigma']))**2/count
                        + float(e.get('preflight_reference_sigma_m', 0.))**2)
        super().set_initial_baro_bias(value)

    def _propagate(self, z, dt):
        if dt <= 0.: return
        f, w = np.asarray(z[:3])-self.ba, np.asarray(z[3:6])-self.bg
        R = Rotation.from_quat(self.q).as_matrix()
        acceleration = R@f+np.array([0.,0.,-self.gravity])
        self.p += self.v*dt+.5*acceleration*dt*dt; self.v += acceleration*dt
        self.q = _qmul(self.q, Rotation.from_rotvec(w*dt).as_quat()); self.q /= np.linalg.norm(self.q)
        A = np.zeros((16,16)); A[:3,3:6] = np.eye(3)
        A[3:6,6:9] = -R@_skew(f); A[3:6,9:12] = -R
        A[6:9,6:9] = -_skew(w); A[6:9,12:15] = -np.eye(3)
        imu, e = self.profile['imu'], self.profile['estimator']
        L = np.diag(np.r_[np.zeros(3), np.full(3,float(imu['accel_noise_density'])**2),
                         np.full(3,float(imu['gyro_noise_density'])**2),
                         np.full(3,float(e.get('accel_bias_rw_density',imu['accel_bias_rw']))**2),
                         np.full(3,float(e.get('gyro_bias_rw_density',imu['gyro_bias_rw']))**2),
                         float(e.get('baro_bias_rw_density_m_sqrt_s',.03))**2])
        F,Q = discretize_error_model(A,L,dt)
        self.P = F@self.P@F.T+Q; self.P = (self.P+self.P.T)*.5

    def _update_joint(self, measurement, predicted, H, Rm, sensor):
        y, h, Rm = np.asarray(measurement), np.asarray(predicted), np.asarray(Rm)
        innovation = y-h; S = H@self.P@H.T+Rm
        e = self.profile['estimator']
        gate = float(e.get('nis_gates', {}).get(sensor, {'gnss':22.458,'barometer':10.828,'magnetometer':16.266}[sensor]))
        if self.trace_updates:
            self._last_update_detail = dict(measurement=y.tolist(),predicted=h.tolist(),innovation=innovation.tolist(),
                innovation_covariance=S.tolist(),measurement_covariance=Rm.tolist(),
                covariance_diagonal_before=np.diag(self.P).tolist(),state_before=self.state[:13].tolist(),
                correction_error_state=[0.]*16,nis_gate=gate,error_state_dimension=16)
        if not np.all(np.isfinite(innovation)) or not np.all(np.isfinite(S)): return False,np.inf
        try:
            nis = float(innovation@np.linalg.solve(S,innovation))
            if not np.isfinite(nis) or nis > gate: return False,nis
            K = np.linalg.solve(S,H@self.P).T
        except np.linalg.LinAlgError:
            return False,np.inf
        delta = K@innovation
        self._inject(delta[:15]); self.baro_bias += delta[15]
        I = np.eye(16); posterior = (I-K@H)@self.P@(I-K@H).T+K@Rm@K.T
        reset = np.eye(16); reset[6:9,6:9] = right_jacobian(delta[6:9])
        self.P = reset@posterior@reset.T; self.P = (self.P+self.P.T)*.5
        if self.trace_updates: self._last_update_detail['correction_error_state'] = delta.tolist()
        return True,nis

    def _apply_event(self, packet):
        if self.time is None: self.time = float(packet.sample_time)
        dt = float(packet.sample_time)-self.time
        if dt < -1e-9: raise ValueError('events must be applied in timestamp order')
        if self.last_imu is not None: self._propagate(self.last_imu,max(0.,dt))
        self.time = max(self.time,float(packet.sample_time))
        if packet.kind == 'imu': self.last_imu = packet.value.copy(); return None,None
        e = self.profile['estimator']
        if packet.kind == 'gnss':
            H = np.zeros((6,16)); H[:3,:3] = np.eye(3); H[3:6,3:6] = np.eye(3)
            Rm = np.diag(np.r_[np.full(3,float(e['gnss_pos_sigma'])**2),np.full(3,float(e['gnss_vel_sigma'])**2)])
            return self._update_joint(packet.value,np.r_[self.p,self.v],H,Rm,'gnss')
        if packet.kind == 'barometer':
            H = np.zeros((1,16)); H[0,2] = H[0,15] = 1.
            return self._update_joint(packet.value,[self.p[2]+self.baro_bias],H,[[float(e['baro_sigma'])**2]],'barometer')
        if packet.kind == 'magnetometer':
            field = np.asarray(self.profile['magnetometer']['world_field'])
            h = Rotation.from_quat(self.q).as_matrix().T@field
            H = np.zeros((3,16)); H[:,6:9] = _skew(h)
            return self._update_joint(packet.value,h,H,np.eye(3)*float(e['mag_sigma'])**2,'magnetometer')
        return None,None

    def _rebuild_diagnostics(self):
        super()._rebuild_diagnostics()
        self.diagnostics.update(error_state_dimension=16,barometer_bias_variance=float(self.P[15,15]),
                                filter_kind='joint_baro')
