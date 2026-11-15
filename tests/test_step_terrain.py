"""Measured curb and trajectory regressions; not physical traversal proof."""
import numpy as np

from go2_rescue.step_terrain import CurbObservation, raised_swing


def test_step_requires_both_upper_plateau_and_front_face_over_multiple_scans():
    y = np.linspace(-.25, .25, 11)
    top = np.column_stack([.8+.1*y, y, np.full(len(y), .22)])
    face = np.column_stack([np.full(len(y), .5), y, np.linspace(.05,.19,len(y))])
    curb = CurbObservation([0.,0.],[1.,0.],0.)
    for _ in range(4):
        curb.observe(face,[0.,0.])
    assert not curb.ready  # A wall alone is not a supported step.
    for _ in range(2):
        curb.observe(np.vstack([face,top]),[0.,0.])
    assert not curb.ready
    curb.observe(np.vstack([face,top]),[0.,0.])
    assert curb.ready and abs(curb.edge-.5)<.001
    assert curb.height([.4,0.]) == 0. and abs(curb.height([.6,0.])-.22)<.001
    other = CurbObservation([0.,0.],[1.,0.],0.)
    for _ in range(5):
        other.observe(np.vstack([face,top])+[0.,0.,.25],[0.,0.])
    assert not other.ready  # High obstacles are not accepted as this capability.


def test_high_step_swing_lifts_before_translating_and_has_zero_endpoint_rates():
    start,end=np.array([0.,0.,.022]),np.array([.22,0.,.242])
    evaluate=raised_swing(start,end,.4)
    for t,expected in [(0.,start),(.4,end)]:
        position,velocity,acceleration=evaluate(t)
        assert np.allclose(position,expected)
        assert np.allclose(velocity,0.) and np.allclose(acceleration,0.)
    assert evaluate(.14)[0][2]>.29 and abs(evaluate(.14)[0][0])<1e-6
    assert evaluate(.30)[0][0]>.219 and evaluate(.30)[0][2]>.29
