"""The task metric requires an ordered lift, transfer, and sustained release."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from bottle_sim_rollout import Stages


def observation(t, pos=(0, 0, .1), contacts=0, box=False, closed=False):
    return dict(time=t, bottle_pos=list(pos), bottle_velocity=[0]*6,
                hand_distance=.05, hand_contacts=contacts, on_box=box,
                state=[0]*6 + ([.8]*6 if closed else [0]*6))


def test_contact_and_box_proximity_are_not_success():
    stages = Stages(observation(0))
    for t in range(1, 10):
        stages.update(observation(t, contacts=1, box=True, closed=True))
    assert stages.events['final_success'] is None
    assert stages.events['grasp_lift'] is None


def test_success_requires_stable_release_and_is_revoked_after_fall():
    stages = Stages(observation(0))
    stages.update(observation(.1, (0, 0, .2), contacts=1, closed=True))
    stages.update(observation(.2, (.2, 0, .2), contacts=1, closed=True))
    stages.update(observation(.3, (.2, 0, .2), box=True))
    stages.update(observation(1., (.2, 0, .2), box=True))
    assert stages.events['final_success'] is None
    stages.update(observation(1.4, (.2, 0, .2), box=True))
    assert stages.events['final_success'] is not None
    stages.update(observation(1.5, (.2, 0, .1)))
    assert stages.events['final_success'] is None
