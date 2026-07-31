from __future__ import annotations
import re, subprocess
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'native/jaka_minimal_joint_probe'
BUILD=ROOT/'build/jaka_minimal_joint_probe'
BINARY=BUILD/'jaka_gate3c_plan_probe'
MOTION=BUILD/'jaka_gate3c_motion_probe'
FIVE_PLAN=BUILD/'jaka_gate3c_5deg_plan_probe'
pytestmark = pytest.mark.skipif(
    not sys.platform.startswith("linux"),
    reason="the JAKA vendor SDK probe is Linux-only",
)

def setup_module() -> None:
    subprocess.run(['cmake','-S',str(SOURCE),'-B',str(BUILD),'-DCMAKE_BUILD_TYPE=Release'],check=True)
    subprocess.run(['cmake','--build',str(BUILD),'-j2'],check=True)

def test_default_is_nonconnecting():
    r=subprocess.run([str(BINARY)],text=True,capture_output=True,check=True)
    assert '"connection_opened":false' in r.stdout and '"commands_issued":0' in r.stdout

def test_motion_help_is_nonconnecting_even_with_vendor_selected():
    r=subprocess.run(
        [str(MOTION),'--backend','vendor','--help'],
        text=True,capture_output=True,check=True,
    )
    assert 'Usage: jaka_gate3c_motion_probe' in r.stdout
    assert '--physical-hardware' in r.stdout
    assert '--fake-deterministic-clock' in r.stdout
    assert r.stderr == ''

def test_plan_binary_has_no_write_capability():
    symbols=subprocess.run(['nm','-D','--undefined-only',str(BINARY)],text=True,capture_output=True,check=True).stdout
    forbidden=('edg_init','edg_servo','servo_move_enable','joint_move','linear_move','servo_j','servo_p','power_on','enable_robot')
    assert not any(name in symbols for name in forbidden)

def test_septic_trajectory_is_bounded_and_has_stationary_endpoints():
    text=(SOURCE/'trajectory.hpp').read_text()
    assert '35.0 * s4 - 84.0 * s5 + 70.0 * s6 - 20.0 * s7' in text
    # Offline numerical verification of the exact polynomial used by the native plan.
    import numpy as np
    d=np.deg2rad(.25); t=2.; s=np.linspace(0,1,100001)
    p=35*s**4-84*s**5+70*s**6-20*s**7
    v=(140*s**3-420*s**4+420*s**5-140*s**6)*d/t
    a=(420*s**2-1680*s**3+2100*s**4-840*s**5)*d/t**2
    j=(840*s-5040*s**2+8400*s**3-4200*s**4)*d/t**3
    assert p[0]==0 and p[-1]==1
    assert v[0]==v[-1]==a[0]==a[-1]==j[0]==j[-1]==0
    assert max(abs(v))<.005 and max(abs(a))<.010 and max(abs(j))<.040

def test_source_is_arm_only_and_plan_main_has_no_write_calls():
    text='\n'.join(p.read_text().lower() for p in SOURCE.glob('*.*'))
    assert '#include "rh56' not in text and '#include "teledex' not in text and '#include "quest' not in text
    main=(SOURCE/'plan_main.cpp').read_text()
    assert not re.search(r'\b(edg_init|edg_servo_j|servo_move_enable|joint_move|linear_move)\s*\(',main)

def run_motion(tmp_path: Path, *args: str):
    result=tmp_path/'result.json'; csv=tmp_path/'trajectory.csv'
    p=subprocess.run([str(MOTION),'--backend','fake','--fake-deterministic-clock','--result-file',str(result),'--trajectory-csv',str(csv),*args],text=True,capture_output=True)
    import json
    return p,json.loads(result.read_text()),csv

def test_fake_outward_hold_return_lifecycle(tmp_path):
    p,r,csv=run_motion(tmp_path)
    assert p.returncode==0 and r['outcome']=='completed'
    assert r['commands']==r['planned_commands']==551
    assert r['start']==r['return_target']
    assert r['outward_target'][5]-r['start'][5]==pytest.approx(np.deg2rad(.25))
    assert r['peak_tracking_error_rad']<.0005
    assert r['lifecycle'].endswith('disable_servo_move,exit_edg,logout')
    assert len(csv.read_text().splitlines())==552

def test_fake_command_failure_stops_without_automatic_return(tmp_path):
    p,r,_=run_motion(tmp_path,'--fake-command-failure-cycle','20')
    assert p.returncode==2 and r['outcome']=='command_failure'
    assert r['commands']==19
    assert r['lifecycle'].endswith('disable_servo_move,exit_edg,logout')

@pytest.mark.parametrize(('args','outcome'),[
    (('--fake-read-failure-cycle','2'),'edg_read_failure'),
    (('--fake-servo-enable-failure',),'servo_enable_failure'),
    (('--fake-servo-disable-failure',),'servo_disable_failure'),
    (('--fake-observed-delta-rad','0.001'),'first_command_guard'),
])
def test_fake_failure_paths_cleanup(tmp_path,args,outcome):
    p,r,_=run_motion(tmp_path,*args)
    assert p.returncode==2 and r['outcome']==outcome
    assert r['lifecycle'].endswith('exit_edg,logout')

def test_motion_binary_write_surface_is_joint_edg_only():
    symbols=subprocess.run(['nm','-D','--undefined-only',str(MOTION)],text=True,capture_output=True,check=True).stdout
    assert 'edg_servo_j' in symbols and 'servo_move_enable' in symbols and 'edg_init' in symbols
    assert not any(x in symbols for x in ('edg_servo_p','joint_move','linear_move','servo_p','power_on','enable_robot'))

def test_five_degree_plan_is_nonconnecting_by_default_and_write_incapable():
    r=subprocess.run([str(FIVE_PLAN)],text=True,capture_output=True,check=True)
    assert '"connection_opened":false' in r.stdout
    symbols=subprocess.run(['nm','-D','--undefined-only',str(FIVE_PLAN)],text=True,capture_output=True,check=True).stdout
    assert not any(x in symbols for x in ('edg_init','edg_servo','servo_move_enable','joint_move','linear_move','servo_j','servo_p','power_on','enable_robot'))

def test_five_degree_dynamic_threshold_math():
    d=np.deg2rad(5);t=5.;s=np.linspace(0,1,100001)
    v=(140*s**3-420*s**4+420*s**5-140*s**6)*d/t
    hard=np.maximum(np.deg2rad(.75),2.5*np.abs(v)*.150)
    assert np.rad2deg(max(abs(v)))==pytest.approx(2.1875)
    assert np.rad2deg(max(hard))==pytest.approx(.8203125)

def test_five_degree_fake_outward_hold_return_and_instrumentation(tmp_path):
    p,r,csv=run_motion(tmp_path,'--five-degree-profile')
    assert p.returncode==0 and r['outcome']=='completed'
    assert r['timing_clock']=='deterministic_fake'
    assert r['profile']=='joint6_plus_5deg'
    assert r['commands']==r['planned_commands']==1439
    assert r['outward_target'][5]-r['start'][5]==pytest.approx(np.deg2rad(5))
    assert r['tracking_warning_threshold_rad']==pytest.approx(np.deg2rad(.2))
    assert r['dynamic_hard_base_rad']==pytest.approx(np.deg2rad(.75))
    assert r['configured_observation_delay_s']==pytest.approx(.150)
    assert r['tracking_warning_crossings']==0
    assert r['hard_threshold_crossings']==0
    assert r['maximum_non_target_observation_delta_rad']==0
    assert r['lifecycle'].endswith('disable_servo_move,exit_edg,logout')
    header=csv.read_text().splitlines()[0]
    assert 'expected_lag_error_rad' in header
    assert 'dynamic_hard_threshold_rad' in header
    assert len(csv.read_text().splitlines())==1440

@pytest.mark.parametrize(('args','outcome'),[
    (('--fake-tracking-offset-rad','0.02'),'persistent_dynamic_tracking'),
    (('--fake-tracking-growth-rad','0.005'),'rapid_tracking_divergence'),
    (('--fake-non-target-offset-rad','0.002'),'non_target_envelope'),
    (('--fake-tracking-offset-rad','-0.001'),'wrong_direction'),
])
def test_five_degree_fake_safety_aborts_and_cleans_up(tmp_path,args,outcome):
    p,r,_=run_motion(tmp_path,'--five-degree-profile',*args)
    assert p.returncode==2 and r['outcome']==outcome
    assert r['lifecycle'].endswith('disable_servo_move,exit_edg,logout')
    assert r['commands']<r['planned_commands']

def test_five_degree_vendor_execution_requires_all_physical_flags(tmp_path):
    result=tmp_path/'result.json'; csv=tmp_path/'trajectory.csv'
    p=subprocess.run([
        str(MOTION),'--backend','vendor','--five-degree-profile',
        '--robot-ip','192.0.2.1','--edg-state-ip','192.0.2.2',
        '--result-file',str(result),'--trajectory-csv',str(csv),
    ],text=True,capture_output=True)
    assert p.returncode==64
    assert 'all physical confirmations required' in p.stderr
    assert not result.exists() and not csv.exists()

def test_vendor_backend_rejects_fake_deterministic_clock(tmp_path):
    result=tmp_path/'result.json'; csv=tmp_path/'trajectory.csv'
    p=subprocess.run([
        str(MOTION),'--backend','vendor','--fake-deterministic-clock',
        '--result-file',str(result),'--trajectory-csv',str(csv),
    ],text=True,capture_output=True)
    assert p.returncode==64
    assert '--fake-deterministic-clock is forbidden' in p.stderr
    assert not result.exists() and not csv.exists()
