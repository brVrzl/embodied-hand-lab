"""Offline MuJoCo bottle task; native actions are radians + six closure fractions."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from rh56_sim.model import RH56_CHANNELS

ROOT = Path(__file__).resolve().parents[2]

def _s(values):
    return ' '.join(str(float(v)) for v in values)

def _camera(parent, name, position, target, fovy=50):
    pos, target = np.asarray(position), np.asarray(target)
    z = pos-target; z /= np.linalg.norm(z)
    x = np.cross([0,0,1], z); x /= np.linalg.norm(x)
    y = np.cross(z,x)
    ET.SubElement(parent,'camera',name=name,pos=_s(pos),xyaxes=_s(np.r_[x,y]),fovy=str(fovy))

class BottleScene:
    def __init__(self, config_path=None):
        self.config=json.loads(Path(config_path or ROOT/'assets/bottle_sim/scene.json').read_text())
        c=self.config
        tree=ET.parse(ROOT/'assets/jaka_rh56_visual_coacd.xml'); root=tree.getroot()
        for mesh in root.findall('asset/mesh'):
            mesh.set('file', str(ROOT/'assets'/mesh.get('file')))
        root.find('option').set('timestep',str(1/600))
        root.find('option').set('integrator','implicitfast')
        world=root.find('worldbody')
        world.find("geom[@name='floor']").set('pos','0 0 -0.05')
        ET.SubElement(world,'geom', name='table',type='box',pos=_s([0,.35,-.025]),size='.6 .6 .025',rgba='.65 .47 .29 1')
        ET.SubElement(world,'geom', name='cardboard_box',type='box',pos=_s(c['box_center']),size=_s(c['box_halfsize']),rgba='.72 .69 .55 1')
        bottle=ET.SubElement(world,'body',name='bottle',pos=_s(c['bottle_center']))
        ET.SubElement(bottle,'freejoint',name='bottle_free')
        ET.SubElement(bottle,'geom',name='bottle_geom',type='cylinder',size=_s([c['bottle_radius'],c['bottle_height']/2]),mass=str(c['bottle_mass']),rgba='.35 .65 .85 1',friction='.8 .005 .0001',condim='4')
        ET.SubElement(bottle,'geom',name='bottle_cap',type='cylinder',pos=_s([0,0,c['bottle_height']/2+.006]),size='.015 .006',mass='.005',rgba='.9 .95 .95 1')
        _camera(world,'workspace',c['workspace_camera_position'],c['workspace_camera_target'],c['workspace_fovy'])
        hand=root.find(".//body[@name='rh56_R_hand_base_link']")
        ET.SubElement(hand,'camera',name='wrist',pos=_s(c['wrist_camera_position_local']),xyaxes=_s(c['wrist_camera_xyaxes_local']),fovy='65')
        # Existing arm position servo is a kinematic preview gain (40). Use an
        # explicitly documented finite-force servo for dynamic target replay.
        for act in root.findall('actuator/position')[:6]:
            act.set('kp','500');act.set('kv','30');act.set('forcelimited','true');act.set('forcerange','-60 60')
        visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='640',offheight='480')
        self.model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        self.data=mujoco.MjData(self.model)
        self.joint_names=[f'jaka_joint_{i}' for i in range(1,7)]+[x.joint for x in RH56_CHANNELS]
        self.jids=np.array([self.model.joint(n).id for n in self.joint_names])
        self.qids=self.model.jnt_qposadr[self.jids]
        self.aids=np.array([self.model.actuator(n+'_act').id for n in self.joint_names])
        self.hand_ranges=self.model.jnt_range[self.jids[6:]].copy()
        self.lower=np.r_[self.model.jnt_range[self.jids[:6],0],np.zeros(6)]
        self.upper=np.r_[self.model.jnt_range[self.jids[:6],1],np.ones(6)]
        self.renderer=None
        self.last_action={}
        self.reset()

    def native_to_ctrl(self, native):
        a=np.asarray(native,dtype=float)
        return np.r_[a[:6],self.hand_ranges[:,0]+a[6:]*(self.hand_ranges[:,1]-self.hand_ranges[:,0])]

    def state(self):
        q=self.data.qpos[self.qids]
        return np.r_[q[:6],(q[6:]-self.hand_ranges[:,0])/(self.hand_ranges[:,1]-self.hand_ranges[:,0])].astype(np.float32)

    def reset(self, instance=None):
        instance=instance or {}
        mujoco.mj_resetData(self.model,self.data)
        initial=np.array(instance.get('initial_state',self.config['initial_state']),dtype=float)
        self.data.qpos[self.qids]=self.native_to_ctrl(initial)
        # Equality dependent joints initialize consistently; subsequent motion
        # exclusively uses ctrl + mj_step, never qpos replay.
        for eq in range(self.model.neq):
            j1,j2=self.model.eq_obj1id[eq],self.model.eq_obj2id[eq]
            x=self.data.qpos[self.model.jnt_qposadr[j2]]
            self.data.qpos[self.model.jnt_qposadr[j1]]=sum(self.model.eq_data[eq,k]*x**k for k in range(5))
        qadr=self.model.joint('bottle_free').qposadr[0]
        pos=np.array(instance.get('bottle_pos',self.config['bottle_center']),dtype=float)
        if 'bottle_xy' in instance: pos[:2]=instance['bottle_xy']
        self.data.qpos[qadr:qadr+3]=pos
        yaw=float(instance.get('bottle_yaw',0));self.data.qpos[qadr+3:qadr+7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
        self.data.ctrl[self.aids]=self.native_to_ctrl(initial)
        mujoco.mj_forward(self.model,self.data)
        self.last_action={}
        return self.snapshot()

    def step(self, native):
        raw=np.asarray(native,dtype=float)
        if raw.shape!=(12,) or not np.all(np.isfinite(raw)): raise ValueError('native action requires 12 finite values')
        executed=np.clip(raw,self.lower,self.upper)
        self.last_action={'raw':raw.tolist(),'executed':executed.tolist(),'projected':(raw!=executed).tolist()}
        self.data.ctrl[self.aids]=self.native_to_ctrl(executed)
        mujoco.mj_step(self.model,self.data,nstep=20)
        return self.snapshot()

    def snapshot(self):
        b=self.data.body('bottle');h=self.data.body('rh56_R_hand_base_link')
        hand_center=h.xpos+h.xmat.reshape(3,3)@np.array([0,0,-.10])
        bid=self.model.body('bottle').id
        contacts=0;on_box=False
        for con in self.data.contact:
            g1,g2=int(con.geom1),int(con.geom2)
            if self.model.geom_bodyid[g1]==bid or self.model.geom_bodyid[g2]==bid:
                other=g2 if self.model.geom_bodyid[g1]==bid else g1
                name=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_GEOM,other) or ''
                contacts+=int(name.startswith('rh56'))
                on_box |= name=='cardboard_box'
        box=np.asarray(self.config['box_center']);half=np.asarray(self.config['box_halfsize'])
        axis_z=b.xmat.reshape(3,3)[2,2]
        extent_z=self.config['bottle_height']/2*abs(axis_z)+self.config['bottle_radius']*np.sqrt(max(0.,1-axis_z**2))
        bottom=b.xpos[2]-extent_z
        on_box = on_box and bool(np.all(np.abs(b.xpos[:2]-box[:2]) < half[:2])) and abs(bottom-(box[2]+half[2])) < .012
        vadr=self.model.joint('bottle_free').dofadr[0]
        return {'time':float(self.data.time),'state':self.state().tolist(),'bottle_pos':b.xpos.copy().tolist(),'bottle_velocity':self.data.qvel[vadr:vadr+6].copy().tolist(),'hand_distance':float(np.linalg.norm(hand_center-b.xpos)),'hand_contacts':contacts,'on_box':bool(on_box),'contacts':int(self.data.ncon)}

    diagnostics=snapshot

    def render(self,camera='workspace'):
        if self.renderer is None: self.renderer=mujoco.Renderer(self.model,height=480,width=640)
        opt=mujoco.MjvOption();opt.geomgroup[3]=0
        self.renderer.update_scene(self.data,camera=camera,scene_option=opt)
        return self.renderer.render().copy()

    def observe(self):
        return {'state':self.state(),'workspace':self.render('workspace'),'wrist':self.render('wrist')}

    def close(self):
        if self.renderer is not None:self.renderer.close();self.renderer=None
