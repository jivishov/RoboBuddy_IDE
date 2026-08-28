from pathlib import Path
import re, types, sys, tempfile, os, time
text=Path(__file__).parents[1].joinpath('src/profiles.js').read_text()

def get(name):
    m=re.search(rf'const {name}=`(.*?)`;',text,re.S)
    if not m: raise RuntimeError(name)
    return m.group(1).replace('\\n','\n').replace('\\"','"')

class Config:
    def __init__(self,*args,**kw): self.__dict__.update(kw)
class Robot:
    events=[]
    def __init__(self,cfg=None): self.cfg=cfg; self.state={}; self.connected=False
    def connect(self,*args,**kwargs): self.connected=True
    def send_action(self,a):
        assert self.connected
        d={str(k):float(v) for k,v in dict(a).items()}
        self.state.update(d); Robot.events.append(d); return d
    def get_observation(self): return dict(self.state)
    def disconnect(self): self.connected=False

def install():
    for n in list(sys.modules):
        if n=='lerobot' or n.startswith('lerobot.'): sys.modules.pop(n,None)
    lr=types.ModuleType('lerobot'); lr.__path__=[]
    robots=types.ModuleType('lerobot.robots'); robots.__path__=[]
    so=types.ModuleType('lerobot.robots.so_follower'); oa=types.ModuleType('lerobot.robots.openarm_follower'); bi=types.ModuleType('lerobot.robots.bi_openarm_follower'); lk=types.ModuleType('lerobot.robots.lekiwi')
    so.SO101Follower=Robot; so.SO101FollowerConfig=Config
    oa.OpenArmFollowerConfigBase=Config; oa.OpenArmFollowerConfig=Config; oa.OpenArmFollower=Robot
    bi.BiOpenArmFollower=Robot; bi.BiOpenArmFollowerConfig=Config
    lk.LeKiwiClient=Robot; lk.LeKiwiClientConfig=Config
    lr.robots=robots; robots.so_follower=so; robots.openarm_follower=oa; robots.bi_openarm_follower=bi; robots.lekiwi=lk
    for n,m in [('lerobot',lr),('lerobot.robots',robots),('lerobot.robots.so_follower',so),('lerobot.robots.openarm_follower',oa),('lerobot.robots.bi_openarm_follower',bi),('lerobot.robots.lekiwi',lk)]: sys.modules[n]=m

orig_sleep=time.sleep; time.sleep=lambda _:None
try:
    specs=[('openarm','openArmMain','openArmTrajectories','openArmConfig',11),('so101','so101Main','so101Traj','so101Config',8),('lekiwi','lekiwiMain','lekiwiTraj','lekiwiConfig',6)]
    for profile,main,traj,cfg,expected in specs:
        install(); Robot.events=[]
        with tempfile.TemporaryDirectory() as td:
            Path(td,'main.py').write_text(get(main)); Path(td,'trajectories.py').write_text(get(traj)); Path(td,'robot_config.py').write_text(get(cfg))
            old=os.getcwd(); os.chdir(td); sys.path.insert(0,td)
            try: exec(compile(Path('main.py').read_text(),'main.py','exec'),{'__name__':'__main__','__file__':'main.py'})
            finally:
                sys.path.remove(td); os.chdir(old); sys.modules.pop('trajectories',None); sys.modules.pop('robot_config',None)
        assert len(Robot.events)==expected,(profile,len(Robot.events),expected)
        print(profile,'starter executes:',len(Robot.events),'send_action calls')
finally: time.sleep=orig_sleep
