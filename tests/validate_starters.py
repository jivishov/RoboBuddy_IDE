from pathlib import Path
import re
text=Path(__file__).parents[1].joinpath('src/profiles.js').read_text()
for marker in ['openArmMain','openArmTrajectories','openArmConfig','so101Main','so101Traj','so101Config','lekiwiMain','lekiwiTraj','lekiwiConfig']:
    m=re.search(rf'const {marker}=`(.*?)`;',text,re.S)
    if not m: raise SystemExit(f'missing {marker}')
    src=m.group(1).replace('\\n','\n').replace('\\"','"')
    compile(src, marker+'.py', 'exec')
print('starter Python syntax: OK')
