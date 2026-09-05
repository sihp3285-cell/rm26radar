#!/usr/bin/env python3
"""Build a fixed-R control in log only; production sources/install stay untouched."""
import difflib,hashlib,json,pathlib,shlex,shutil,subprocess
import yaml
from run import ROOT,BASE

CMP=BASE/'comparison_20260905'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    CMP.mkdir(exist_ok=False)
    files={str(p.relative_to(ROOT)):sha(p) for base in ['src','configs','generated','install','models'] for p in (ROOT/base).rglob('*') if p.is_file()}
    (CMP/'production_before.json').write_text(json.dumps(files,indent=2))
    build=ROOT/'build/tensorrt_detect'
    original=ROOT/'src/tensorrt_detect/src/core/posesolver.cpp'
    source=original.read_text()
    edits={'float covariance_xx = minimum_variance + sigma_squared * g_xx;':'float covariance_xx = 1.0f;',
           'float covariance_xz = sigma_squared * g_xz;':'float covariance_xz = 0.0f;',
           'float covariance_zz = minimum_variance + sigma_squared * g_zz;':'float covariance_zz = 1.0f;',
           'if (result.surface_discontinuity) {':'if (false && result.surface_discontinuity) {'}
    patched=source
    for a,b in edits.items():
        assert patched.count(a)==1,a
        patched=patched.replace(a,b)
    target=CMP/'posesolver_fixed.cpp';target.write_text(patched)
    (CMP/'fixed_source.patch').write_text(''.join(difflib.unified_diff(source.splitlines(True),patched.splitlines(True),fromfile=str(original),tofile=str(target))))
    command=next(x for x in json.loads((build/'compile_commands.json').read_text()) if x['file']==str(original))
    args=shlex.split(command['command']);obj=CMP/'posesolver.cpp.o'
    args[args.index('-o')+1]=str(obj);args[args.index(str(original))]=str(target)
    subprocess.run(args,cwd=build,check=True)
    archive=CMP/'libtensorrt_detect_core_fixed.a';shutil.copy2(build/'libtensorrt_detect_core.a',archive)
    subprocess.run(['ar','r',str(archive),str(obj)],check=True)
    link=shlex.split((build/'CMakeFiles/pose_node_component.dir/link.txt').read_text())
    library=CMP/'libpose_node_fixed.so'
    link[link.index('-o')+1]=str(library)
    link[link.index('libtensorrt_detect_core.a')]=str(archive)
    subprocess.run(link,cwd=build,check=True)
    params=yaml.safe_load((ROOT/'src/tensorrt_detect/config/ros2_params.yaml').read_text())
    params['pose_node']['ros__parameters']['projection_selector_enabled']=False
    (CMP/'armor.yaml').write_text(yaml.safe_dump(params,allow_unicode=True))
    prior=yaml.safe_load((ROOT/'src/position_prior/config/position_prior.yaml').read_text())
    assert list(prior)==['position_prior_node']
    prior['position_prior_node']['ros__parameters']['navgrid_path']=''
    (CMP/'nav_off.yaml').write_text(yaml.safe_dump(prior,allow_unicode=True))
    manifest={'alignment_sha256':sha(BASE/'alignment.json'),'fixed_source_changes':edits,
        'variants':{'baseline':{},'armor':{'params_file':str(CMP/'armor.yaml')},
                    'fixed':{'pose_library':str(library)},'nav_off':{'prior_params_file':str(CMP/'nav_off.yaml')}},
        'artifacts':{p.name:sha(p) for p in [library,target,CMP/'armor.yaml',CMP/'nav_off.yaml']},
        'fixed_semantics':'R=identity for normal valid five-ray projections, no terrain inflation; preserve original invalid-ray fallbacks and selector use of R',
        'nav_off_semantics':'No navgrid loaded; blind-zone prior consequently disabled; stay scale remains 0.8',
        'armor_semantics':'Existing projection_selector_enabled=false: prefer armor, fallback to healthy car if armor invalid'}
    (CMP/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    print('READY',CMP)

if __name__=='__main__':main()
