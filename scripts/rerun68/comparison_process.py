#!/usr/bin/env python3
"""Process completed recordings in order; never read an open recording."""
import json,time
from comparison_run import PLAN
from comparison_analyze import analyze_run,folder_for
from comparison_measurements import inspect
from comparison_build import CMP

def main():
    for key,v,rep in PLAN:
        folder=folder_for(key,v,rep)
        while not (folder/'export_audit.json').exists():time.sleep(5)
        assert json.loads((folder/'run.json').read_text())['status']=='recorded'
        analyze_run(key,v,rep)
        inspect(key,v,rep)
        print('PROCESSED',folder.name,flush=True)
    (CMP/'processed.json').write_text(json.dumps({'status':'all_processed','runs':len(PLAN)}))

if __name__=='__main__':main()
