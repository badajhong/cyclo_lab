"""Run each NEW candidate independently; persist logs, exit codes and scope.

Run inside the existing container after inspecting GPU usage. Does not rerun
pilot IDs and never changes the fixed candidate manifest or assignments.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--manifest',type=Path,default=ROOT/'scenarios/make_toast_v2_batch01/manifest.json')
p.add_argument('--output_root',type=Path,help='New directory, must not exist')
p.add_argument('--ids',nargs='+',help='Optional subset of new IDs')
p.add_argument('--python',default='/isaac-sim/python.sh')
p.add_argument('--timeout',type=float,default=600)
p.add_argument('--steps',type=int,default=50)
p.add_argument('--static_only',action='store_true',help='No simulator; records static checks without promoting runtime status')
p.add_argument('--audit',type=Path,help='Only run IDs NOT_EXCLUDED in this matching evidence audit; record skipped IDs')
a=p.parse_args()
if a.timeout<=0 or a.steps<1:p.error('Positive timeout/steps required')
a.manifest=a.manifest.resolve();raw=a.manifest.read_bytes();manifest=json.loads(raw)
if manifest['schema_version']!=2:p.error('Batch runner accepts new fixed-pose manifests, not pilot manifests')
ids=[s['id'] for s in manifest['scenarios']]
if a.ids:
    if len(set(a.ids))!=len(a.ids) or set(a.ids)-set(ids):p.error('Duplicate or unknown IDs')
    ids=a.ids
audit_rows=None
if a.audit:
    audit=json.loads(a.audit.read_text())
    if audit['manifest_sha256']!=hashlib.sha256(raw).hexdigest():p.error('Audit does not match manifest hash')
    audit_rows={r['id']:r for r in audit['results']}
if a.output_root:
    out=a.output_root.resolve();out.mkdir(parents=True,exist_ok=False)
else:
    (ROOT/'logs').mkdir(exist_ok=True)
    out=Path(tempfile.mkdtemp(prefix='scenario_batch.',dir=ROOT/'logs'));out.chmod(0o755)
summary={'manifest':str(a.manifest),'manifest_sha256':hashlib.sha256(raw).hexdigest(),
 'mode':'STATIC_ONLY' if a.static_only else 'LAYOUT_RESET_ONLY','results':[],
 'reachability':'NOT_PROVEN','task_success':'NOT_TESTED','all_requested_ids_finished':False}
print('Results:',out,flush=True)
for sid in ids:
    if audit_rows is not None and audit_rows.get(sid,{}).get('status')!='NOT_EXCLUDED':
        row={'id':sid,'exit_code':None,'status':'NOT_RUN_AUDIT_BLOCKED','runtime_verified':False,
             'audit':audit_rows.get(sid),'reachability':'NOT_PROVEN','task_success':'NOT_TESTED'}
        summary['results'].append(row)
        (out/(sid+'.result.json')).write_text(json.dumps(row,indent=2)+'\n')
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(sid,row['status'],flush=True)
        continue
    cmd=[sys.executable if a.static_only else a.python,str(ROOT/'scripts/tools/check_make_toast_scenarios.py'),
      '--scenario_id',sid,'--scenario_manifest',str(a.manifest)]
    if a.static_only:cmd+=['--static']
    else:cmd+=['--headless','--snapshots','--steps',str(a.steps),'--output_dir',str(out/sid)]
    start=time.monotonic();reason=None
    with (out/(sid+'.log')).open('x') as log:
        log.write('COMMAND '+json.dumps(cmd)+'\n');log.flush()
        try:code=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=a.timeout).returncode
        except subprocess.TimeoutExpired:code=124;reason='timeout of this child process only'
        except OSError as error:code=127;reason=repr(error)
        log.write('\nEXIT_CODE '+str(code)+'\n')
    report_path=out/sid/'report.json';checked=False;error=None
    if report_path.is_file():
        try:
            r=json.loads(report_path.read_text())
            labels=[s['label'] for s in r.get('samples',[])]
            checked=(r.get('scenario_id')==sid and r.get('runtime_reset')=='PASS' and
                     all(label in labels for label in ('initial','settled_0','reset_0','settled_1','reset_1')))
            error=r.get('error')
        except (ValueError,KeyError,TypeError) as exc:error=repr(exc)
    passed=code==0 and (a.static_only or checked)
    row={'id':sid,'exit_code':code,'seconds':time.monotonic()-start,'command':cmd,
         'status':('STATIC_CHECK_COMPLETED' if a.static_only else 'LAYOUT_RESET_VERIFIED') if passed else 'FAILED_OR_INCOMPLETE',
         'runtime_verified':bool(passed and not a.static_only),'reason':reason,'report_error':error,
         'reachability':'NOT_PROVEN','task_success':'NOT_TESTED'}
    summary['results'].append(row)
    (out/(sid+'.result.json')).write_text(json.dumps(row,indent=2)+'\n')
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(sid,row['status'],'exit',code,flush=True)
summary['all_requested_ids_finished']=len(summary['results'])==len(ids)
summary['verified_layout_count']=sum(r['runtime_verified'] for r in summary['results'])
summary['failed_count']=sum(r['status']=='FAILED_OR_INCOMPLETE' for r in summary['results'])
summary['skipped_count']=sum(r['status']=='NOT_RUN_AUDIT_BLOCKED' for r in summary['results'])
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
sys.exit(1 if summary['failed_count'] or summary['skipped_count'] else 0)
