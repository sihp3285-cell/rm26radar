import json,pathlib,tempfile,unittest
from fusion_sender import fusion_frames

class FusionSenderTest(unittest.TestCase):
    def test_latest_frame_clears_slot_and_uses_source_age(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=pathlib.Path(tmp);epoch=1000*10**9
            (folder/'source.json').write_text(json.dumps({'epoch_ns':epoch,'offset':0}))
            def stamp(t):
                ns=epoch+round(t*1e9);return {'sec':ns//10**9,'nanosec':ns%10**9}
            def fused(topic,receive,header,source,valid=True):
                targets=[{'slot_idx':i,'valid':False} for i in range(10)]
                targets[0]={'slot_idx':0,'valid':valid,'team_id':1,'world_x':1.23,'world_z':-2.34,'source':3,'source_stamp':stamp(source)}
                return {'topic':topic,'receipt_ns':epoch+round(receive*1e9),'message':{'header':{'stamp':stamp(header)},'targets':targets}}
            es=[{'topic':'/world_targets','receipt_ns':epoch+100000000,'message':{'header':{'stamp':stamp(.05)},'targets':[{'valid':True,'team_id':1,'observed':True,'is_dead':False,'position_source':2,'world_x':1.23,'world_z':-2.34}]}}]
            for topic in ['/fused_targets']:
                es.extend([fused(topic,.12,.1,.05),fused(topic,.7,.65,.6,False),fused(topic,.91,.9,.51)])
            es.sort(key=lambda e:e['receipt_ns'])
            (folder/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in es))
            streams,_,_=fusion_frames(folder)
            for name in ['fused']:
                ps=streams[name][0];ts=[round(p['t'],1) for p in ps]
                self.assertEqual(ts,[.2,.4,1.0])
                self.assertTrue(all(p['protocol_x_cm']==1166 and p['protocol_y_cm']==873 for p in ps))
                self.assertTrue(all(p['send_time_ns']>=p['receipt_ns'] for p in ps))

if __name__=='__main__':unittest.main()
