import json,pathlib,tempfile,unittest
from analyze import read_packets,rate_limited

class CausalSelectionTest(unittest.TestCase):
    def test_future_stale_wrong_team_and_tracking_priority(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder=pathlib.Path(tmp);epoch=1000*10**9
            (folder/'source.json').write_text(json.dumps({'epoch_ns':epoch,'offset':0}))
            (folder/'source_frames.csv').write_text('published\n1\n1\n')
            def header(t):
                ns=epoch+round(t*1e9);return {'stamp':{'sec':ns//10**9,'nanosec':ns%10**9}}
            def radar(t,tracked=False):
                x=[0.]*6;y=[0.]*6
                if tracked:x[0]=194.;y[0]=361.
                return {'topic':'/radar_map','receipt_ns':epoch+round(t*1e9),
                        'message':{'header':header(t-.05),'red_x':x,'red_y':y}}
            def prior(t,source_t,team=1):
                return {'topic':'/prior_predictions','receipt_ns':epoch+round(t*1e9),
                    'message':{'header':header(source_t),'model_enabled':True,
                        'predictions':[{'slot_idx':0,'valid':True,'team_id':team,'prior_world_x':2.,'prior_world_z':3.}]}}
            es=[radar(.1),radar(1.),prior(1.1,1.),radar(1.2),radar(1.3,True),
                radar(2.),prior(2.1,2.,2),radar(2.2),radar(419.5)]
            (folder/'events.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in es))
            streams,stats=read_packets(folder)
            self.assertEqual([(p['t'],p['source']) for p in streams['A'][0]],[(1.3,'tracking')])
            self.assertEqual([(p['t'],p['source']) for p in streams['B'][0]],[(1.2,'guess'),(1.3,'tracking')])
            self.assertEqual(stats['guess_packets'],1)
            sampled=rate_limited(streams,folder,10)
            self.assertTrue(sampled['A'][0])
            self.assertTrue(all(1.3<=p['t']<2 for p in sampled['A'][0]))
            self.assertFalse(any(p['t']>=2 for p in sampled['B'][0]))

if __name__=='__main__':unittest.main()
