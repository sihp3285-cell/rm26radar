import unittest
from rules import simulate, classify, intervals, duration, unlocks

def packet(t,state):return {'t':t,'state':state}

class RulesTest(unittest.TestCase):
    def test_official_example(self):
        p=[packet(round(.2*(i+1),8),'accurate') for i in range(15)]
        p += [packet(round(3.2+.2*i,8),'wrong') for i in range(5)]
        p += [packet(5.2,'accurate'),packet(5.4,'accurate')]
        tl=simulate(p,end=5.4)
        observed=[r['P'] for r in tl if r['t']>=3]
        self.assertEqual(observed,[120,119.2,117.6,115.2,112,108,103.2,97.6,98.6,100.6])
    def test_revival_does_not_freeze_timeout(self):
        p=[packet(round(1+.1*i,8),'accurate') for i in range(17)]
        tl=simulate(p,[2.7]);self.assertAlmostEqual(duration(intervals(tl)),5.8)
        self.assertEqual(next(r for r in tl if r['t']==2.7)['P'],150)
        self.assertEqual(next(r for r in tl if abs(r['t']-8.1)<1e-8)['P'],97.2)
        self.assertEqual(tl[-1]['P'],0)
    def test_mixed_positive_and_reset(self):
        tl=simulate([packet(.1,'accurate'),packet(.2,'semi'),packet(.3,'wrong'),packet(.4,'semi')],end=.4)
        self.assertEqual([r['P'] for r in tl],[0,1,2.5,1.7,2.2])
    def test_thresholds(self):
        self.assertEqual([classify(d) for d in [0,.799999,.8,1.599999,1.6]],['accurate','accurate','semi','semi','wrong'])
    def test_empty_and_long_tail(self):
        self.assertEqual(duration(intervals(simulate([]))),0)
        self.assertEqual(simulate([])[-1]['t'],420)
    def test_revival_between_receipts_and_same_time(self):
        p=[packet(.1,'accurate'),packet(2.1,'accurate')]
        tl=simulate(p,[.8],end=2.1)
        self.assertEqual([round(r['t'],2) for r in tl if r['state']=='interrupt'],[.6,1.1,1.6,2.1])
    def test_union_unlock_not_robot_sum(self):
        self.assertEqual(duration([(0,40),(20,60)]),60)
        self.assertEqual(unlocks([(0,40),(20,60),(100,160)]),[60,160])

if __name__=='__main__': unittest.main()
