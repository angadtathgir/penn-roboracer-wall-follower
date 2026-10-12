import csv, sys
import numpy as np
for name in sys.argv[1:]:
    R=list(csv.DictReader(open(f'lg_{name}_0.csv')))
    d={k:np.array([float(r[k]) for r in R]) for k in R[0]}
    m=d['t']>8; d={k:v[m] for k,v in d.items()}
    thr=d['thr']; n=len(thr)
    ev=[]; i=0
    while i<n:
        if thr[i]==0:
            j=i
            while j+1<n and thr[j+1]==0: j+=1
            ev.append((d['t'][i], d['t'][j]-d['t'][i], d['v'][i], d['v_used'][i], d['v_target'][i], d['free'][i], abs(d['steer'][i]), d['x'][i], d['y'][i]))
            i=j+1
        else: i+=1
    ev=np.array(ev)
    T=d['t'][-1]-d['t'][0]
    print(f'== {name}: {len(ev)} brake events in {T:.1f}s, short(<60ms): {(ev[:,1]<0.06).sum()}, time braking {np.sum(thr==0)/n*100:.1f}%')
    print(' t     dur    v   v_used  v_t   free  |st|   x     y')
    for e in ev[:30]: print(' '.join(f'{x:5.2f}' for x in e))
    print('mean |v_used-v|', np.abs(d['v_used']-d['v']).mean().round(3), ' mean v_used-v at brake starts', (ev[:,3]-ev[:,2]).mean().round(3))
