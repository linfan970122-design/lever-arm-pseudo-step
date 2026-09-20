#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import csv, glob, os, math, sys
D = os.path.join(os.environ.get('P2_OUT', 'out'), 'csv')

def q(v,p):
    if not v: return float('nan')
    s=sorted(v); k=(len(s)-1)*p; i=int(k); f=k-i
    return s[i] if i+1>=len(s) else s[i]*(1-f)+s[i+1]*f

# ---- (a) E3b event
print('=== (a) E3b 0906 15:26 bag, largest |dpsi| events')
p=os.path.join(D,'run_20260906_2026-09-06-15-26-15.csv')
rows=[r for r in csv.DictReader(open(p)) if r['dpsi_deg']]
rows.sort(key=lambda r:-abs(float(r['dpsi_deg'])))
hdr=['t','dpsi_deg','dp_pub','dp_ant','pred_jump','resid','theta_body_deg','along','across','cov35','quality','sats','fixed','hdg_deg','dpsi_imu_deg','gz_max']
print(','.join(hdr))
import datetime
for r in rows[:6]:
    r2=dict(r); r2['t']=datetime.datetime.fromtimestamp(float(r['t'])).strftime('%H:%M:%S.%f')[:-3]
    print(','.join(str(r2[h]) for h in hdr))

# ---- (b) 0912 bag with /rtk/fix: dp_fix vs dp_ant vs dp_pub
print()
print('=== (b) bags with /rtk/fix: correlation dp_fix vs dp_ant / dp_pub')
for f in sorted(glob.glob(D+'/*.csv')):
    rows=[r for r in csv.DictReader(open(f)) if r['dp_fix'] and r['dp_ant'] and r['dp_pub']]
    if not rows: continue
    a=[float(r['dp_fix']) for r in rows]; b=[float(r['dp_ant']) for r in rows]; c=[float(r['dp_pub']) for r in rows]
    def corr(u,v):
        n=len(u); mu=sum(u)/n; mv=sum(v)/n
        su=math.sqrt(sum((x-mu)**2 for x in u)); sv=math.sqrt(sum((x-mv)**2 for x in v))
        if su==0 or sv==0: return float('nan')
        return sum((u[i]-mu)*(v[i]-mv) for i in range(n))/(su*sv)
    mad_ant=q([abs(a[i]-b[i]) for i in range(len(a))],.5)
    mad_pub=q([abs(a[i]-c[i]) for i in range(len(a))],.5)
    print('%-46s n=%5d  corr(fix,ant)=%.6f corr(fix,pub)=%.4f  med|fix-ant|=%.5f med|fix-pub|=%.5f'
          %(os.path.basename(f),len(rows),corr(a,b),corr(a,c),mad_ant,mad_pub))
    if len(rows)>10:
        big=sorted(rows,key=lambda r:-abs(float(r['dpsi_deg'])))[:4]
        for r in big:
            print('      dpsi=%8s dp_pub=%7s dp_ant=%7s dp_fix=%7s' % (r['dpsi_deg'],r['dp_pub'],r['dp_ant'],r['dp_fix']))

# ---- pooled stats
print()
print('=== pooled over all processed bags')
allv=[];cat={'straight':[],'turn':[],'slow':[],'na':[]};cati={'straight':[],'turn':[],'na':[]}
ngt={5:0,10:0,20:0,45:0}; nfr=0; gz=[]; hdok0=0; mxpub=0; mxant=0
big=[]
for f in sorted(glob.glob(D+'/*.csv')):
    for r in csv.DictReader(open(f)):
        if not r['dpsi_deg']: continue
        v=abs(float(r['dpsi_deg'])); nfr+=1
        allv.append(v); cat[r['cls']].append(v); cati[r['cls_imu']].append(v)
        for k in ngt:
            if v>k: ngt[k]+=1
        if r['hd_ok']=='0': hdok0+=1
        dp=float(r['dp_pub']); da=float(r['dp_ant'])
        mxpub=max(mxpub,dp); mxant=max(mxant,da)
        if r['gz_max']: gz.append(float(r['gz_max']))
        if v>20: big.append((v,dp,da,r['dpsi_imu_deg'],r['cls'],os.path.basename(f),r['t']))
print('frames with dpsi: %d'%nfr, 'hd_ok=0 frames: %d'%hdok0)
print('ALL      med %.3f p90 %.3f p99 %.3f max %.3f'%(q(allv,.5),q(allv,.9),q(allv,.99),max(allv)))
for k in ['straight','turn','slow','na']:
    if cat[k]: print('%-9s n %7d med %.3f p90 %.3f p99 %.3f max %.3f'%(k,len(cat[k]),q(cat[k],.5),q(cat[k],.9),q(cat[k],.99),max(cat[k])))
print('-- IMU-based classification (independent of RTK heading)')
for k in ['straight','turn','na']:
    if cati[k]: print('%-9s n %7d med %.3f p90 %.3f p99 %.3f max %.3f'%(k,len(cati[k]),q(cati[k],.5),q(cati[k],.9),q(cati[k],.99),max(cati[k])))
print('counts >5/10/20/45 deg:',ngt)
print('max dp_pub %.4f  max dp_ant %.4f'%(mxpub,mxant))
if gz:
    print('gyro |wz| p99 %.4f rad/s -> %.3f deg per 0.2 s frame ; max %.4f rad/s -> %.3f deg'
          %(q(gz,.99),math.degrees(q(gz,.99))*0.2,max(gz),math.degrees(max(gz))*0.2))
print()
print('=== frames with |dpsi|>20 deg (top 25 by |dpsi|)')
big.sort(reverse=True)
for x in big[:25]:
    print('  |dpsi|=%7.3f dp_pub=%7.4f dp_ant=%7.4f dpsi_imu=%8s %-8s %s t=%s'%x)
print('total frames >20 deg:',len(big))
