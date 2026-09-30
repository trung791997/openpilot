"""clutter2: outcome-labelled stopped tracks entering adopt's hard width; feature screen. Offline log only."""
import numpy as np, core, csv, os, json
Ds = core.load_all()
RC = 1.52
def flag55(D):
  key = D['run_r']; sl, n, rms, sp = core.roll_lsq(key, D['t'], D['d_rel'], 0.5, 0.0, D['t'].min()); sl[n < 6] = np.nan
  va = D['v_ego'] + sl - D['yaw'] * D['y_rel']; fl = np.nan_to_num((np.abs(va) < 1.0 + 0.02 * D['v_ego']).astype(float))
  m = core.roll_mean(key, D['t'], fl, 0.5, 0.0, D['t'].min()); K = key * 1e5 + D['t'] - D['t'].min()
  nn = np.searchsorted(K, K, 'right') - np.searchsorted(K, K - 0.5 - 1e-9, 'left'); return (m > 0.999) & (nn >= 4)
NUM = ['f3_AZIMUTH_EDGE_A_RAD', 'f3_AZIMUTH_EDGE_B_RAD', 'f3_AZIMUTH_EDGE_SIGMA_A_RAW', 'aux_AZIMUTH_EDGE_SIGMA_B_RAW', 'f0_RANGE_SIGMA_RAW',
       'f1_OBJECT_EXISTENCE_PROBABILITY_RAW', 'f1_ANGULAR_WIDTH_DEG', 'u10', 'f2_NORMALIZED_CLOSING_SIGMA_RAW', 'f2_NORMALIZED_CLOSING', 'aux_RANGE_RATIO',
       'aux_REL_VELOCITY_UNCERTAINTY_RAW', 'f2_LIFECYCLE_RAW', 'vrel_u11']
recs = []
for D in Ds:
  r3 = D['rid'][5:8]
  z = np.load(f'npz/bat_{r3}.npz', allow_pickle=True)
  m = z['trk_valid'] > 0; o = np.lexsort((z['trk_t'][m], z['trk_inc'][m], z['trk_tid'][m]))
  X = {}
  for k in z.files:
    if not k.startswith('trk_') or k in ('trk_raw', 'trk_have'): continue
    kk = k[4:]
    if kk in NUM or 'lid' in kk: X[kk] = z[k][m][o].astype(float)
  raw = z['trk_raw'][m][o].astype(np.uint8); have = z['trk_have'][m][o].astype(bool)
  pe = np.load(f'pathend/pe_{r3}.npz')
  t = D['t']; d = D['d_rel']; y = D['y_rel']; v = D['v_ego']
  j = np.clip(np.searchsorted(pe['t'], t, 'right') - 1, 0, None); pxe = pe['x'][j]; pye = pe['y'][j]
  off = np.where(np.isfinite(D['path_y']), y - D['path_y'], y + pye)   # radar sign; beyond path end: flat extrapolation of last point
  beyond = ~np.isfinite(D['path_y'])
  fl = flag55(D)
  stat = np.abs(D['vabs1']) < 1.0 + 0.03 * v
  elig = fl & (np.abs(off) <= 2.0) & (d >= 20) & (d <= 90) & (v > 5)
  forced = None
  if r3 == '297':
    q = np.where((D['tid'] == 42) & (t >= 1824.75) & (t < 1826))[0]; forced = D['inc_id'][q[0]]; elig[q[0]] = True
  # ego pose (radar frame: x fwd, y left, yaw + left)
  tu, iu = np.unique(t, return_index=True); vu = v[iu]; wu = D['yaw'][iu]
  dt = np.r_[0, np.diff(tu)]; dt[dt > 0.5] = 0
  h = np.cumsum(wu * dt); px = np.cumsum(vu * np.cos(h) * dt); py = np.cumsum(vu * np.sin(h) * dt)
  def pose(tt):
    k = np.clip(np.searchsorted(tu, tt), 0, len(tu) - 1); return px[k], py[k], h[k], k
  sweeps = {}  # t -> row indices for context
  order_t = np.argsort(t, kind='stable'); ts = t[order_t]
  for inc in np.unique(D['inc_id'][elig]):
    r = np.where(D['inc_id'] == inc)[0]; e = r[elig[r]]; i0 = e[0]; te = t[i0]
    w = r[(t[r] >= te) & (t[r] <= te + 1.0)]
    # world position of the object at eligibility
    X0, Y0, H0, k0 = pose(te); ox = X0 + np.cos(H0) * d[i0] - np.sin(H0) * y[i0]; oy = Y0 + np.sin(H0) * d[i0] + np.cos(H0) * y[i0]
    ks = np.arange(k0, min(len(tu), k0 + 3000)); ex = np.cos(h[ks]) * (ox - px[ks]) + np.sin(h[ks]) * (oy - py[ks]); ey = -np.sin(h[ks]) * (ox - px[ks]) + np.cos(h[ks]) * (oy - py[ks])
    cross = np.where(ex <= 0)[0]; yc = ey[cross[0]] if len(cross) else np.nan; tc = tu[ks[cross[0]]] if len(cross) else np.nan; vc = vu[ks[cross[0]]] if len(cross) else np.nan
    # STOPPED_BEHIND: ego vEgo<1 within reach, with a track at 2-20 m |y|<1.5 within 2 m of the predicted object position
    lab = 'UNRESOLVED'; sb = False
    stop_k = ks[(vu[ks] < 1.0) & (ex > 0)]
    if len(stop_k):
      ts0 = tu[stop_k[0]]; kk = np.searchsorted(ts, ts0 - 0.1); kk2 = np.searchsorted(ts, ts0 + 1.0)
      q = order_t[kk:kk2]; ii = stop_k[0] - ks[0]
      c = q[(d[q] >= 2) & (d[q] <= 20) & (np.abs(y[q]) < 1.5) & (np.abs(d[q] - ex[ii]) <= 2 + 0.03 * ex[ii]) & (np.abs(y[q] - ey[ii]) <= 2)]
      sb = len(c) > 0
    if sb: lab = 'STOPPED_BEHIND'
    elif inc == forced: lab = 'FP42'; fin = r[-1]; fin_d, fin_y = d[fin], y[fin]
    else:
      after = r[t[r] >= te]; near = after[d[after] < 5]; fin = near[0] if len(near) else after[-1]
      if v[fin] > 3 and abs(y[fin]) > 1.2: lab = 'PASSED'
      fin_d, fin_y = d[fin], y[fin]
      fin_d, fin_y = d[fin], y[fin]
    if sb: fin = r[-1]; fin_d, fin_y = d[fin], y[fin]
    # context: other stationary tracks at the eligibility sweep
    kk = np.searchsorted(ts, te - 0.03); kk2 = np.searchsorted(ts, te + 0.03); q = order_t[kk:kk2]; q = q[D['inc_id'][q] != inc]
    ctx = int(((np.abs(y[q] - y[i0]) <= 3) & (np.abs(d[q] - d[i0]) <= 10) & stat[q]).sum())
    kap = D['yaw'][i0] / max(v[i0], 1)
    before = r[t[r] < te]
    rec = dict(vabs1_elig=D['vabs1'][i0], vabs1_1s=np.nanmean(D['vabs1'][w]), flag55_frac1s=fl[w].mean(), route=D['rid'][:8], tid=int(D['tid'][i0]), inc=int(D['inc'][i0]), t_elig=round(te, 2), label=lab, d_elig=d[i0], y_elig=y[i0], off_model=off[i0], beyond_end=int(beyond[i0]),
      path_end=pxe[i0], v_elig=v[i0], m_prob=D['m_prob'][i0], fin_d=fin_d, fin_y=fin_y, y_driven=yc, v_cross=vc, model_path_err=(yc - off[i0]) if np.isfinite(yc) else np.nan,
      kappa=kap, outside_curve=int(np.sign(off[i0] if not np.isfinite(yc) else yc) * np.sign(kap) < 0) if abs(kap) > 0.002 else 0, abs_kappa=abs(kap),
      ever_moving=int((np.abs(D['vabs1'][before]) > 2).any()) if len(before) else 0, birth_d=d[r[0]], born=int(D['born'][r[0]]), age_elig=te - t[r[0]], ctx_stat=ctx)
    feats = {}
    for k, a in X.items():
      feats['s_' + k] = a[i0]; feats['m_' + k] = np.nanmean(a[w]) if np.isfinite(a[w]).any() else np.nan
    for pre, ii in (('s_', [i0]), ('m_', w)):
      A = X['f3_AZIMUTH_EDGE_A_RAD'][ii]; B = X['f3_AZIMUTH_EDGE_B_RAD'][ii]
      feats[pre + 'az_extent'] = np.nanmean(np.abs(A - B)); feats[pre + 'width_m'] = np.nanmean(d[ii] * np.abs(A - B))
      bits = np.unpackbits(raw[ii], axis=2).astype(float); hv = have[ii]
      for fr in range(5):
        if hv[:, fr].any():
          b = bits[hv[:, fr], fr].mean(0)
          for bp in range(64): feats[f'{pre}raw_f{fr}_B{bp // 8}_b{7 - bp % 8}'] = b[bp]
    rec.update(feats); recs.append(rec)
np.save('/tmp/rv/stopshadow/clutter2_recs.npy', recs, allow_pickle=True)
from collections import Counter
print(Counter(r['label'] for r in recs)); print(Counter((r['route'], r['label']) for r in recs))
