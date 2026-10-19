#!/usr/bin/env python3
"""
Compare announced MCM trajectories with the driven trajectories (SUMO FCD).

This is the noise floor of thesis Section 6.3 at full time resolution: for every
FCD sample of a vehicle, the reference trajectory of the latest MCM which that
vehicle had transmitted is interpolated at the sample time, and the error is split
along (e_lon) and across (e_lat) the announced trajectory. Samples beyond the
horizon of the latest MCM (e.g. after a silent abort) are skipped.

usage: announcement_error.py <results dir> <run prefix, e.g. ground-truth-0>
"""
import csv
import math
import sys
import xml.etree.ElementTree as ET
from bisect import bisect_right
from collections import defaultdict

STEP_LENGTH = 0.1  # SUMO step length of the scenario

A = 6378137.0
F = 1 / 298.257223563
E2 = F * (2 - F)


def to_enu(ref_lat, ref_lon, lat, lon):
    s = math.sin(math.radians(ref_lat))
    w = 1 - E2 * s * s
    n = A / math.sqrt(w)
    m = A * (1 - E2) / (w * math.sqrt(w))
    return (math.radians(lon - ref_lon) * n * math.cos(math.radians(ref_lat)),
            math.radians(lat - ref_lat) * m)


def parse_points(text):
    points = []
    for item in text.split(';') if text else []:
        t, e, n, v, h = (int(x) for x in item.split(':'))
        points.append((t / 1000, e / 100, n / 100, v / 100, h / 10))
    return points


def sample(points, t):
    """announced position and secant heading at t seconds after generation"""
    t0, e0, n0 = 0.0, 0.0, 0.0
    for (t1, e1, n1, _, h1) in points:
        if t <= t1:
            f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
            de, dn = e1 - e0, n1 - n0
            heading = math.degrees(math.atan2(de, dn)) if math.hypot(de, dn) > 1e-3 else h1
            return (e0 + f * de, n0 + f * dn), heading
        t0, e0, n0 = t1, e1, n1
    return None, None


def percentiles(values, ps=(50, 95, 99, 100)):
    if not values:
        return {}
    values = sorted(values)
    return {p: values[min(len(values) - 1, int(math.ceil(p / 100 * len(values))) - 1)] for p in ps}


def main(results, run):
    stations = {}
    with open(f'{results}/{run}-stations.csv') as f:
        for row in csv.DictReader(f):
            stations[int(row['station_id'])] = row['vehicle_id']

    # transmitted MCMs per station, ordered by transmission time
    tx = defaultdict(list)
    with open(f'{results}/{run}-mcm-tx.csv') as f:
        for row in csv.DictReader(f):
            row['gen'] = float(row['generation_time'])
            tx[int(row['station_id'])].append(row)

    errors = defaultdict(lambda: ([], []))
    by_vehicle = {v: s for s, v in stations.items()}
    times_cache = {s: [float(m['time']) for m in msgs] for s, msgs in tx.items()}

    for _, elem in ET.iterparse(f'{results}/{run}-fcd.xml', events=('end',)):
        if elem.tag != 'timestep':
            continue
        # SUMO labels an FCD timestep with the start of the step, but the positions
        # are those after the step, which TraCI (and thus CAM and MCM) reports
        # with the end time of the step
        t = float(elem.get('time')) + STEP_LENGTH
        for veh in elem.findall('vehicle'):
            station = by_vehicle.get(veh.get('id'))
            if station is None:
                continue
            i = bisect_right(times_cache[station], t) - 1
            if i < 0:
                continue
            m = tx[station][i]
            if not m['reference']:
                continue
            points = parse_points(m['reference'])
            dt = t - m['gen']
            p, heading = sample(points, dt)
            if p is None:
                continue
            ref_lat, ref_lon = int(m['latitude']) * 1e-7, int(m['longitude']) * 1e-7
            obs = to_enu(ref_lat, ref_lon, float(veh.get('y')), float(veh.get('x')))
            de, dn = obs[0] - p[0], obs[1] - p[1]
            h = math.radians(heading)
            e_lon = de * math.sin(h) + dn * math.cos(h)
            e_lat = -de * math.cos(h) + dn * math.sin(h)
            kind = {'0': 'keep lane', '1': 'lane change', '2': 'lane change', '3': 'lane change',
                    '4': 'yield'}[m['reference_type']]
            errors[kind][0].append(abs(e_lat))
            errors[kind][1].append(abs(e_lon))
            errors['all'][0].append(abs(e_lat))
            errors['all'][1].append(abs(e_lon))
        elem.clear()

    print(f'{"announcement":<12} {"samples":>8}   |e_lat| p50/p95/p99/max [m]      |e_lon| p50/p95/p99/max [m]')
    for kind, (lat, lon) in sorted(errors.items()):
        pl, po = percentiles(lat), percentiles(lon)
        print(f'{kind:<12} {len(lat):>8}   ' + ' / '.join(f'{pl[p]:.3f}' for p in pl) +
              '   ' + ' / '.join(f'{po[p]:.3f}' for p in po))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
