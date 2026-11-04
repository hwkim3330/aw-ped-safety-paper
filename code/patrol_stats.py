#!/usr/bin/env python3
"""What the patrol's driving came to, per day, from ~/.cache/aw-demo/patrol_legs.jsonl (patrol.py writes one line a leg):
legs, arrived (share), km driven, hours, stuck / MRM / route lost / driver takeovers, and km per intervention.
    python3 patrol_stats.py [--since 2026-11-02]"""
import argparse
import collections
import json
import os

ap = argparse.ArgumentParser()
ap.add_argument('--since', default='')
ap.add_argument('--file', default=os.path.expanduser('~/.cache/aw-demo/patrol_legs.jsonl'))
a = ap.parse_args()

days = collections.defaultdict(list)
for line in open(a.file):
    if line.strip():
        r = json.loads(line)
        if r['start'] >= a.since:
            days[r['start'][:10]].append(r)

print(f"{'day':<11} {'legs':>5} {'arrived':>10} {'km':>7} {'h':>5} {'stuck':>6} {'mrm':>4} {'lost':>5} {'takeover':>9} {'km/intervention':>16}")
for day, rs in sorted(days.items()):
    n = len(rs)
    c = collections.Counter(r['outcome'] for r in rs)
    km = sum(r['m'] for r in rs) / 1000
    h = sum(r['secs'] for r in rs) / 3600
    take = sum(1 for r in rs if r.get('local'))
    inter = c['stuck'] + c['mrm'] + take
    print(f"{day:<11} {n:>5} {c['arrived']:>4} ({100 * c['arrived'] / n:3.0f}%) {km:>7.1f} {h:>5.1f} {c['stuck']:>6} {c['mrm']:>4} {c['lost']:>5} {take:>9} "
          f"{(km / inter if inter else float('inf')):>16.1f}")
