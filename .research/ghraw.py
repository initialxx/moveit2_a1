#!/usr/bin/env python3
"""Fetch raw.githubusercontent.com files with caching. Usage: ghraw.py owner/repo branch path [path...]"""
import os, sys, urllib.request

CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cache")
os.makedirs(CACHE, exist_ok=True)

repo, branch = sys.argv[1], sys.argv[2]
for p in sys.argv[3:]:
    url = f"https://raw.githubusercontent.com/{repo}/{branch}/{p}"
    key = (repo + "_" + branch + "_" + p).replace("/", "_")
    path = os.path.join(CACHE, key)
    if os.path.exists(path):
        txt = open(path).read()
    else:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "research"})
            with urllib.request.urlopen(req, timeout=60) as r:
                txt = r.read().decode("utf-8", "replace")
            open(path, "w").write(txt)
        except Exception as e:
            print(f"!!! FAILED {url}: {e}\n")
            continue
    print(f"===== {url} =====")
    print(txt)
    print()
