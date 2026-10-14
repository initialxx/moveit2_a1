#!/usr/bin/env python3
"""Fetch GitHub repo trees (cached) and filter for interesting paths."""
import json, os, sys, urllib.request

CACHE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cache")
os.makedirs(CACHE, exist_ok=True)


def api(url):
    key = url.replace("https://api.github.com/", "").replace("/", "_").replace("?", "_")
    path = os.path.join(CACHE, key + ".json")
    if os.path.exists(path):
        return json.load(open(path))
    req = urllib.request.Request(url, headers={
        "Accept": "application/vnd.github+json",
        "User-Agent": "research-script",
    })
    with urllib.request.urlopen(req, timeout=60) as r:
        data = json.load(r)
    json.dump(data, open(path, "w"))
    return data


def tree(repo, branch):
    d = api(f"https://api.github.com/repos/{repo}/git/trees/{branch}?recursive=1")
    if "tree" not in d:
        print(f"!! {repo}@{branch}: {d.get('message')}")
        return None
    return d


def blobs(d):
    return [t["path"] for t in d["tree"] if t["type"] == "blob"]


if __name__ == "__main__":
    repo, branch = sys.argv[1], sys.argv[2]
    pats = sys.argv[3:] or ["hardware_interface", "ros2_control", "package.xml",
                            "controllers.yaml", "moveit", "launch"]
    d = tree(repo, branch)
    if not d:
        sys.exit(1)
    paths = blobs(d)
    print(f"### {repo}@{branch}  total blobs={len(paths)} truncated={d.get('truncated')}")
    pkgs = sorted({p.rsplit("/package.xml", 1)[0] for p in paths if p.endswith("package.xml")})
    print("\n--- PACKAGES ---")
    for p in pkgs:
        print(" ", p or "<root>")
    print("\n--- MATCHES ---")
    for p in paths:
        if any(k.lower() in p.lower() for k in pats):
            print(" ", p)
