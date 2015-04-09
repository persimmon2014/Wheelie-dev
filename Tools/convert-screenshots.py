#!/usr/bin/python

import os.path
import os
import sys
import time
import glob
from multiprocessing import Pool
import subprocess

def do(f):
    p = subprocess.Popen(["./dump-to-png", f])
    p.wait()

def do_remove(f):
    p = subprocess.Popen(["./dump-to-png", f])
    p.wait()
    os.remove(f)

if __name__ == '__main__':
    usage_str = """Usage: convert-screenshots [nprocesses] [--remove-after]"""
    nprocs = 1
    remove = False
    if len(sys.argv) > 1:
        tmp = int(sys.argv[1])
        if(tmp > 0):
            nprocs = tmp
        else:
            print >> os.stderr, usage_str
            sys.exit(1)
    if len(sys.argv) == 3 and sys.argv[2] == '--remove-after':
        remove = True

    print "Using", nprocs, "threads"
    if not remove:
        print "Not removing",
    else:
        print "Removing",
    print "dump files after conversion"
    pool = Pool(processes=nprocs)
    while True:
        print "Polling"
        dumps = glob.glob("*.dump")
        pngs  = glob.glob("*.png")
        dumps = set((os.path.splitext(d)[0] for d in dumps))
        pngs  = set((os.path.splitext(p)[0] for p in pngs))
        res   = [ d + ".dump" for d in dumps.difference(pngs) ]
        if(len(res) > 0):
            print "Dispatching on", len(res), "jobs"
            if remove:
                w = pool.map_async(do_remove, res)
            else:
                w = pool.map_async(do, res)
            w.wait()
            print "Finished pass"
        else:
            time.sleep(5)


