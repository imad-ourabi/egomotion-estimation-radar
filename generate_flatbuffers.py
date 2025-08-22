#!/usr/bin/env python3
# This file is part of project link.developers/new-node-hugin.
# It is copyrighted by the contributors recorded in the version control history of the file,
# available from its original location git@github.com:imad-ourabi/new-node-hugin.git.
#
# SPDX-License-Identifier: MPL-2.0

import sys
import os
import subprocess

BFBS_PATH = "data"
PY_PATH = "new_node_hugin"
FBS_FILES = [
    "data/RadarPointCloud.fbs",
    "data/GenericMatrix3D.fbs",
    "data/EgoMotion.fbs",
]

errs = 0
for fbs in FBS_FILES:
    print("{}: Generating binary flatbuffer schema ...".format(fbs))
    cmd = ["flatc", "--binary", "--schema", "-o", BFBS_PATH, fbs]
    proc = subprocess.run(cmd)
    if proc.returncode != 0:
        errs += 1
        print("bfbs generation failed!")
        continue

    print("{}: Generating Python files ...".format(fbs))
    cmd = ["flatc", "--python", "-o", PY_PATH, fbs]
    proc = subprocess.run(cmd)
    if proc.returncode != 0:
        errs += 1
        print("bfbs generation failed!")
        continue

if errs == 0:
    print("Done. Files processed: {}".format(len(FBS_FILES)))
    sys.exit(0)
else:
    print("Errors: {}".format(errs))
    sys.exit(1)
