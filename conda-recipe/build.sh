#!/usr/bin/env bash
# This file is part of project link.developers/new-node-hugin.
# It is copyrighted by the contributors recorded in the version control history of the file,
# available from its original location git@github.com:imad-ourabi/new-node-hugin.git.
#
# SPDX-License-Identifier: MPL-2.0

"$PYTHON" ./generate_flatbuffers.py

"$PYTHON" setup.py install

STATIC_ASSET_PATH="$PREFIX/share/link2/static_assets/new-node-hugin"
mkdir -p "$STATIC_ASSET_PATH/data"
cp -t "$STATIC_ASSET_PATH" specification.json
cp -t "$STATIC_ASSET_PATH/data" -r data/*.bfbs
