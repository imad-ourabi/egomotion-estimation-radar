REM This file is part of project link.developers/new-node-hugin.
REM It is copyrighted by the contributors recorded in the version control history of the file,
REM available from its original location git@github.com:imad-ourabi/new-node-hugin.git.
REM
REM SPDX-License-Identifier: MPL-2.0

"%PYTHON%" generate_flatbuffers.py

"%PYTHON%" setup.py install

set STATIC_ASSET_PATH=%LIBRARY_PREFIX%\share\link2\static_assets\new-node-hugin
mkdir "%STATIC_ASSET_PATH%\data"
copy specification.json "%STATIC_ASSET_PATH%\"
xcopy data\*.bfbs "%STATIC_ASSET_PATH%\data\" /s
