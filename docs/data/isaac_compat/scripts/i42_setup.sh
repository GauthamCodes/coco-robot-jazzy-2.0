#!/usr/bin/env bash
# Isolated Isaac Sim 4.2.0 install. Nothing outside ~/isaacsim-4.2.0-test is written
# (pip cache and pip's own state live inside it).
set -u
T=/home/gautham/isaacsim-4.2.0-test
MODE=${1:-dry}
mkdir -p $T/pipcache $T/home $T/tmp
[ -x $T/venv/bin/python ] || /usr/bin/python3.10 -m venv $T/venv
export PIP_CACHE_DIR=$T/pipcache PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONNOUSERSITE=1
PKGS="isaacsim==4.2.0.2 isaacsim-extscache-physics==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2"
if [ "$MODE" = dry ]; then
  env -i HOME=$T/home PATH=/usr/bin:/bin PIP_CACHE_DIR=$PIP_CACHE_DIR PYTHONNOUSERSITE=1 TMPDIR=$T/tmp \
    $T/venv/bin/pip install --dry-run --quiet --report $T/dryrun.json $PKGS --extra-index-url https://pypi.nvidia.com 2>&1 | tail -5
  python3 -c "
import json; r=json.load(open('$T/dryrun.json'))
items=r['install']; print(len(items),'packages')
big=[(i['metadata']['name'],i['metadata']['version']) for i in items if i['metadata']['name'].startswith(('isaacsim','omniverse','torch','nvidia'))]
print(sorted(big))"
else
  env -i HOME=$T/home PATH=/usr/bin:/bin PIP_CACHE_DIR=$PIP_CACHE_DIR PYTHONNOUSERSITE=1 TMPDIR=$T/tmp \
    $T/venv/bin/pip install --upgrade pip > $T/pip_upgrade.log 2>&1
  env -i HOME=$T/home PATH=/usr/bin:/bin PIP_CACHE_DIR=$PIP_CACHE_DIR PYTHONNOUSERSITE=1 TMPDIR=$T/tmp \
    /usr/bin/time -v $T/venv/bin/pip install $PKGS --extra-index-url https://pypi.nvidia.com > $T/pip_install.log 2>&1
  echo "pip exit=$?"; tail -3 $T/pip_install.log; grep -E 'Elapsed' $T/pip_install.log
  du -sh $T/venv $T/pipcache
fi
