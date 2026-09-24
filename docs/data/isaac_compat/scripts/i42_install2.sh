#!/usr/bin/env bash
# Second attempt: reuse the complete Isaac/Kit wheels, pin release-era torch.
T=/home/gautham/isaacsim-4.2.0-test
mkdir -p $T/wheels
#find $T/tmp -name 'isaacsim*-4.2.0.2-*.whl' -exec mv {} $T/wheels/ \;
#find $T/tmp -name 'omniverse_kit-*.whl' -exec mv {} $T/wheels/ \;
rm -rf $T/tmp/*
ls $T/wheels | wc -l; du -sh $T/wheels
PKGS="torch==2.4.0 $(ls $T/wheels/*.whl | tr "\n" " ")"
env -i HOME=$T/home PATH=/usr/bin:/bin PIP_CACHE_DIR=$T/pipcache PYTHONNOUSERSITE=1 TMPDIR=$T/tmp \
  PIP_DISABLE_PIP_VERSION_CHECK=1 \
  /usr/bin/time -v $T/venv/bin/pip install $PKGS --find-links $T/wheels \
  --extra-index-url https://pypi.nvidia.com > $T/pip_install2.log 2>&1
echo "pip exit=$?"
tail -3 $T/pip_install2.log | cut -c1-300
grep -E 'Elapsed' $T/pip_install2.log
du -sh $T/venv
