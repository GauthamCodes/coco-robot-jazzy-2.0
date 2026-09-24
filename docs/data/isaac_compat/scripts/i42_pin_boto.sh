#!/usr/bin/env bash
# Kit 106.1 pre-bundles botocore 1.34.68 ahead of site-packages; an unpinned
# install pulled s3transfer 0.19.2 (2026), which needs a newer botocore.
T=/home/gautham/isaacsim-4.2.0-test
env -i HOME=$T/home PATH=/usr/bin:/bin TMPDIR=$T/tmp PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
  $T/venv/bin/pip install -q "boto3==1.34.68" "botocore==1.34.68" "s3transfer>=0.10.0,<0.11" 2>&1 | tail -3
$T/venv/bin/pip list 2>/dev/null | grep -iE "^(boto3|botocore|s3transfer) "
