#!/bin/bash
cd `dirname $0`
tar zcvf src.tar.gz \
  --exclude src/aichallenge-bringup-final/image \
  --exclude src/aichallenge-bringup-final/data/simulator.pcd \
  --exclude src/aichallenge-bringup-final/.git \
  src
