#!/bin/sh
# Download the micromouseonline/mazefiles collection into third_party/ (not
# committed: the collection has no licence file). Then run e.g.
#   ./tools/sim/sim third_party/mazefiles/classic/*.txt
set -e
cd "$(dirname "$0")/.."
if [ -d third_party/mazefiles/.git ]; then
  git -C third_party/mazefiles pull --ff-only
else
  mkdir -p third_party
  git clone --depth 1 https://github.com/micromouseonline/mazefiles.git third_party/mazefiles
fi
