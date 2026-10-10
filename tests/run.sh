#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-only
set -euo pipefail
cd "$(dirname "$0")"
test_build_dir=$(mktemp -d)
trap 'rm -rf "$test_build_dir"' EXIT
for source in *_test.cpp; do
  "${CXX:-c++}" -std=c++17 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined \
    -fno-omit-frame-pointer -I stubs -I ../components/ecowitt_rf/sensor \
    "$source" -o "$test_build_dir/${source%.cpp}"
  "$test_build_dir/${source%.cpp}"
done
