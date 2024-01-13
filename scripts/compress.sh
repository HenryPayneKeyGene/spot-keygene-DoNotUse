#!/usr/bin/bash

BASE_DIR=$(dirname "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )")
echo "in dir $BASE_DIR"

pushd $BASE_DIR || exit
	tar cvf kg.tar.xz --use-compress-program='xz -1T0' autowalks LICENSE pyproject.toml README.md *.py *.txt scripts spotkg tags .gitignore .gitmodules
popd || exit
