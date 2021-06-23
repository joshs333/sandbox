#!/usr/bin/env bash
# Makes sure JD works from any directory
__dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
__file_path="${__dir}/$(basename "${BASH_SOURCE[0]}")"
__file_name="$(basename "${BASH_SOURCE[0]}")"
__call_dir=$(pwd)

export JD_ROOT=$__dir/jd/src
if [ ! `echo :$PATH: | grep -F :$JD_ROOT:` ]; then
   export PATH=$JD_ROOT:$PATH
fi

#if [[ -z $DISPLAY ]]; then
export DISPLAY=`hostname`:0
#fi
