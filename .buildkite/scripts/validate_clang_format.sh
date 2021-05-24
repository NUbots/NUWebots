#!/bin/bash

check_formatting() {
    echo "Validating formatting for $1"

    # This option means that it will return the exit-code of the first nonzero command
    # Or if all are 0 return 0. This ensures that if any of the commands here fail the
    # formatting test will still fail
    set -o pipefail

    # Check the formatting
    clang-format -style=file "$1" | colordiff --color=yes -u "$1" -

    # Return 1 on failure, 0 on success
    if [ $? -eq 0 ]; then
        return 0
    else
        return 1
    fi
}
export -f check_formatting

# Loop through all c/cpp/proto files and check validation
# We don't want to format the webots proto files though - protos/* are skipped 
git ls-files | grep '.*\.\(c\|cc\|cpp\|cxx\|hpp\|ipp\|proto\)$' | grep -v '^protos/' \
    | parallel --joblog /var/tmp/formatting.log -j$(nproc) check_formatting

# Count how many returned a non zero exist status
ret=$(tail -n +2 /var/tmp/formatting.log | awk '{ sum += $7; } END {print sum}')

echo "$ret files are not formatted correctly"
exit $ret
