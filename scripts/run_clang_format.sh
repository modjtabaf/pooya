# run this command to see the config:
#  clang-format -style=Microsoft -dump-config

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
find $SCRIPT_DIR/../src -regex '.*\.\(cpp\|hpp\)' -exec clang-format -style=file -i {} \;
