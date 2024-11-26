# run this command to see the config:
#  clang-format -style=Microsoft -dump-config

find .. -regex '.*\.\(cpp\|hpp\)' -exec clang-format -style=file -i {} \;
