
make clean -f $1.Makefile
make -f $1.Makefile "${@:2}"
../build/apps/$1
