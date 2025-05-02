#!/bin/bash

set -eu

# find the real linker, from env or same defaults as go
REAL_LD="${CC}"
if [[ -z "${REAL_LD}" ]]; then
	REAL_LD="$(which gcc)"
fi
if [[ -z "${REAL_LD}" ]]; then
	REAL_LD="$(which clang)"
fi


ARGS=("$@")

# list of linker arguments to ignore
STRIPPED_ARGS="-g -O2"

# list of arguments to prefix with -Bstatic
STATIC_ARGS="-lx264 -lnlopt -lstdc++"

# add explicit static standard library flags
FILTERED=("-static-libgcc" "-static-libstdc++")
# Loop through the arguments and filter
for ARG in "${ARGS[@]}"; do
	if [[ "${STRIPPED_ARGS[@]}" =~ "${ARG}" ]]; then
		# don't forward the arg
		:
	elif [[ "${STATIC_ARGS[@]}" =~ "${ARG}" ]]; then
		# wrap the arg as a static one
		FILTERED+=("-Wl,-Bstatic" "${ARG}" "-Wl,-Bdynamic")
	else
		# pass through with no filtering
		FILTERED+=("${ARG}")
	fi
done

# add libstdc++ statically (and last)
FILTERED+=("-Wl,-Bstatic" "-lstdc++" "-Wl,-Bdynamic")

# call the real linker with the filtered arguments

set -x

exec "$REAL_LD" "${FILTERED[@]}"
