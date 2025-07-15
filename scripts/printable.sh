#!/bin/bash

# Check arguments.
if [ $# -eq 0 ]; then
	echo "usage: $0 [CODE] [OUTPUT]"
	exit 1
fi

code_file=$1
output_file=$2
enscript_bin="/usr/bin/enscript"

# Check file exists.
if [ ! -f "$code_file" ]; then 
	echo "error: file '$code_file' not found"
	exit 1
fi

# Check file is readable.
if [ ! -r "$code_file" ]; then
	echo "error: file '$code_file' is not readable"
	exit 1
fi

# Run enscript executable.
"$enscript_bin" -o "$output_file" "$code_file"
