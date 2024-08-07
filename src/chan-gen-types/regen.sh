#!/bin/sh

dest="gen_c/"
codegen="../../libs/chan/compiler/JuliaCompiler/generate_types.jl --lc -d $dest"

mkdir -p $dest

for file in defs/*.jl; do
    # Check if the file exists (handles case where there are no .jl files)
    if [ -f "$file" ]; then
	$codegen $file
    else
        echo "No .jl files found in $directory"
        exit 1
    fi
done
