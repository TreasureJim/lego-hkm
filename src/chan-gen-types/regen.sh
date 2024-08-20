#!/bin/sh

compiler="../../libs/chan/compiler/JuliaCompiler/generate_types.jl"
codegen="julia $compiler"  # Assuming Julia is the interpreter for the .jl script

found_files=false

for file in defs/*.jl; do
    if [ -f "$file" ]; then
        $codegen --lc -d $1/gen_c $file
        $codegen -l jlt -d $1/juliet-server/src/gen $file
        found_files=true
    fi
done

if [ "$found_files" = false ]; then
    echo "No .jl files found in defs/"
    exit 1
fi
