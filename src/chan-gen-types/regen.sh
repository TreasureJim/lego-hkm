#!/bin/sh

compiler="../../libs/chan/compiler/JuliaCompiler/generate_types.jl"
codegen="julia $compiler"  # Assuming Julia is the interpreter for the .jl script

mkdir -p gen_c

found_files=false

for file in defs/*.jl; do
    if [ -f "$file" ]; then
        $codegen --lc -d gen_c $file
        $codegen -l jlt -d ../juliet-server/src/gen_types $file
        found_files=true
    fi
done

if [ "$found_files" = false ]; then
    echo "No .jl files found in defs/"
    exit 1
fi
