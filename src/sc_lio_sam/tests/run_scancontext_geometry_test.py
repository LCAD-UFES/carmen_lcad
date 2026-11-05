#!/usr/bin/env python3
"""Test the compiled Scan Context object using the mapper's build dependencies."""
from pathlib import Path
import shlex
import subprocess
import tempfile

workspace = Path(__file__).resolve().parents[1]
build = workspace / 'build/lio_sam'
target = build / 'CMakeFiles/lio_sam_mapOptmization.dir'
flags = (target / 'flags.make').read_text()
includes = next(line.split(' = ', 1)[1] for line in flags.splitlines()
                if line.startswith('CXX_INCLUDES ='))
with tempfile.TemporaryDirectory(prefix='scancontext-test-') as temporary:
    output = Path(temporary)
    obj = output / 'geometry.o'
    executable = output / 'geometry'
    subprocess.run(['/usr/bin/c++', '-std=c++17'] + shlex.split(includes) +
                   ['-c', str(workspace / 'tests/scancontext_geometry_test.cpp'), '-o', str(obj)], check=True)
    args = shlex.split((target / 'link.txt').read_text())
    args = [word for word in args if not word.startswith('-Wl,--dependency-file=')]
    args[args.index('CMakeFiles/lio_sam_mapOptmization.dir/src/mapOptmization.cpp.o')] = str(obj)
    args[args.index('-o') + 1] = str(executable)
    subprocess.run(args, cwd=build, check=True)
    subprocess.run([str(executable)], check=True)
