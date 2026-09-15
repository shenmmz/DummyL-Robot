import subprocess, os, sys

ROOT = r"C:/Users/111/Desktop/DummyL-Robot"
GCC = r"C:/Qt/Tools/mingw1310_64/bin/gcc.exe"
os.chdir(ROOT)

src = [
    r"tests/movl_verify.c",
    r"src/kinematics/dh.c",
    r"src/kinematics/ik.c",
    r"src/trajectory/line.c",
]
out = r"tests/movl_verify.exe"
cmd = [GCC, "-O2", *src, "-Isrc", "-lm", "-o", out]
print("BUILD:", " ".join(cmd))
r = subprocess.run(cmd, capture_output=True, text=True)
print(r.stdout)
print(r.stderr, end="")
if r.returncode != 0:
    print("BUILD FAILED rc=", r.returncode)
    sys.exit(r.returncode)

print("=== RUN ===")
r2 = subprocess.run([out], capture_output=True, text=True)
print(r2.stdout, end="")
print(r2.stderr, end="")
sys.exit(r2.returncode)
